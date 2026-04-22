#include "player/rtcm3_nav.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

extern "C" {
#include "gpssim.h"
}

namespace {

struct Options {
  std::string navfile;
  bool have_llh = false;
  bool have_xyz = false;
  double llh[3] = {0.0, 0.0, 0.0};
  double xyz[3] = {0.0, 0.0, 0.0};
  bool have_time = false;
  datetime_t time = {0, 0, 0, 0, 0, 0.0};
  int leap_sec = 18;
  std::string rtcm_host;
  int rtcm_port = 5018;
  std::string rtcm_mount;
  std::string rtcm_user;
  int rtcm_timeout_ms = 3000;
  double rtcm_warmup_sec = 5.0;
  int top = 12;
  bool show_all = false;
  int manual_observed[MAX_SAT] = {0};
  int manual_observed_count = 0;
};

struct Candidate {
  int prn = 0;
  bool rinex_current = false;
  double rinex_current_el = NAN;
  double nearest_toe = NAN;
  bool rtcm_eph = false;
  double rtcm_current_el = NAN;
  bool live_obs = false;
  bool scan_ok = false;
  double scan_delta = NAN;
  double scan_el = NAN;
  gpstime_t scan_toe = {0, 0.0};
  bool best_ok = false;
  double best_delta = NAN;
  double best_el = NAN;
  gpstime_t best_toe = {0, 0.0};
};

static void usage(const char *prog) {
  std::fprintf(
      stderr,
      "Usage: %s -e <rinex_nav> (-l lat,lon,h | -c x,y,z) [options]\n"
      "\n"
      "Options:\n"
      "  -t YYYY/MM/DD,hh:mm:ss       Scenario/current GPS-calendar time\n"
      "  --rtcm-host <host>           RTCM TCP/NTRIP host\n"
      "  --rtcm-port <port>           RTCM port (default 5018)\n"
      "  --rtcm-mount <name>          Optional NTRIP mount point\n"
      "  --rtcm-user <user[:pass]>    Optional NTRIP credentials\n"
      "  --rtcm-warmup-sec <sec>      Live sample duration (default 5)\n"
      "  --rtcm-timeout-ms <ms>       RTCM read/connect timeout (default 3000)\n"
      "  --obs-prns <list>            Manual live-observed PRNs, e.g. 5,6,11\n"
      "  --top <n>                    Rows to print (default 12)\n"
      "  --all                        Print every PRN\n"
      "\n"
      "Example:\n"
      "  %s -e hour1120.26n -l 21.0047844,105.8460541,5 \\\n"
      "    --rtcm-host 192.168.5.245 --rtcm-port 5005 \\\n"
      "    --rtcm-mount NAVIS --rtcm-user NAVIS:navis123\n",
      prog, prog);
}

static bool parseDouble3(const char *s, double *a, double *b, double *c) {
  char tail = '\0';
  return std::sscanf(s, "%lf,%lf,%lf%c", a, b, c, &tail) == 3;
}

static bool parseTime(const char *s, datetime_t *t) {
  char tail = '\0';
  return std::sscanf(s, "%d/%d/%d,%d:%d:%lf%c", &t->y, &t->m, &t->d, &t->hh,
                     &t->mm, &t->sec, &tail) == 6;
}

static bool parsePrnList(const char *s, int observed[MAX_SAT], int *count) {
  const char *p = s;
  int n = 0;

  std::memset(observed, 0, sizeof(int) * MAX_SAT);

  while (*p != '\0') {
    char *end = nullptr;
    long prn = std::strtol(p, &end, 10);

    if (end == p || prn < 1 || prn > MAX_SAT)
      return false;

    if (observed[prn - 1] != TRUE) {
      observed[prn - 1] = TRUE;
      n++;
    }

    if (*end == '\0') {
      p = end;
    } else if (*end == ',') {
      p = end + 1;
    } else {
      return false;
    }
  }

  *count = n;
  return true;
}

static bool parseOptions(int argc, char **argv, Options *opt) {
  for (int i = 1; i < argc; i++) {
    const char *arg = argv[i];

    if (std::strcmp(arg, "-e") == 0 && i + 1 < argc) {
      opt->navfile = argv[++i];
    } else if (std::strcmp(arg, "-l") == 0 && i + 1 < argc) {
      double lat = 0.0, lon = 0.0, h = 0.0;
      if (!parseDouble3(argv[++i], &lat, &lon, &h)) {
        std::fprintf(stderr, "ERROR: -l must be lat,lon,h.\n");
        return false;
      }
      opt->llh[0] = lat * D2R;
      opt->llh[1] = lon * D2R;
      opt->llh[2] = h;
      opt->have_llh = true;
    } else if (std::strcmp(arg, "-c") == 0 && i + 1 < argc) {
      if (!parseDouble3(argv[++i], &opt->xyz[0], &opt->xyz[1],
                        &opt->xyz[2])) {
        std::fprintf(stderr, "ERROR: -c must be x,y,z.\n");
        return false;
      }
      opt->have_xyz = true;
    } else if (std::strcmp(arg, "-t") == 0 && i + 1 < argc) {
      if (!parseTime(argv[++i], &opt->time)) {
        std::fprintf(stderr, "ERROR: -t must be YYYY/MM/DD,hh:mm:ss.\n");
        return false;
      }
      opt->have_time = true;
    } else if (std::strcmp(arg, "--rtcm-host") == 0 && i + 1 < argc) {
      opt->rtcm_host = argv[++i];
    } else if (std::strcmp(arg, "--rtcm-port") == 0 && i + 1 < argc) {
      opt->rtcm_port = std::atoi(argv[++i]);
    } else if (std::strcmp(arg, "--rtcm-mount") == 0 && i + 1 < argc) {
      opt->rtcm_mount = argv[++i];
    } else if (std::strcmp(arg, "--rtcm-user") == 0 && i + 1 < argc) {
      opt->rtcm_user = argv[++i];
    } else if (std::strcmp(arg, "--rtcm-timeout-ms") == 0 && i + 1 < argc) {
      opt->rtcm_timeout_ms = std::atoi(argv[++i]);
    } else if (std::strcmp(arg, "--rtcm-warmup-sec") == 0 && i + 1 < argc) {
      opt->rtcm_warmup_sec = std::atof(argv[++i]);
    } else if (std::strcmp(arg, "--obs-prns") == 0 && i + 1 < argc) {
      if (!parsePrnList(argv[++i], opt->manual_observed,
                        &opt->manual_observed_count)) {
        std::fprintf(stderr, "ERROR: --obs-prns must be comma-separated PRNs.\n");
        return false;
      }
    } else if (std::strcmp(arg, "--top") == 0 && i + 1 < argc) {
      opt->top = std::atoi(argv[++i]);
    } else if (std::strcmp(arg, "--all") == 0) {
      opt->show_all = true;
    } else if (std::strcmp(arg, "--help") == 0 ||
               std::strcmp(arg, "-h") == 0) {
      usage(argv[0]);
      std::exit(0);
    } else {
      std::fprintf(stderr, "ERROR: unknown argument: %s\n", arg);
      return false;
    }
  }

  if (opt->navfile.empty()) {
    std::fprintf(stderr, "ERROR: -e <rinex_nav> is required.\n");
    return false;
  }
  if (!opt->have_llh && !opt->have_xyz) {
    std::fprintf(stderr, "ERROR: provide -l lat,lon,h or -c x,y,z.\n");
    return false;
  }
  if (opt->have_llh && opt->have_xyz) {
    std::fprintf(stderr, "ERROR: use only one of -l or -c.\n");
    return false;
  }
  if (opt->rtcm_port <= 0 || opt->rtcm_port > 65535) {
    std::fprintf(stderr, "ERROR: --rtcm-port must be 1-65535.\n");
    return false;
  }
  if (opt->rtcm_timeout_ms < 100 || opt->rtcm_timeout_ms > 30000) {
    std::fprintf(stderr, "ERROR: --rtcm-timeout-ms must be 100-30000.\n");
    return false;
  }
  if (opt->rtcm_warmup_sec < 0.0 || opt->rtcm_warmup_sec > 300.0) {
    std::fprintf(stderr, "ERROR: --rtcm-warmup-sec must be 0-300.\n");
    return false;
  }
  if (opt->top < 1 || opt->top > MAX_SAT)
    opt->top = MAX_SAT;

  return true;
}

static gpstime_t currentGpsTime(int leap_sec) {
  using clock = std::chrono::system_clock;
  auto now = clock::now();
  auto sec = std::chrono::time_point_cast<std::chrono::seconds>(now);
  double frac =
      std::chrono::duration<double>(now - sec).count();
  std::time_t unix_sec = clock::to_time_t(sec);
  std::time_t gps_like = unix_sec + leap_sec;
  std::tm *gmt = std::gmtime(&gps_like);
  datetime_t t;
  gpstime_t g;

  t.y = gmt->tm_year + 1900;
  t.m = gmt->tm_mon + 1;
  t.d = gmt->tm_mday;
  t.hh = gmt->tm_hour;
  t.mm = gmt->tm_min;
  t.sec = (double)gmt->tm_sec + frac;
  date2gps(&t, &g);

  return g;
}

static double elevationDeg(ephem_t eph, gpstime_t t, const double xyz[3]) {
  double azel[2] = {0.0, 0.0};

  if (checkSatVisibility(eph, t, const_cast<double *>(xyz), -90.0, azel) != 1)
    return NAN;

  return azel[1] * R2D;
}

static bool findNearest(const ephem_t eph[][MAX_SAT], int n_sets, int prn,
                        gpstime_t t, ephem_t *out, double *abs_dt) {
  int sv = prn - 1;
  bool found = false;
  double best = 0.0;

  for (int i = 0; i < n_sets; i++) {
    double dt;

    if (eph[i][sv].vflg != 1)
      continue;

    dt = std::fabs(subGpsTime(eph[i][sv].toe, t));
    if (!found || dt < best) {
      found = true;
      best = dt;
      *out = eph[i][sv];
    }
  }

  if (found && abs_dt != nullptr)
    *abs_dt = best;

  return found;
}

static void listPrns(const char *label, const int flags[MAX_SAT]) {
  bool any = false;

  std::printf("%s", label);
  for (int sv = 0; sv < MAX_SAT; sv++) {
    if (flags[sv] == TRUE) {
      std::printf("%s%d", any ? "," : "", sv + 1);
      any = true;
    }
  }
  if (!any)
    std::printf("(none)");
  std::printf("\n");
}

static bool sampleRtcm(const Options &opt, rtcm3_nav_stream_t *stream,
                       int observed[MAX_SAT]) {
  rtcm3_nav_options_t rtcm_opt;
  char err[RTCM3_NAV_ERR_SIZE] = "";
  auto start = std::chrono::steady_clock::now();

  if (opt.rtcm_host.empty()) {
    std::memcpy(observed, opt.manual_observed, sizeof(int) * MAX_SAT);
    return true;
  }

  std::memset(&rtcm_opt, 0, sizeof(rtcm_opt));
  rtcm_opt.host = opt.rtcm_host.c_str();
  rtcm_opt.port = opt.rtcm_port;
  rtcm_opt.timeout_ms = opt.rtcm_timeout_ms;
  rtcm_opt.mount_point =
      opt.rtcm_mount.empty() ? nullptr : opt.rtcm_mount.c_str();
  rtcm_opt.credentials = opt.rtcm_user.empty() ? nullptr : opt.rtcm_user.c_str();

  if (rtcm3_nav_open(stream, &rtcm_opt, err, sizeof(err)) != TRUE) {
    std::fprintf(stderr, "ERROR: RTCM open failed: %s\n", err);
    return false;
  }

  while (true) {
    int updated = FALSE;
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - start).count();
    int wait_ms = 200;

    if (elapsed >= opt.rtcm_warmup_sec)
      break;

    if (rtcm3_nav_pump(stream, wait_ms, &updated, err, sizeof(err)) != TRUE) {
      std::fprintf(stderr, "ERROR: RTCM read failed: %s\n", err);
      rtcm3_nav_close(stream);
      return false;
    }
  }

  rtcm3_nav_copy_observed(stream, observed);
  for (int sv = 0; sv < MAX_SAT; sv++) {
    if (opt.manual_observed[sv] == TRUE)
      observed[sv] = TRUE;
  }

  return true;
}

static const char *statusFor(const Candidate &c, bool have_live_obs) {
  if (c.live_obs)
    return "LIVE_OBS";
  if (!c.scan_ok)
    return "NO_REVIVE";
  if (!have_live_obs && c.rinex_current && c.rinex_current_el >= 0.0)
    return "CUR_UP";
  return "OK";
}

static int sortBucket(const Candidate &c) {
  if (c.scan_ok && !c.live_obs)
    return 0;
  if (c.scan_ok)
    return 1;
  return 2;
}

static void printDouble(double value, const char *suffix = "") {
  if (std::isnan(value))
    std::printf("   nan%s", suffix);
  else
    std::printf("%6.1f%s", value, suffix);
}

} // namespace

int main(int argc, char **argv) {
  Options opt;
  ephem_t eph[EPHEM_ARRAY_SIZE][MAX_SAT];
  ionoutc_t ionoutc;
  rtcm3_nav_stream_t rtcm_stream;
  int observed[MAX_SAT] = {0};
  double rx_xyz[3] = {0.0, 0.0, 0.0};
  gpstime_t t_now;
  int n_sets;
  std::vector<Candidate> rows;
  bool have_live_obs = false;

  rtcm3_nav_init(&rtcm_stream);

  if (!parseOptions(argc, argv, &opt)) {
    usage(argv[0]);
    return 2;
  }

  if (opt.have_llh)
    llh2xyz(opt.llh, rx_xyz);
  else
    std::memcpy(rx_xyz, opt.xyz, sizeof(rx_xyz));

  std::memset(eph, 0, sizeof(eph));
  std::memset(&ionoutc, 0, sizeof(ionoutc));
  n_sets = readRinexNavAll(eph, &ionoutc, opt.navfile.c_str());
  if (n_sets <= 0) {
    std::fprintf(stderr, "ERROR: failed to read RINEX nav file: %s\n",
                 opt.navfile.c_str());
    return 1;
  }

  if (opt.have_time)
    date2gps(&opt.time, &t_now);
  else
    t_now = currentGpsTime(opt.leap_sec);

  if (!sampleRtcm(opt, &rtcm_stream, observed))
    return 1;

  if (!opt.have_time && rtcm3_nav_observed_prns(&rtcm_stream) > 0 &&
      rtcm3_nav_last_obs_tow(&rtcm_stream) > 0.0) {
    t_now.sec = rtcm3_nav_last_obs_tow(&rtcm_stream);
  }

  for (int sv = 0; sv < MAX_SAT; sv++) {
    if (observed[sv] == TRUE)
      have_live_obs = true;
  }

  for (int prn = 1; prn <= MAX_SAT; prn++) {
    Candidate c;
    ephem_t nearest;
    ephem_t picked;
    gpstime_t picked_toe;
    double nearest_dt = 0.0;
    double delta = NAN;
    double elev = NAN;
    int found_ephem = FALSE;

    c.prn = prn;
    c.live_obs = observed[prn - 1] == TRUE;
    c.rtcm_eph = rtcm3_nav_has_prn(&rtcm_stream, prn) == TRUE;
    if (c.rtcm_eph)
      c.rtcm_current_el =
          elevationDeg(rtcm_stream.gps_rtcm_eph[prn - 1], t_now, rx_xyz);

    if (findNearest(eph, n_sets, prn, t_now, &nearest, &nearest_dt) &&
        nearest_dt < SYNTH_REVIVE_IODE_BUMP_PER_SEC) {
      c.rinex_current = true;
      c.nearest_toe = nearest.toe.sec;
      c.rinex_current_el = elevationDeg(nearest, t_now, rx_xyz);
    }

    if (scanEphemerisForRevive(eph, n_sets, prn, t_now, rx_xyz, &picked,
                               &picked_toe, &delta, &elev,
                               &found_ephem) == TRUE) {
      c.scan_ok = true;
      c.scan_delta = delta;
      c.scan_el = elev;
      c.scan_toe = picked_toe;
    }

    for (int i = 0; i < n_sets; i++) {
      int sv = prn - 1;
      double d;
      double e;

      if (eph[i][sv].vflg != 1)
        continue;

      d = subGpsTime(t_now, eph[i][sv].toe);
      if (d < SYNTH_REVIVE_SCAN_STEP_SEC)
        continue;

      e = elevationDeg(eph[i][sv], eph[i][sv].toe, rx_xyz);
      if (std::isnan(e))
        continue;

      if (!c.best_ok || e > c.best_el) {
        c.best_ok = true;
        c.best_delta = d;
        c.best_el = e;
        c.best_toe = eph[i][sv].toe;
      }
    }

    rows.push_back(c);
  }

  std::sort(rows.begin(), rows.end(), [](const Candidate &a,
                                         const Candidate &b) {
    int ba = sortBucket(a);
    int bb = sortBucket(b);
    if (ba != bb)
      return ba < bb;
    if (a.scan_ok && b.scan_ok && std::fabs(a.scan_el - b.scan_el) > 1.0e-9)
      return a.scan_el > b.scan_el;
    if (a.best_ok && b.best_ok && std::fabs(a.best_el - b.best_el) > 1.0e-9)
      return a.best_el > b.best_el;
    return a.prn < b.prn;
  });

  std::printf("RINEX: %s  sets=%d\n", opt.navfile.c_str(), n_sets);
  std::printf("Time: GPS week %d tow %.3f\n", t_now.week, t_now.sec);
  std::printf("Receiver ECEF: %.3f %.3f %.3f\n", rx_xyz[0], rx_xyz[1],
              rx_xyz[2]);
  if (!opt.rtcm_host.empty()) {
    std::printf("RTCM: %s:%d warmup=%.1fs 1019_prns=%d obs_prns=%d\n",
                opt.rtcm_host.c_str(), opt.rtcm_port, opt.rtcm_warmup_sec,
                rtcm3_nav_valid_prns(&rtcm_stream),
                rtcm3_nav_observed_prns(&rtcm_stream));
  }

  listPrns("Live observed 1004 PRNs: ", observed);

  {
    int eph_flags[MAX_SAT] = {0};
    for (int prn = 1; prn <= MAX_SAT; prn++) {
      if (rtcm3_nav_has_prn(&rtcm_stream, prn) == TRUE)
        eph_flags[prn - 1] = TRUE;
    }
    listPrns("Live ephemeris 1019 PRNs: ", eph_flags);
  }

  std::printf("\n");
  std::printf("Recommended rows have status OK. LIVE_OBS means current RTCM "
              "observations already contain that PRN.\n");
  std::printf("rank prn status    obs eph rinex_now rtcm_now scan_h scan_el "
              "scan_toe     best_h best_el best_toe\n");

  int printed = 0;
  for (const Candidate &c : rows) {
    if (!opt.show_all && printed >= opt.top)
      break;

    std::printf("%4d %3d %-9s  %c   %c ",
                printed + 1, c.prn, statusFor(c, have_live_obs),
                c.live_obs ? 'Y' : '-', c.rtcm_eph ? 'Y' : '-');
    printDouble(c.rinex_current_el);
    std::printf(" ");
    printDouble(c.rtcm_current_el);
    std::printf(" ");

    if (c.scan_ok) {
      std::printf("%6.2f %7.1f %4d:%06.0f", c.scan_delta / 3600.0, c.scan_el,
                  c.scan_toe.week, c.scan_toe.sec);
    } else {
      std::printf("   nan     nan        nan");
    }

    if (c.best_ok) {
      std::printf(" %6.2f %7.1f %4d:%06.0f", c.best_delta / 3600.0,
                  c.best_el, c.best_toe.week, c.best_toe.sec);
    } else {
      std::printf("    nan     nan        nan");
    }
    std::printf("\n");

    printed++;
  }

  rtcm3_nav_close(&rtcm_stream);
  return 0;
}
