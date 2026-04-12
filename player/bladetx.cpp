/*
 * bladetx.cpp — Integrated bladeRF GPS signal generator and transmitter.
 *
 * Generates GPS L1 C/A IQ samples in-process and transmits them directly
 * via the libbladeRF API with timed TX metadata.  Eliminates the pipe-based
 * latency of the old gps-sdr-sim | bladeplayer path.
 *
 * Mirrors the x300tx.cpp architecture, substituting libbladeRF for UHD.
 *
 * Build:  make bladetx   (from the project root)
 *
 * Copyright (c) 2024-2026  — same licence as gps-sdr-sim
 */

#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <unistd.h>
#include <vector>

#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>

#include <libbladeRF.h>

extern "C" {
#include "gpssim.h"
}

////////////////////////////////////////////////////////////
// Constants
////////////////////////////////////////////////////////////

#define TX_FREQUENCY 1575420000 // GPS L1 (Hz)
#define TX_SAMPLERATE 2600000   // 2.6 Msps
#define TX_BANDWIDTH 2500000
#define TX_VGA1_DEFAULT (-25)
#define TX_VGA2_DEFAULT 0

#define NUM_BUFFERS 32
#define SAMPLES_PER_BUFFER (32 * 1024)
#define NUM_TRANSFERS 16
#define TIMEOUT_MS 1000

#define PREBUFFER_DEFAULT 5 // 0.5 s pre-buffer
#define EPOCH_TARGET_SEC 0.1

// Trimble time-tag mode defaults
#define TRIMBLE_START_OFFSET_DEFAULT 2
#define TRIMBLE_TAG_LEAD_MS_DEFAULT 500
#define TRIMBLE_TIMEOUT_MS_DEFAULT 30000
#define TRIMBLE_LEAP_SEC_DEFAULT 18
#define TRIMBLE_TX_CAL_NS_DEFAULT 0LL

#define TX_START_LEAD_DEFAULT_NS 250000000LL // 250 ms startup lead
#define TX_START_LEAD_MAX_SEC 60.0
#define TX_START_LEAD_MIN_SEC 0.02

////////////////////////////////////////////////////////////
// Signal handling
////////////////////////////////////////////////////////////

static void sighandler(int) {
  stop_requested = 1; // gpssim.c global
}

static void installSignalHandlers(void) {
  signal(SIGINT, sighandler);
  signal(SIGTERM, sighandler);
  signal(SIGPIPE, SIG_IGN);
}

////////////////////////////////////////////////////////////
// Wall-clock GPS time (for -n mode)
////////////////////////////////////////////////////////////

static double getWallClockRealtimeSeconds(void) {
  struct timespec ts;
  time_t timer;

  if (timespec_get(&ts, TIME_UTC) != TIME_UTC) {
    time(&timer);
    ts.tv_sec = timer;
    ts.tv_nsec = 0;
  }

  return (double)ts.tv_sec + (double)ts.tv_nsec * 1.0e-9;
}

static void epochSecondsToUtcDateTime(double epoch_sec, datetime_t *t0) {
  time_t timer = (time_t)floor(epoch_sec);
  struct tm *gmt = gmtime(&timer);
  double frac_sec = epoch_sec - (double)timer;

  t0->y = gmt->tm_year + 1900;
  t0->m = gmt->tm_mon + 1;
  t0->d = gmt->tm_mday;
  t0->hh = gmt->tm_hour;
  t0->mm = gmt->tm_min;
  t0->sec = (double)gmt->tm_sec + frac_sec;
}

////////////////////////////////////////////////////////////
// Monotonic time helper
////////////////////////////////////////////////////////////

static double getMonotonicSeconds(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (double)ts.tv_sec + (double)ts.tv_nsec * 1.0e-9;
}

////////////////////////////////////////////////////////////
// Trimble 1PPS time-tag helpers
////////////////////////////////////////////////////////////

static int trimbleTcpConnect(const char *host, int port, int timeout_ms) {
  struct addrinfo hints = {}, *res = NULL;
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;

  char port_str[16];
  snprintf(port_str, sizeof(port_str), "%d", port);

  if (getaddrinfo(host, port_str, &hints, &res) != 0 || !res) {
    fprintf(stderr, "[TRIMBLE] ERROR: Cannot resolve host %s\n", host);
    return -1;
  }

  int fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
  if (fd < 0) {
    freeaddrinfo(res);
    fprintf(stderr, "[TRIMBLE] ERROR: socket() failed: %s\n", strerror(errno));
    return -1;
  }

  {
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    int rc = connect(fd, res->ai_addr, res->ai_addrlen);
    if (rc < 0 && errno != EINPROGRESS) {
      freeaddrinfo(res);
      close(fd);
      fprintf(stderr, "[TRIMBLE] ERROR: connect() to %s:%d failed: %s\n", host,
              port, strerror(errno));
      return -1;
    }

    if (rc < 0) {
      struct pollfd pfd = {fd, POLLOUT, 0};
      int pr = poll(&pfd, 1, timeout_ms);
      if (pr <= 0) {
        freeaddrinfo(res);
        close(fd);
        fprintf(stderr, "[TRIMBLE] ERROR: connect() to %s:%d timed out\n", host,
                port);
        return -1;
      }
      int so_err = 0;
      socklen_t so_len = sizeof(so_err);
      getsockopt(fd, SOL_SOCKET, SO_ERROR, &so_err, &so_len);
      if (so_err != 0) {
        freeaddrinfo(res);
        close(fd);
        fprintf(stderr, "[TRIMBLE] ERROR: connect() to %s:%d failed: %s\n",
                host, port, strerror(so_err));
        return -1;
      }
    }

    fcntl(fd, F_SETFL, flags); // restore blocking mode
  }

  freeaddrinfo(res);
  return fd;
}

static const char *findUtcMarker(const char *buf, int len) {
  for (int i = 0; i <= len - 4; i++) {
    if (buf[i] == 'U' && buf[i + 1] == 'T' && buf[i + 2] == 'C' &&
        buf[i + 3] == ' ')
      return &buf[i];
  }
  return NULL;
}

static int trimbleReadTag(int fd, int timeout_ms, int *yy, int *mm, int *dd,
                          int *hh, int *min, int *sec) {
  char buf[4096];
  int pos = 0;

  double deadline = getMonotonicSeconds() + (double)timeout_ms * 1.0e-3;

  while (getMonotonicSeconds() < deadline) {
    int remaining_ms = (int)((deadline - getMonotonicSeconds()) * 1000.0);
    if (remaining_ms <= 0)
      break;

    struct pollfd pfd = {fd, POLLIN, 0};
    int ret = poll(&pfd, 1, remaining_ms);
    if (ret <= 0)
      break;

    ssize_t n = read(fd, buf + pos, sizeof(buf) - (size_t)pos - 1);
    if (n <= 0)
      break;
    pos += (int)n;

    const char *utc = findUtcMarker(buf, pos);
    if (utc) {
      int remaining_bytes = pos - (int)(utc - buf);

      if (remaining_bytes >= 23) {
        char tmp[64];
        int copy_len = remaining_bytes < (int)sizeof(tmp) - 1
                           ? remaining_bytes
                           : (int)sizeof(tmp) - 1;
        memcpy(tmp, utc, copy_len);
        tmp[copy_len] = '\0';

        int nn = sscanf(tmp + 4, "%d.%d.%d %d:%d:%d", yy, mm, dd, hh, min, sec);
        if (nn >= 6 && *yy >= 0 && *yy <= 99 && *mm >= 1 && *mm <= 12 &&
            *dd >= 1 && *dd <= 31 && *hh >= 0 && *hh <= 23 && *min >= 0 &&
            *min <= 59 && *sec >= 0 && *sec <= 59) {
          return 1;
        }
      }
    }

    if (pos > (int)sizeof(buf) - 512) {
      int keep = 64;
      memmove(buf, buf + pos - keep, keep);
      pos = keep;
    }
  }

  return 0;
}

static void trimbleUtcToGpsEpoch(int yy, int mm, int dd, int hh, int min,
                                 int sec, int offset_sec, int leap_sec,
                                 datetime_t *t0, gpstime_t *g0) {
  struct tm tm_utc = {};
  tm_utc.tm_year = (yy >= 80 ? yy : yy + 100);
  tm_utc.tm_mon = mm - 1;
  tm_utc.tm_mday = dd;
  tm_utc.tm_hour = hh;
  tm_utc.tm_min = min;
  tm_utc.tm_sec = sec;

  time_t utc_epoch = timegm(&tm_utc);
  time_t gps_epoch = utc_epoch + offset_sec + leap_sec;

  struct tm *gmt = gmtime(&gps_epoch);
  t0->y = gmt->tm_year + 1900;
  t0->m = gmt->tm_mon + 1;
  t0->d = gmt->tm_mday;
  t0->hh = gmt->tm_hour;
  t0->mm = gmt->tm_min;
  t0->sec = (double)gmt->tm_sec;

  date2gps(t0, g0);
}

static double resolveWallClockGpsTime(datetime_t *t0, gpstime_t *g0,
                                      double lead_sec) {
  double wall_clock_sec = getWallClockRealtimeSeconds();
  double gps_time_sec = wall_clock_sec + 18.0 + lead_sec;

  fprintf(stderr,
          "[TIMING] bladetx wall-clock latch: %lld.%09ld s (CLOCK_REALTIME)\n",
          (long long)floor(wall_clock_sec),
          (long)((wall_clock_sec - floor(wall_clock_sec)) * 1.0e9));

  epochSecondsToUtcDateTime(gps_time_sec, t0);
  date2gps(t0, g0);

  fprintf(stderr,
          "[TIMING] bladetx GPS epoch: week %d  tow %.9f s  "
          "(lead %.3f s included)\n",
          g0->week, g0->sec, lead_sec);

  return wall_clock_sec;
}

////////////////////////////////////////////////////////////
// Epoch plan — fractional sample count tracking
////////////////////////////////////////////////////////////

typedef struct {
  int base_samples;
  double fractional_samples;
  double accumulator;
  int max_samples;
} epoch_plan_t;

static void initEpochPlan(epoch_plan_t *plan, double sample_rate_hz) {
  double exact_samples = sample_rate_hz * EPOCH_TARGET_SEC;

  plan->base_samples = (int)floor(exact_samples);
  plan->fractional_samples = exact_samples - (double)plan->base_samples;
  plan->accumulator = 0.0;
  plan->max_samples =
      plan->base_samples + (plan->fractional_samples > 1.0e-12 ? 1 : 0);
}

static int nextEpochSampleCount(epoch_plan_t *plan) {
  int sample_count = plan->base_samples;

  plan->accumulator += plan->fractional_samples;
  if (plan->accumulator >= 1.0 - 1.0e-12) {
    sample_count++;
    plan->accumulator -= 1.0;
  }

  return sample_count;
}

static double getEpochDurationSec(int sample_count, double sample_rate_hz) {
  return (double)sample_count / sample_rate_hz;
}

////////////////////////////////////////////////////////////
// Generate one 0.1-second epoch of SC16 IQ samples
////////////////////////////////////////////////////////////

static void
generateEpoch(short *iq_buff, int sample_count, channel_t chan[MAX_CHAN],
              int gain[MAX_CHAN], ephem_t *active_eph, ionoutc_t *ionoutc,
              gpstime_t grx, int staticLocationMode, double epoch_duration,
              double delt, int path_loss_enable, int fixed_gain,
              double ant_pat[37], int attack_enabled,
              const attack_config_t *attack_cfg,
              unsigned int attack_noise_state[MAX_SAT], double jam_js_linear) {
  int i, sv, isamp;
  int ip, qp, iTable;
  int ibs;
  double path_loss, ant_gain;

  (void)sv;

  for (i = 0; i < MAX_CHAN; i++) {
    if (chan[i].prn > 0) {
      range_t rho;
      sv = chan[i].prn - 1;

      computeRange(&rho, active_eph[sv], ionoutc, grx, xyz[0]);

      chan[i].azel[0] = rho.azel[0];
      chan[i].azel[1] = rho.azel[1];

      computeCodePhase(&chan[i], rho, epoch_duration);
#ifndef FLOAT_CARR_PHASE
      chan[i].carr_phasestep =
          (int)round(512.0 * 65536.0 * chan[i].f_carr * delt);
#endif
      path_loss = 20200000.0 / rho.d;

      ibs = (int)((90.0 - rho.azel[1] * R2D) / 5.0);
      ant_gain = ant_pat[ibs];

      if (path_loss_enable == TRUE)
        gain[i] = (int)(path_loss * ant_gain * 128.0);
      else
        gain[i] = fixed_gain;

      if (attack_enabled == TRUE)
        applyGainAttack(attack_cfg, chan[i].prn, &gain[i]);

      if (attack_cfg->partial_mode && attack_cfg->gain_boost_db != 0.0)
        gain[i] = (int)(gain[i] * pow(10.0, attack_cfg->gain_boost_db / 20.0));
    }
  }

  for (isamp = 0; isamp < sample_count; isamp++) {
    int i_acc = 0;
    int q_acc = 0;

    for (i = 0; i < MAX_CHAN; i++) {
      if (chan[i].prn > 0) {
        attack_method_t attack_method = ATTACK_METHOD_NONE;
        if (attack_enabled == TRUE)
          attack_method = getAttackMethod(attack_cfg, chan[i].prn);

#ifdef FLOAT_CARR_PHASE
        iTable = (int)floor(chan[i].carr_phase * 512.0);
#else
        iTable = (chan[i].carr_phase >> 16) & 0x1ff;
#endif
        if (attack_method == ATTACK_METHOD_JAM_NOISE) {
          unsigned int *state = &attack_noise_state[chan[i].prn - 1];
          double noise_amp = (double)gain[i] * jam_js_linear;
          double ni = (double)nextGaussianNoise(state) / JAM_NOISE_RMS_SCALE;
          double nq = (double)nextGaussianNoise(state) / JAM_NOISE_RMS_SCALE;

          ip = clipInt32FromDouble(ni * chan[i].codeCA * cosTable512[iTable] *
                                   noise_amp);
          qp = clipInt32FromDouble(nq * chan[i].codeCA * sinTable512[iTable] *
                                   noise_amp);
        } else {
          ip = chan[i].dataBit * chan[i].codeCA * cosTable512[iTable] * gain[i];
          qp = chan[i].dataBit * chan[i].codeCA * sinTable512[iTable] * gain[i];
        }

        i_acc += ip;
        q_acc += qp;

        chan[i].code_phase += chan[i].f_code * delt;

        if (chan[i].code_phase >= CA_SEQ_LEN) {
          chan[i].code_phase -= CA_SEQ_LEN;
          chan[i].icode++;

          if (chan[i].icode >= 20) {
            chan[i].icode = 0;
            chan[i].ibit++;

            if (chan[i].ibit >= 30) {
              chan[i].ibit = 0;
              chan[i].iword++;
            }

            chan[i].dataBit =
                (int)((chan[i].dwrd[chan[i].iword] >> (29 - chan[i].ibit)) &
                      0x1UL) *
                    2 -
                1;
          }
        }

        chan[i].codeCA = chan[i].ca[(int)chan[i].code_phase] * 2 - 1;

#ifdef FLOAT_CARR_PHASE
        chan[i].carr_phase += chan[i].f_carr * delt;
        if (chan[i].carr_phase >= 1.0)
          chan[i].carr_phase -= 1.0;
        else if (chan[i].carr_phase < 0.0)
          chan[i].carr_phase += 1.0;
#else
        chan[i].carr_phase += chan[i].carr_phasestep;
#endif
      }
    }

    i_acc = (i_acc + 64) >> 7;
    q_acc = (q_acc + 64) >> 7;

    iq_buff[isamp * 2] = clipInt16(i_acc);
    iq_buff[isamp * 2 + 1] = clipInt16(q_acc);
  }
}

static void refreshNavState(channel_t chan[MAX_CHAN], ephem_t eph[][MAX_SAT],
                            int neph, int *ieph, ephem_t *active_eph,
                            synth_ephem_store_t *synth_eph,
                            const synth_config_t *synth_cfg,
                            const ionoutc_t *ionoutc, gpstime_t grx,
                            double elvmask,
                            const attack_config_t *attack_cfg) {
  int i;
  int eph_changed = FALSE;

  if (*ieph + 1 < neph) {
    gpstime_t next_toc;

    if (getSetReferenceToc(eph[*ieph + 1], &next_toc) == TRUE &&
        shouldAdvanceEphSet(next_toc, grx) == TRUE) {
      (*ieph)++;
      eph_changed = TRUE;
    }
  }

  if (synth_cfg->enabled) {
    gpstime_t synth_ref = quantizeSynthReferenceTime(grx);

    if (refreshSyntheticEphemerisSet(synth_eph, eph[*ieph], ionoutc, synth_cfg,
                                     xyz[0], synth_ref) == TRUE)
      eph_changed = TRUE;
  }

  if (eph_changed == TRUE) {
    overlaySyntheticEphemerisSet(active_eph, eph[*ieph], synth_cfg, synth_eph);

    for (i = 0; i < MAX_CHAN; i++) {
      if (chan[i].prn != 0)
        eph2sbf(active_eph[chan[i].prn - 1], *ionoutc, chan[i].sbf);
    }
  }

  for (i = 0; i < MAX_CHAN; i++) {
    if (chan[i].prn > 0)
      generateNavMsg(grx, &chan[i], 0);
  }

  allocateChannel(chan, active_eph, *ionoutc, grx, xyz[0], elvmask, attack_cfg,
                  synth_cfg);
}

////////////////////////////////////////////////////////////
// Usage
////////////////////////////////////////////////////////////

static void bladetx_usage(void) {
  fprintf(
      stderr,
      "Usage: bladetx [options]\n"
      "\n"
      "GPS simulation options:\n"
      "  -e <rinex_nav>              Navigation RINEX file (required)\n"
      "  -l <lat,lon,alt>            Static location (deg,deg,m)\n"
      "  -c <x,y,z>                  Static ECEF position (m)\n"
      "  -t <YYYY/MM/DD,hh:mm:ss>   Start time (UTC)\n"
      "  -n                          Stream-now mode (wall clock)\n"
      "  -d <seconds>                Duration\n"
      "  -P <prn[,prn...]>           Partial constellation PRN list\n"
      "  -S <synth_spec>             Synthetic satellite config\n"
      "  -A <attack_spec>            Attack config\n"
      "  -J <dB>                     Jammer-to-signal ratio\n"
      "  -G <dB>                     Power boost for partial mode\n"
      "  -r <lead_sec>               Future TX start lead for -n mode\n"
      "  -p [fixed_gain]             Disable path loss\n"
      "  -i                          Disable ionospheric correction\n"
      "  -v                          Verbose\n"
      "\n"
      "bladeRF options:\n"
      "  --gps-week <N>              GPS week number\n"
      "  --gps-tow <sec>             GPS time of week (seconds)\n"
      "  --tx-advance-ns <ns>        Future TX start lead (default 250000000)\n"
      "  --txvga1 <dB>               TX VGA1 gain [-35..-4] (default %d)\n"
      "  --txvga2 <dB>               TX VGA2 gain [0..25] (default %d)\n"
      "  --device <devstr>           bladeRF device string (default: auto)\n"
      "  --prebuffer <N>             Pre-buffer epochs (default 5)\n"
      "\n"
      "Trimble 1PPS time-tag options (mutually exclusive with -n and "
      "--gps-week/tow):\n"
      "  --trimble-time-tag-host <host>  Trimble TCP host (enables Trimble "
      "mode)\n"
      "  --trimble-time-tag-port <port>  Trimble TCP port (enables Trimble "
      "mode)\n"
      "  --trimble-start-offset-sec <s>  Future offset from tag (default 2)\n"
      "  --trimble-tag-lead-ms <ms>      Tag-to-PPS lead estimate (default "
      "500)\n"
      "  --trimble-timeout-ms <ms>       TCP read timeout (default 30000)\n"
      "  --trimble-leap-sec <sec>        UTC-to-GPS leap offset (default 18)\n"
      "  --trimble-tx-cal-ns <ns>        Calibration term in ns (default 0)\n"
      "\n"
      "Examples:\n"
      "  bladetx -e hour0910.26n -l 21.0047844,105.8460541,5 \\\n"
      "    -P 3,4,7,8 -S 3:0/60,4:90/45,7:180/30,8:45/55 \\\n"
      "    --gps-week 2361 --gps-tow 118800.0 --txvga1 -35\n"
      "\n"
      "  bladetx -e hour0910.26n -l 21.0047844,105.8460541,5 \\\n"
      "    -P 3,4,7,8 -S 3:0/60,4:90/45,7:180/30,8:45/55 \\\n"
      "    --trimble-time-tag-host 192.168.5.245 \\\n"
      "    --trimble-time-tag-port 5017 \\\n"
      "    --trimble-start-offset-sec 2 --txvga1 -35\n"
      "\n",
      TX_VGA1_DEFAULT, TX_VGA2_DEFAULT);
}

////////////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////////////

int main(int argc, char *argv[]) {
  int sv, i;
  int neph, ieph;
  ephem_t eph[EPHEM_ARRAY_SIZE][MAX_SAT];
  ephem_t active_eph[MAX_SAT];
  synth_ephem_store_t synth_eph;
  gpstime_t g0;

  double llh[3];

  channel_t chan[MAX_CHAN];
  double elvmask = 0.0;

  int gain[MAX_CHAN];
  short *iq_buff = NULL;

  gpstime_t grx;
  double delt;

  int iumd, numd;

  int staticLocationMode = TRUE;
  int location_specified = FALSE;

  char navfile[MAX_CHAR];

  double samp_freq;
  int iq_buff_size;

  int result;

  double path_loss;
  double ant_gain;
  int fixed_gain = 128;
  double ant_pat[37];

  datetime_t t0, tmin, tmax;
  gpstime_t gmin, gmax;
  int igrx;

  double duration;
  int iduration;
  int verb = FALSE;
  int duration_specified = FALSE;
  int current_time_mode = FALSE;
  int stream_mode = FALSE;
  int stream_forever = FALSE;
  double wall_clock_latch_sec = -1.0;
  double current_epoch_duration = 0.0;

  int timeoverwrite = FALSE;
  int attack_enabled = FALSE;
  unsigned int attack_noise_state[MAX_SAT];
  double jam_js_linear = 10.0;

  ionoutc_t ionoutc;
  attack_config_t attack_cfg;
  synth_config_t synth_cfg;
  int path_loss_enable = TRUE;

  // GPS epoch options
  int gps_week_set = FALSE;
  int gps_tow_set = FALSE;
  int explicit_gps_week = -1;
  double explicit_gps_tow = 0.0;
  long long tx_advance_ns = TX_START_LEAD_DEFAULT_NS;

  // bladeRF-specific options
  int txvga1 = TX_VGA1_DEFAULT;
  int txvga2 = TX_VGA2_DEFAULT;
  char blade_devstr[256] = "";
  int prebuffer_count = PREBUFFER_DEFAULT;

  // Trimble time-tag mode
  char trimble_host[256] = "";
  int trimble_port = 0;
  int trimble_mode = FALSE;
  int trimble_start_offset = TRIMBLE_START_OFFSET_DEFAULT;
  int trimble_tag_lead_ms = TRIMBLE_TAG_LEAD_MS_DEFAULT;
  int trimble_timeout_ms = TRIMBLE_TIMEOUT_MS_DEFAULT;
  int trimble_leap_sec = TRIMBLE_LEAP_SEC_DEFAULT;
  long long trimble_tx_cal_ns = TRIMBLE_TX_CAL_NS_DEFAULT;
  double trimble_tag_mono = -1.0;

  epoch_plan_t epoch_plan;

  (void)path_loss;
  (void)ant_gain;

  ////////////////////////////////////////////////////////////
  // Parse options
  ////////////////////////////////////////////////////////////

  navfile[0] = 0;
  samp_freq = TX_SAMPLERATE;
  g0.week = -1;
  iduration = USER_MOTION_SIZE;
  duration = (double)iduration / 10.0;
  ionoutc.enable = TRUE;
  ionoutc.leapen = FALSE;
  initAttackConfig(&attack_cfg);
  initAttackNoiseState(attack_noise_state);
  initSynthConfig(&synth_cfg);
  initSynthEphemStore(&synth_eph);

  if (argc == 2 &&
      (strcmp(argv[1], "--help") == 0 || strcmp(argv[1], "-h") == 0)) {
    bladetx_usage();
    return 0;
  }

  if (argc < 3) {
    bladetx_usage();
    return 1;
  }

  // Separate long options (--xxx) from short options for getopt
  std::vector<char *> short_argv;
  short_argv.push_back(argv[0]);

  for (i = 1; i < argc; i++) {
    if (strncmp(argv[i], "--", 2) != 0) {
      short_argv.push_back(argv[i]);
      continue;
    }

    const char *opt = argv[i] + 2;

    if (strcmp(opt, "help") == 0) {
      bladetx_usage();
      return 0;
    }

    if (i + 1 >= argc) {
      fprintf(stderr, "ERROR: Missing value for option --%s.\n", opt);
      return 1;
    }

    const char *val = argv[++i];

    if (strcmp(opt, "gps-week") == 0) {
      explicit_gps_week = atoi(val);
      gps_week_set = TRUE;
      continue;
    }
    if (strcmp(opt, "gps-tow") == 0) {
      explicit_gps_tow = atof(val);
      gps_tow_set = TRUE;
      continue;
    }
    if (strcmp(opt, "tx-advance-ns") == 0) {
      tx_advance_ns = atoll(val);
      if (tx_advance_ns < 0 ||
          tx_advance_ns > (long long)(TX_START_LEAD_MAX_SEC * 1.0e9)) {
        fprintf(stderr, "ERROR: --tx-advance-ns must be 0-%.0f seconds.\n",
                TX_START_LEAD_MAX_SEC);
        return 1;
      }
      continue;
    }
    if (strcmp(opt, "txvga1") == 0) {
      txvga1 = atoi(val);
      if (txvga1 < -35 || txvga1 > -4) {
        fprintf(stderr, "ERROR: --txvga1 must be -35..-4 dB.\n");
        return 1;
      }
      continue;
    }
    if (strcmp(opt, "txvga2") == 0) {
      txvga2 = atoi(val);
      if (txvga2 < 0 || txvga2 > 25) {
        fprintf(stderr, "ERROR: --txvga2 must be 0..25 dB.\n");
        return 1;
      }
      continue;
    }
    if (strcmp(opt, "device") == 0) {
      strncpy(blade_devstr, val, sizeof(blade_devstr) - 1);
      blade_devstr[sizeof(blade_devstr) - 1] = '\0';
      continue;
    }
    if (strcmp(opt, "prebuffer") == 0) {
      prebuffer_count = atoi(val);
      if (prebuffer_count < 1 || prebuffer_count > 50) {
        fprintf(stderr, "ERROR: prebuffer must be 1-50.\n");
        return 1;
      }
      continue;
    }

    // Trimble time-tag options
    if (strcmp(opt, "trimble-time-tag-host") == 0) {
      strncpy(trimble_host, val, sizeof(trimble_host) - 1);
      trimble_host[sizeof(trimble_host) - 1] = '\0';
      continue;
    }
    if (strcmp(opt, "trimble-time-tag-port") == 0) {
      trimble_port = atoi(val);
      if (trimble_port <= 0 || trimble_port > 65535) {
        fprintf(stderr, "ERROR: --trimble-time-tag-port must be 1-65535.\n");
        return 1;
      }
      continue;
    }
    if (strcmp(opt, "trimble-start-offset-sec") == 0) {
      trimble_start_offset = atoi(val);
      if (trimble_start_offset < 1 || trimble_start_offset > 30) {
        fprintf(stderr, "ERROR: --trimble-start-offset-sec must be 1-30.\n");
        return 1;
      }
      continue;
    }
    if (strcmp(opt, "trimble-tag-lead-ms") == 0) {
      trimble_tag_lead_ms = atoi(val);
      if (trimble_tag_lead_ms < 0 || trimble_tag_lead_ms > 5000) {
        fprintf(stderr, "ERROR: --trimble-tag-lead-ms must be 0-5000.\n");
        return 1;
      }
      continue;
    }
    if (strcmp(opt, "trimble-timeout-ms") == 0) {
      trimble_timeout_ms = atoi(val);
      if (trimble_timeout_ms < 100 || trimble_timeout_ms > 30000) {
        fprintf(stderr, "ERROR: --trimble-timeout-ms must be 100-30000.\n");
        return 1;
      }
      continue;
    }
    if (strcmp(opt, "trimble-leap-sec") == 0) {
      trimble_leap_sec = atoi(val);
      if (trimble_leap_sec < 0 || trimble_leap_sec > 50) {
        fprintf(stderr, "ERROR: --trimble-leap-sec must be 0-50.\n");
        return 1;
      }
      continue;
    }
    if (strcmp(opt, "trimble-tx-cal-ns") == 0) {
      trimble_tx_cal_ns = atoll(val);
      continue;
    }

    fprintf(stderr, "ERROR: Unknown option --%s.\n", opt);
    return 1;
  }

  short_argv.push_back(nullptr);
  optind = 1;

  while ((result = getopt((int)short_argv.size() - 1, short_argv.data(),
                          "e:l:c:t:d:P:S:A:J:G:r:ipvnh")) != -1) {
    switch (result) {
    case 'e':
      strncpy(navfile, optarg, MAX_CHAR - 1);
      navfile[MAX_CHAR - 1] = '\0';
      break;
    case 'l':
      location_specified = TRUE;
      staticLocationMode = TRUE;
      sscanf(optarg, "%lf,%lf,%lf", &llh[0], &llh[1], &llh[2]);
      llh[0] = llh[0] / R2D;
      llh[1] = llh[1] / R2D;
      llh2xyz(llh, xyz[0]);
      break;
    case 'c':
      location_specified = TRUE;
      staticLocationMode = TRUE;
      sscanf(optarg, "%lf,%lf,%lf", &xyz[0][0], &xyz[0][1], &xyz[0][2]);
      break;
    case 't':
      current_time_mode = FALSE;
      sscanf(optarg, "%d/%d/%d,%d:%d:%lf", &t0.y, &t0.m, &t0.d, &t0.hh, &t0.mm,
             &t0.sec);
      if (t0.y <= 1980 || t0.m < 1 || t0.m > 12 || t0.d < 1 || t0.d > 31 ||
          t0.hh < 0 || t0.hh > 23 || t0.mm < 0 || t0.mm > 59 || t0.sec < 0.0 ||
          t0.sec >= 60.0) {
        fprintf(stderr, "ERROR: Invalid date and time.\n");
        return 1;
      }
      t0.sec = floor(t0.sec);
      date2gps(&t0, &g0);
      timeoverwrite = TRUE;
      break;
    case 'd':
      duration_specified = TRUE;
      duration = atof(optarg);
      break;
    case 'P':
      if (parsePartialPrns(&attack_cfg, optarg) == FALSE) {
        fprintf(stderr, "ERROR: Invalid PRN list.\n");
        return 1;
      }
      break;
    case 'S':
      if (parseSynthConfig(&synth_cfg, optarg) == FALSE) {
        fprintf(stderr, "ERROR: Invalid synthetic spec.\n");
        return 1;
      }
      break;
    case 'A':
      if (parseAttackConfig(&attack_cfg, optarg) == FALSE) {
        fprintf(stderr, "ERROR: Invalid attack spec.\n");
        return 1;
      }
      attack_enabled = TRUE;
      break;
    case 'J':
      attack_cfg.jam_js_db = atof(optarg);
      jam_js_linear = pow(10.0, attack_cfg.jam_js_db / 20.0);
      break;
    case 'G':
      attack_cfg.gain_boost_db = atof(optarg);
      break;
    case 'r': {
      double lead_sec = atof(optarg);
      if (lead_sec < 0.0 || lead_sec > TX_START_LEAD_MAX_SEC) {
        fprintf(stderr, "ERROR: TX start lead must be 0-%.0f seconds.\n",
                TX_START_LEAD_MAX_SEC);
        return 1;
      }
      tx_advance_ns = (long long)llround(lead_sec * 1.0e9);
      break;
    }
    case 'p':
      if (optind < argc && argv[optind][0] != '-') {
        fixed_gain = atoi(argv[optind]);
        if (fixed_gain < 1 || fixed_gain > 128) {
          fprintf(stderr, "ERROR: Fixed gain must be between 1 and 128.\n");
          return 1;
        }
        optind++;
      }
      path_loss_enable = FALSE;
      break;
    case 'i':
      ionoutc.enable = FALSE;
      break;
    case 'v':
      verb = TRUE;
      break;
    case 'n':
      stream_mode = TRUE;
      timeoverwrite = TRUE;
      current_time_mode = TRUE;
      break;
    case 'h':
      bladetx_usage();
      return 0;
    case ':':
    case '?':
      bladetx_usage();
      return 1;
    default:
      break;
    }
  }

  if (navfile[0] == 0) {
    fprintf(stderr, "ERROR: Navigation RINEX file is required (-e).\n");
    return 1;
  }

  // Explicit GPS week/TOW overrides -t and -n
  if (gps_week_set && gps_tow_set) {
    g0.week = explicit_gps_week;
    g0.sec = explicit_gps_tow;
    gps2date(&g0, &t0);
    timeoverwrite = TRUE;
    current_time_mode = FALSE;
    fprintf(stderr, "[TIMING] Using explicit GPS epoch: week %d  tow %.3f\n",
            g0.week, g0.sec);
  } else if (gps_week_set || gps_tow_set) {
    fprintf(stderr,
            "ERROR: --gps-week and --gps-tow must both be specified.\n");
    return 1;
  }

  // Activate Trimble mode if both host and port are specified
  if (trimble_host[0] != '\0' && trimble_port > 0) {
    trimble_mode = TRUE;
    timeoverwrite = TRUE;
    stream_mode = TRUE;
  } else if (trimble_host[0] != '\0' || trimble_port > 0) {
    fprintf(stderr,
            "ERROR: --trimble-time-tag-host and --trimble-time-tag-port "
            "must both be specified.\n");
    return 1;
  }

  // Mutual exclusion: Trimble vs -n vs explicit GPS epoch
  if (trimble_mode) {
    if (current_time_mode) {
      fprintf(stderr,
              "ERROR: Trimble time-tag mode cannot be combined with -n.\n");
      return 1;
    }
    if (gps_week_set || gps_tow_set) {
      fprintf(stderr, "ERROR: Trimble time-tag mode cannot be combined with "
                      "--gps-week/--gps-tow.\n");
      return 1;
    }
  }

  // Default location if none specified
  if (!location_specified) {
    llh[0] = 35.681298 / R2D;
    llh[1] = 139.766247 / R2D;
    llh[2] = 10.0;
    llh2xyz(llh, xyz[0]);
  }

  stream_forever = (stream_mode == TRUE && staticLocationMode == TRUE &&
                    duration_specified == FALSE);

  if (!stream_forever && duration < 0.0) {
    fprintf(stderr, "ERROR: Invalid duration.\n");
    return 1;
  }
  if (!stream_forever && duration > STATIC_MAX_DURATION && staticLocationMode) {
    fprintf(stderr, "ERROR: Invalid duration.\n");
    return 1;
  }
  iduration = (int)(duration * 10.0 + 0.5);

  numd = iduration;

  xyz2llh(xyz[0], llh);
  fprintf(stderr, "xyz = %11.1f, %11.1f, %11.1f\n", xyz[0][0], xyz[0][1],
          xyz[0][2]);
  fprintf(stderr, "llh = %11.6f, %11.6f, %11.1f\n", llh[0] * R2D, llh[1] * R2D,
          llh[2]);

  ////////////////////////////////////////////////////////////
  // Read ephemeris
  ////////////////////////////////////////////////////////////

  neph = readRinexNavAll(eph, &ionoutc, navfile);
  if (neph == 0) {
    fprintf(stderr, "ERROR: No ephemeris available.\n");
    return 1;
  } else if (neph == -1) {
    fprintf(stderr, "ERROR: Ephemeris file not found.\n");
    return 1;
  }

  for (sv = 0; sv < MAX_SAT; sv++) {
    if (eph[0][sv].vflg == 1) {
      gmin = eph[0][sv].toc;
      tmin = eph[0][sv].t;
      break;
    }
  }

  gmax.sec = 0;
  gmax.week = 0;
  tmax.sec = 0;
  tmax.mm = 0;
  tmax.hh = 0;
  tmax.d = 0;
  tmax.m = 0;
  tmax.y = 0;
  for (sv = 0; sv < MAX_SAT; sv++) {
    if (eph[neph - 1][sv].vflg == 1) {
      gmax = eph[neph - 1][sv].toc;
      tmax = eph[neph - 1][sv].t;
      break;
    }
  }

  ////////////////////////////////////////////////////////////
  // Configure bladeRF
  ////////////////////////////////////////////////////////////

  struct bladerf *dev = NULL;
  int status;

  fprintf(stderr, "\n[BLADE] Opening bladeRF device ...\n");

  status = bladerf_open(&dev, blade_devstr[0] ? blade_devstr : NULL);
  if (status != 0) {
    fprintf(stderr, "ERROR: Failed to open bladeRF: %s\n",
            bladerf_strerror(status));
    return 1;
  }

  {
    struct bladerf_devinfo info;
    if (bladerf_get_devinfo(dev, &info) == 0) {
      fprintf(stderr, "[BLADE] Device: %s  serial: %s\n",
              bladerf_backend_str(info.backend), info.serial);
    }
  }

  status = bladerf_set_frequency(dev, BLADERF_MODULE_TX, TX_FREQUENCY);
  if (status != 0) {
    fprintf(stderr, "ERROR: Failed to set TX frequency: %s\n",
            bladerf_strerror(status));
    goto cleanup_dev;
  }
  fprintf(stderr, "[BLADE] TX frequency: %u Hz\n", TX_FREQUENCY);

  status = bladerf_set_sample_rate(dev, BLADERF_MODULE_TX, TX_SAMPLERATE, NULL);
  if (status != 0) {
    fprintf(stderr, "ERROR: Failed to set TX sample rate: %s\n",
            bladerf_strerror(status));
    goto cleanup_dev;
  }
  fprintf(stderr, "[BLADE] TX sample rate: %u sps\n", TX_SAMPLERATE);

  status = bladerf_set_bandwidth(dev, BLADERF_MODULE_TX, TX_BANDWIDTH, NULL);
  if (status != 0) {
    fprintf(stderr, "ERROR: Failed to set TX bandwidth: %s\n",
            bladerf_strerror(status));
    goto cleanup_dev;
  }
  fprintf(stderr, "[BLADE] TX bandwidth: %u Hz\n", TX_BANDWIDTH);

  status = bladerf_set_txvga1(dev, txvga1);
  if (status != 0) {
    fprintf(stderr, "ERROR: Failed to set TX VGA1 gain: %s\n",
            bladerf_strerror(status));
    goto cleanup_dev;
  }
  fprintf(stderr, "[BLADE] TX VGA1: %d dB\n", txvga1);

  status = bladerf_set_txvga2(dev, txvga2);
  if (status != 0) {
    fprintf(stderr, "ERROR: Failed to set TX VGA2 gain: %s\n",
            bladerf_strerror(status));
    goto cleanup_dev;
  }
  fprintf(stderr, "[BLADE] TX VGA2: %d dB\n", txvga2);

  // Configure sync interface with metadata for timed TX
  status = bladerf_sync_config(dev, BLADERF_TX_X1, BLADERF_FORMAT_SC16_Q11_META,
                               NUM_BUFFERS, SAMPLES_PER_BUFFER, NUM_TRANSFERS,
                               TIMEOUT_MS);
  if (status != 0) {
    fprintf(stderr, "ERROR: Failed to configure TX sync interface: %s\n",
            bladerf_strerror(status));
    goto cleanup_dev;
  }

  status = bladerf_enable_module(dev, BLADERF_MODULE_TX, true);
  if (status != 0) {
    fprintf(stderr, "ERROR: Failed to enable TX module: %s\n",
            bladerf_strerror(status));
    goto cleanup_dev;
  }

  // Lock sample rate and epoch plan to device actual rate
  samp_freq = (double)TX_SAMPLERATE;
  delt = 1.0 / samp_freq;
  initEpochPlan(&epoch_plan, samp_freq);
  iq_buff_size = epoch_plan.max_samples;

  fprintf(stderr, "[TIMING] Generator sample rate: %.6f Hz\n", samp_freq);
  if (epoch_plan.max_samples == epoch_plan.base_samples) {
    fprintf(stderr, "[TIMING] Epoch sample count: %d samples every %.1f ms\n",
            epoch_plan.base_samples, EPOCH_TARGET_SEC * 1000.0);
  } else {
    fprintf(stderr,
            "[TIMING] Epoch sample count dithers between %d and %d samples "
            "to match the sample rate.\n",
            epoch_plan.base_samples, epoch_plan.max_samples);
  }

  ////////////////////////////////////////////////////////////
  // Resolve GPS start time
  ////////////////////////////////////////////////////////////

  if (trimble_mode) {
    ////////////////////////////////////////////////////////////////
    // Trimble time-tag epoch resolution
    ////////////////////////////////////////////////////////////////

    fprintf(stderr, "\n[TRIMBLE] Connecting to %s:%d ...\n", trimble_host,
            trimble_port);

    int tfd = trimbleTcpConnect(trimble_host, trimble_port, trimble_timeout_ms);
    if (tfd < 0) {
      status = 1;
      goto cleanup_module;
    }
    fprintf(stderr, "[TRIMBLE] Connected.\n");

    int tag_yy, tag_mm, tag_dd, tag_hh, tag_min, tag_sec;

    int got_tag = trimbleReadTag(tfd, trimble_timeout_ms, &tag_yy, &tag_mm,
                                 &tag_dd, &tag_hh, &tag_min, &tag_sec);
    if (got_tag)
      trimble_tag_mono = getMonotonicSeconds();

    close(tfd);

    if (!got_tag) {
      fprintf(stderr,
              "[TRIMBLE] ERROR: No valid time tag received within timeout.\n");
      status = 1;
      goto cleanup_module;
    }

    int full_year = (tag_yy >= 80) ? 1900 + tag_yy : 2000 + tag_yy;
    fprintf(stderr,
            "[TRIMBLE] Tag received: UTC %04d-%02d-%02d %02d:%02d:%02d\n",
            full_year, tag_mm, tag_dd, tag_hh, tag_min, tag_sec);
    fprintf(stderr,
            "[TRIMBLE] Config: start-offset=%d s  leap=%d s  "
            "tag-lead=%d ms  cal=%lld ns\n",
            trimble_start_offset, trimble_leap_sec, trimble_tag_lead_ms,
            trimble_tx_cal_ns);

    trimbleUtcToGpsEpoch(tag_yy, tag_mm, tag_dd, tag_hh, tag_min, tag_sec,
                         trimble_start_offset, trimble_leap_sec, &t0, &g0);

    // Apply calibration term
    if (trimble_tx_cal_ns != 0) {
      double cal_sec = (double)trimble_tx_cal_ns * 1.0e-9;
      g0 = incGpsTime(g0, cal_sec);
      gps2date(&g0, &t0);
      fprintf(stderr, "[TRIMBLE] Applied calibration: %+.3f us\n",
              cal_sec * 1.0e6);
    }

    fprintf(stderr, "[TRIMBLE] Target GPS epoch: week %d  tow %.9f\n", g0.week,
            g0.sec);
    fprintf(stderr,
            "[TRIMBLE] Target datetime:  %04d/%02d/%02d,%02d:%02d:%06.3f\n",
            t0.y, t0.m, t0.d, t0.hh, t0.mm, t0.sec);

  } else if (current_time_mode == TRUE && !gps_week_set) {
    double lead_sec = (double)tx_advance_ns * 1.0e-9;

    wall_clock_latch_sec = resolveWallClockGpsTime(&t0, &g0, lead_sec);
    fprintf(stderr,
            "[TIMING] GPS time (wall clock + %.3f s TX lead): "
            "%4d/%02d/%02d,%02d:%02d:%06.3f\n",
            lead_sec, t0.y, t0.m, t0.d, t0.hh, t0.mm, t0.sec);
  }

  if (g0.week >= 0) {
    if (timeoverwrite == TRUE) {
      gpstime_t gtmp;
      datetime_t ttmp;
      double dsec;

      gtmp.week = g0.week;
      gtmp.sec = (double)(((int)(g0.sec)) / 7200) * 7200.0;
      dsec = subGpsTime(gtmp, gmin);

      ionoutc.wnt = gtmp.week;
      ionoutc.tot = (int)gtmp.sec;

      for (sv = 0; sv < MAX_SAT; sv++) {
        for (i = 0; i < neph; i++) {
          if (eph[i][sv].vflg == 1) {
            gtmp = incGpsTime(eph[i][sv].toc, dsec);
            gps2date(&gtmp, &ttmp);
            eph[i][sv].toc = gtmp;
            eph[i][sv].t = ttmp;

            gtmp = incGpsTime(eph[i][sv].toe, dsec);
            eph[i][sv].toe = gtmp;
          }
        }
      }
    } else {
      if (subGpsTime(g0, gmin) < 0.0 || subGpsTime(gmax, g0) < 0.0) {
        fprintf(stderr, "ERROR: Invalid start time.\n");
        fprintf(stderr, "tmin = %4d/%02d/%02d,%02d:%02d:%02.0f (%d:%.0f)\n",
                tmin.y, tmin.m, tmin.d, tmin.hh, tmin.mm, tmin.sec, gmin.week,
                gmin.sec);
        fprintf(stderr, "tmax = %4d/%02d/%02d,%02d:%02d:%02.0f (%d:%.0f)\n",
                tmax.y, tmax.m, tmax.d, tmax.hh, tmax.mm, tmax.sec, gmax.week,
                gmax.sec);
        status = 1;
        goto cleanup_module;
      }
    }
  } else {
    g0 = gmin;
    t0 = tmin;
  }

  fprintf(stderr, "Start time = %4d/%02d/%02d,%02d:%02d:%06.3f (%d:%.3f)\n",
          t0.y, t0.m, t0.d, t0.hh, t0.mm, t0.sec, g0.week, g0.sec);
  if (stream_forever)
    fprintf(stderr, "Duration = streaming until interrupted\n");
  else
    fprintf(stderr, "Duration = %.1f [sec]\n", (double)numd / 10.0);

  ////////////////////////////////////////////////////////////
  // Select ephemeris set
  ////////////////////////////////////////////////////////////

  ieph = -1;
  for (i = 0; i < neph; i++) {
    gpstime_t ref_toc;
    if (getSetReferenceToc(eph[i], &ref_toc) == TRUE &&
        shouldAdvanceEphSet(ref_toc, g0) == TRUE) {
      ieph = i;
      break;
    }
  }
  if (ieph == -1) {
    fprintf(stderr, "ERROR: No current ephemeris set found.\n");
    status = 1;
    goto cleanup_module;
  }

  ////////////////////////////////////////////////////////////
  // Build synthetic satellite overlay
  ////////////////////////////////////////////////////////////

  if (synth_cfg.enabled) {
    gpstime_t synth_ref = quantizeSynthReferenceTime(g0);

    for (sv = 0; sv < MAX_SAT; sv++) {
      if (synth_cfg.mode[sv] == SYNTH_OVERHEAD ||
          synth_cfg.mode[sv] == SYNTH_AZEL) {
        double az, el;

        if (synth_cfg.mode[sv] == SYNTH_OVERHEAD) {
          az = 0.0;
          el = PI / 2.0;
        } else {
          az = synth_cfg.azimuth[sv];
          el = synth_cfg.elevation[sv];
        }

        {
          double test_sat[3];

          if (synthAzelReachable(xyz[0], az, el, test_sat) == FALSE) {
            fprintf(stderr,
                    "WARNING: PRN %d az=%.1f el=%.1f deg lies outside the "
                    "synthetic GPS inclination envelope at this location. "
                    "Skipping.\n",
                    sv + 1, az * R2D, el * R2D);
            synth_cfg.mode[sv] = SYNTH_NONE;
            continue;
          }
        }

        fprintf(stderr, "Synthetic PRN %02d: az=%.1f el=%.1f deg\n", sv + 1,
                az * R2D, el * R2D);
      } else if (synth_cfg.mode[sv] == SYNTH_FORCE) {
        if (eph[ieph][sv].vflg != 1) {
          fprintf(stderr,
                  "WARNING: PRN %d forced but no ephemeris. Skipping.\n",
                  sv + 1);
          synth_cfg.mode[sv] = SYNTH_NONE;
        } else {
          fprintf(stderr, "Forced PRN %02d (below-horizon bypass)\n", sv + 1);
        }
      }
    }

    refreshSyntheticEphemerisSet(&synth_eph, eph[ieph], &ionoutc, &synth_cfg,
                                 xyz[0], synth_ref);
  }

  overlaySyntheticEphemerisSet(active_eph, eph[ieph], &synth_cfg, &synth_eph);

  ////////////////////////////////////////////////////////////
  // Allocate IQ buffer and channels
  ////////////////////////////////////////////////////////////

  iq_buff = (short *)calloc(2 * iq_buff_size, sizeof(short));
  if (!iq_buff) {
    fprintf(stderr, "ERROR: Failed to allocate IQ buffer.\n");
    status = 1;
    goto cleanup_module;
  }

  for (i = 0; i < MAX_CHAN; i++)
    chan[i].prn = 0;
  for (sv = 0; sv < MAX_SAT; sv++)
    allocatedSat[sv] = -1;

  grx = incGpsTime(g0, 0.0);
  allocateChannel(chan, active_eph, ionoutc, grx, xyz[0], elvmask, &attack_cfg,
                  &synth_cfg);

  for (i = 0; i < MAX_CHAN; i++) {
    if (chan[i].prn > 0)
      fprintf(stderr, "%02d %6.1f %5.1f %11.1f %5.1f\n", chan[i].prn,
              chan[i].azel[0] * R2D, chan[i].azel[1] * R2D, chan[i].rho0.d,
              chan[i].rho0.iono_delay);
  }

  // Antenna gain pattern
  for (i = 0; i < 37; i++)
    ant_pat[i] = pow(10.0, -ant_pat_db[i] / 20.0);

  if (attack_enabled) {
    fprintf(stderr, "Attack config:\n");
    for (sv = 0; sv < MAX_SAT; sv++) {
      if (attack_cfg.method[sv] != ATTACK_METHOD_NONE)
        fprintf(stderr, "  PRN %02d -> %s\n", sv + 1,
                attackMethodName(attack_cfg.method[sv]));
    }
  }
  if (attack_cfg.partial_mode) {
    fprintf(stderr, "Partial constellation mode: rendering only PRNs");
    for (sv = 0; sv < MAX_SAT; sv++)
      if (attack_cfg.prn_select[sv])
        fprintf(stderr, " %d", sv + 1);
    fprintf(stderr, "\n");
    if (attack_cfg.gain_boost_db != 0.0)
      fprintf(stderr, "  Power boost: +%.1f dB\n", attack_cfg.gain_boost_db);
  }

  ////////////////////////////////////////////////////////////
  // Pre-buffer phase
  ////////////////////////////////////////////////////////////

  {
    fprintf(stderr, "\n[TX] Pre-buffering %d epochs (%.1f s) ...\n",
            prebuffer_count, prebuffer_count * 0.1);

    // Allocate ring buffer for pre-buffered epochs
    int ring_size = prebuffer_count + 1;
    short **ring = (short **)calloc(ring_size, sizeof(short *));
    int *ring_sample_counts = (int *)calloc(ring_size, sizeof(int));
    for (i = 0; i < ring_size; i++)
      ring[i] = (short *)calloc(2 * iq_buff_size, sizeof(short));

    int ring_write = 0;
    int ring_read = 0;
    int ring_count = 0;

    installSignalHandlers();

    // Advance to first epoch
    ring_sample_counts[ring_write] = nextEpochSampleCount(&epoch_plan);
    current_epoch_duration =
        getEpochDurationSec(ring_sample_counts[ring_write], samp_freq);
    grx = incGpsTime(grx, current_epoch_duration);

    // Generate pre-buffer epochs
    for (int pb = 0; pb < prebuffer_count && !stop_requested; pb++) {
      int sample_count = ring_sample_counts[ring_write];

      generateEpoch(ring[ring_write], sample_count, chan, gain, active_eph,
                    &ionoutc, grx, staticLocationMode, current_epoch_duration,
                    delt, path_loss_enable, fixed_gain, ant_pat, attack_enabled,
                    &attack_cfg, attack_noise_state, jam_js_linear);

      ring_write = (ring_write + 1) % ring_size;
      ring_count++;
      ring_sample_counts[(ring_write - 1 + ring_size) % ring_size] =
          sample_count;

      // 30-second nav/channel refresh
      igrx = (int)(grx.sec * 10.0 + 0.5);
      if (igrx % (int)(SYNTH_EPHEM_REFRESH_SEC * 10.0 + 0.5) == 0)
        refreshNavState(chan, eph, neph, &ieph, active_eph, &synth_eph,
                        &synth_cfg, &ionoutc, grx, elvmask, &attack_cfg);

      ring_sample_counts[ring_write] = nextEpochSampleCount(&epoch_plan);
      current_epoch_duration =
          getEpochDurationSec(ring_sample_counts[ring_write], samp_freq);
      grx = incGpsTime(grx, current_epoch_duration);
      iumd = pb + 2;
    }

    fprintf(stderr, "[TX] Pre-buffer complete.\n");

    ////////////////////////////////////////////////////////////
    // Schedule timed TX start
    //
    // Strategy: compute the total delay until the GPS epoch should
    // be transmitted, sleep for most of it, THEN read the bladeRF
    // device timestamp and schedule TX with a short ~300ms lead.
    //
    // Why: the bladeRF FPGA doesn't consume queued samples until
    // the timed TX start fires.  If we schedule 2+ seconds ahead
    // and immediately start queuing, the 32-buffer pool fills up
    // before the FPGA begins draining, and bladerf_sync_tx times
    // out waiting for free buffers.  By sleeping first and only
    // scheduling with a short lead, the FPGA starts consuming
    // almost immediately after we begin sending.
    ////////////////////////////////////////////////////////////

    // The short lead between bladerf_get_timestamp() and the
    // scheduled TX start.  Must be long enough to queue the first
    // epoch + pre-buffer drain, short enough that the FPGA starts
    // consuming before the buffer pool fills.
    // 300ms = 780,000 samples = ~24 of 32 internal buffers.
    static const double BLADE_TX_LEAD_SEC = 0.3;

    // Phase 1: compute total remaining delay until TX should start
    double total_remaining_sec = (double)tx_advance_ns * 1.0e-9;

    if (trimble_mode && trimble_tag_mono >= 0.0) {
      double tag_lead_sec = (double)trimble_tag_lead_ms * 1.0e-3;
      double planned_delay = tag_lead_sec + (double)trimble_start_offset;
      double prep_elapsed = getMonotonicSeconds() - trimble_tag_mono;
      total_remaining_sec = planned_delay - prep_elapsed;

      fprintf(stderr, "[TRIMBLE] Tag lead estimate:     %.3f s\n",
              tag_lead_sec);
      fprintf(stderr, "[TRIMBLE] Planned delay (lead + offset): %.3f s\n",
              planned_delay);
      fprintf(stderr, "[TRIMBLE] Prep elapsed since tag: %.3f ms\n",
              prep_elapsed * 1000.0);
      fprintf(stderr, "[TRIMBLE] Remaining until TX epoch: %.3f ms\n",
              total_remaining_sec * 1000.0);

      if (total_remaining_sec < TX_START_LEAD_MIN_SEC) {
        fprintf(
            stderr,
            "[TRIMBLE] ERROR: Target second is stale (remaining %.3f ms "
            "< minimum %.3f ms).\n"
            "  Increase --trimble-start-offset-sec or reduce --prebuffer.\n",
            total_remaining_sec * 1000.0, TX_START_LEAD_MIN_SEC * 1000.0);
        for (i = 0; i < ring_size; i++)
          free(ring[i]);
        free(ring);
        free(ring_sample_counts);
        free(iq_buff);
        status = 1;
        goto cleanup_module;
      }

    } else if (wall_clock_latch_sec >= 0.0) {
      double prep_elapsed_sec =
          getWallClockRealtimeSeconds() - wall_clock_latch_sec;

      fprintf(stderr, "[TIMING] Startup prep after wall-clock latch: %.3f ms\n",
              prep_elapsed_sec * 1000.0);

      total_remaining_sec -= prep_elapsed_sec;
      if (total_remaining_sec < TX_START_LEAD_MIN_SEC) {
        fprintf(stderr,
                "[TIMING] WARNING: startup prep exceeded requested TX lead. "
                "Using minimum margin.\n");
        total_remaining_sec = TX_START_LEAD_MIN_SEC;
      }
    }

    // Phase 2: sleep for most of the delay, leaving only a short lead
    {
      double wait_sec = total_remaining_sec - BLADE_TX_LEAD_SEC;
      if (wait_sec > 0.05) {
        fprintf(stderr,
                "[TX] Waiting %.1f s for TX epoch (then %.0f ms lead) ...\n",
                wait_sec, BLADE_TX_LEAD_SEC * 1000.0);
        struct timespec ts_wait;
        ts_wait.tv_sec = (time_t)floor(wait_sec);
        ts_wait.tv_nsec = (long)((wait_sec - floor(wait_sec)) * 1.0e9);
        nanosleep(&ts_wait, NULL);
      }
    }

    // Phase 3: recompute actual remaining after sleep (may be imprecise)
    double blade_lead_sec;
    if (trimble_mode && trimble_tag_mono >= 0.0) {
      double tag_lead_sec = (double)trimble_tag_lead_ms * 1.0e-3;
      double planned_delay = tag_lead_sec + (double)trimble_start_offset;
      double elapsed = getMonotonicSeconds() - trimble_tag_mono;
      blade_lead_sec = planned_delay - elapsed;
    } else if (wall_clock_latch_sec >= 0.0) {
      double elapsed = getWallClockRealtimeSeconds() - wall_clock_latch_sec;
      blade_lead_sec = (double)tx_advance_ns * 1.0e-9 - elapsed;
    } else {
      blade_lead_sec = BLADE_TX_LEAD_SEC;
    }

    if (blade_lead_sec < TX_START_LEAD_MIN_SEC)
      blade_lead_sec = TX_START_LEAD_MIN_SEC;

    // Phase 4: read device timestamp and schedule with the short lead
    bladerf_timestamp blade_now = 0;
    status = bladerf_get_timestamp(dev, BLADERF_TX, &blade_now);
    if (status != 0) {
      fprintf(stderr, "ERROR: Failed to query bladeRF TX timestamp: %s\n",
              bladerf_strerror(status));
      for (i = 0; i < ring_size; i++)
        free(ring[i]);
      free(ring);
      free(ring_sample_counts);
      free(iq_buff);
      goto cleanup_module;
    }

    bladerf_timestamp lead_samples =
        (bladerf_timestamp)(blade_lead_sec * samp_freq + 0.5);
    bladerf_timestamp tx_start_ts = blade_now + lead_samples;

    fprintf(stderr, "\n[TX] bladeRF device timestamp: %llu samples\n",
            (unsigned long long)blade_now);
    fprintf(stderr, "[TX] TX lead after sleep: %.1f ms = %llu samples\n",
            blade_lead_sec * 1000.0, (unsigned long long)lead_samples);
    fprintf(stderr,
            "[TX] Scheduled TX start: %llu samples (device_now + lead)\n",
            (unsigned long long)tx_start_ts);

    // Phase 5: send first pre-buffered epoch with timed burst start
    {
      struct bladerf_metadata meta;
      memset(&meta, 0, sizeof(meta));
      meta.flags = BLADERF_META_FLAG_TX_BURST_START;
      meta.timestamp = tx_start_ts;

      status = bladerf_sync_tx(dev, ring[ring_read],
                               (unsigned int)ring_sample_counts[ring_read],
                               &meta, TIMEOUT_MS);
      if (status != 0) {
        fprintf(stderr, "ERROR: First timed TX send failed: %s\n",
                bladerf_strerror(status));
        for (i = 0; i < ring_size; i++)
          free(ring[i]);
        free(ring);
        free(ring_sample_counts);
        free(iq_buff);
        goto cleanup_module;
      }
    }
    ring_read = (ring_read + 1) % ring_size;
    ring_count--;

    // Drain remaining pre-buffered epochs into the device.
    // With the short lead, the FPGA will start consuming soon,
    // so these sends should not stall.
    while (ring_count > 0 && !stop_requested) {
      struct bladerf_metadata meta;
      memset(&meta, 0, sizeof(meta));

      status = bladerf_sync_tx(dev, ring[ring_read],
                               (unsigned int)ring_sample_counts[ring_read],
                               &meta, TIMEOUT_MS);
      if (status != 0) {
        fprintf(stderr, "[BLADE] Pre-buffer drain failed: %s\n",
                bladerf_strerror(status));
        break;
      }
      ring_read = (ring_read + 1) % ring_size;
      ring_count--;
    }

    fprintf(stderr, "[TX] Streaming ...\n\n");

    ////////////////////////////////////////////////////////////
    // Main streaming loop
    ////////////////////////////////////////////////////////////

    while (status == 0 && !stop_requested &&
           (stream_forever == TRUE || iumd < numd)) {
      int sample_count = ring_sample_counts[ring_write];

      // Generate next epoch into ring buffer
      generateEpoch(ring[ring_write], sample_count, chan, gain, active_eph,
                    &ionoutc, grx, staticLocationMode, current_epoch_duration,
                    delt, path_loss_enable, fixed_gain, ant_pat, attack_enabled,
                    &attack_cfg, attack_noise_state, jam_js_linear);
      ring_write = (ring_write + 1) % ring_size;
      ring_count++;

      // Send oldest buffered epoch (continuation — no timestamp needed)
      if (ring_count > 0) {
        struct bladerf_metadata meta;
        memset(&meta, 0, sizeof(meta));
        // No flags — continuation of burst, bladeRF auto-advances timestamp

        status = bladerf_sync_tx(dev, ring[ring_read],
                                 (unsigned int)ring_sample_counts[ring_read],
                                 &meta, TIMEOUT_MS);
        if (status != 0) {
          if (!stop_requested)
            fprintf(stderr, "\n[BLADE] TX error: %s\n",
                    bladerf_strerror(status));
          break;
        }
        ring_read = (ring_read + 1) % ring_size;
        ring_count--;
      }

      // 30-second nav/channel refresh
      igrx = (int)(grx.sec * 10.0 + 0.5);
      if (igrx % (int)(SYNTH_EPHEM_REFRESH_SEC * 10.0 + 0.5) == 0) {
        refreshNavState(chan, eph, neph, &ieph, active_eph, &synth_eph,
                        &synth_cfg, &ionoutc, grx, elvmask, &attack_cfg);

        if (verb) {
          fprintf(stderr, "\n");
          for (i = 0; i < MAX_CHAN; i++)
            if (chan[i].prn > 0)
              fprintf(stderr, "%02d %6.1f %5.1f %11.1f %5.1f\n", chan[i].prn,
                      chan[i].azel[0] * R2D, chan[i].azel[1] * R2D,
                      chan[i].rho0.d, chan[i].rho0.iono_delay);
        }
      }

      ring_sample_counts[ring_write] = nextEpochSampleCount(&epoch_plan);
      current_epoch_duration =
          getEpochDurationSec(ring_sample_counts[ring_write], samp_freq);
      grx = incGpsTime(grx, current_epoch_duration);
      iumd++;

      fprintf(stderr, "\rTime into run = %4.1f", subGpsTime(grx, g0));
      fflush(stderr);
    }

    ////////////////////////////////////////////////////////////
    // Shutdown
    ////////////////////////////////////////////////////////////

    fprintf(stderr, "\n\n[TX] Shutting down ...\n");

    // Send end-of-burst
    {
      struct bladerf_metadata meta;
      memset(&meta, 0, sizeof(meta));
      meta.flags = BLADERF_META_FLAG_TX_BURST_END;

      int16_t zero_buf[2] = {0, 0};
      bladerf_sync_tx(dev, zero_buf, 1, &meta, TIMEOUT_MS);
    }

    // Cleanup ring buffer
    for (i = 0; i < ring_size; i++)
      free(ring[i]);
    free(ring);
    free(ring_sample_counts);
  }

  free(iq_buff);

cleanup_module:
  bladerf_enable_module(dev, BLADERF_MODULE_TX, false);

cleanup_dev:
  fprintf(stderr, "[BLADE] Closing device...\n");
  bladerf_close(dev);

  return status != 0 ? 1 : 0;
}
