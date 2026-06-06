/*
 * x300tx.cpp — Integrated USRP X300 GPS signal generator and transmitter.
 *
 * Generates GPS L1 C/A IQ samples in-process and transmits them directly
 * via the UHD C++ API with timed TX metadata.  Eliminates the pipe-based
 * latency of the old gps-sdr-sim | gps-sdr-sim-uhd.py path.
 *
 * Build:  make x300tx   (from the project root)
 *
 * Copyright (c) 2024-2026  — same licence as gps-sdr-sim
 */

#include <uhd/stream.hpp>
#include <uhd/types/metadata.hpp>
#include <uhd/usrp/multi_usrp.hpp>

#include <algorithm>
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

#include "player/rtcm3_nav.hpp"

extern "C" {
#include "gpssim.h"
}

////////////////////////////////////////////////////////////
// Constants
////////////////////////////////////////////////////////////

#define TX_FREQUENCY 1575420000.0            // GPS L1 (Hz)
#define TX_SAMPLERATE 2500000.0              // 2.5 Msps, exact from X300 200 MHz clock
#define PREBUFFER_DEFAULT 5                  // 0.5 s pre-buffer
#define TX_START_LEAD_DEFAULT_NS 250000000LL // 250 ms startup lead
#define TX_START_LEAD_MAX_SEC 60.0
#define TX_START_LEAD_MIN_SEC 0.02
#define EPOCH_TARGET_SEC 0.1
#define GPS_TIME_PPM_DEFAULT 0.0
#define GPS_TIME_PPM_MAX_ABS 1000.0

// Trimble time-tag mode defaults
#define TRIMBLE_START_OFFSET_DEFAULT 2
#define TRIMBLE_TAG_LEAD_MS_DEFAULT 500
#define TRIMBLE_TIMEOUT_MS_DEFAULT 30000
#define TRIMBLE_LEAP_SEC_DEFAULT 18
#define TRIMBLE_TX_CAL_NS_DEFAULT 0LL
#define TRIMBLE_RTCM_PORT_DEFAULT 5018
#define TRIMBLE_RTCM_WARMUP_DEFAULT 30
#define TRIMBLE_RTCM_MIN_PRNS_DEFAULT 16

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

  // Set a connect timeout via poll on non-blocking socket
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

/*
 * Scan a raw byte buffer for the 4-byte sequence "UTC ".
 * The stream may be binary GSOF with embedded ASCII time tags,
 * so we cannot rely on C string functions (buffer may contain \0).
 * Returns pointer to the 'U' of the first "UTC " found, or NULL.
 */
static const char *findUtcMarker(const char *buf, int len) {
  for (int i = 0; i <= len - 4; i++) {
    if (buf[i] == 'U' && buf[i+1] == 'T' &&
        buf[i+2] == 'C' && buf[i+3] == ' ')
      return &buf[i];
  }
  return NULL;
}

/*
 * Read from a Trimble TCP stream (which may be binary GSOF) and extract
 * the first valid "1 PPS TIME TAG" ASCII time tag.
 *
 * The tag format is: "UTC YY.MM.DD HH:MM:SS SS\r\n"
 * embedded inside binary GSOF framing.  We scan the raw byte stream
 * for the "UTC " marker and parse from there.
 *
 * Returns 1 on success, 0 on failure/timeout.
 */
static int trimbleReadTag(int fd, int timeout_ms,
                          int *yy, int *mm, int *dd,
                          int *hh, int *min, int *sec) {
  char buf[4096];
  int pos = 0;

  double deadline = getMonotonicSeconds() + (double)timeout_ms * 1.0e-3;

  while (getMonotonicSeconds() < deadline) {
    int remaining_ms = (int)((deadline - getMonotonicSeconds()) * 1000.0);
    if (remaining_ms <= 0) break;

    struct pollfd pfd = {fd, POLLIN, 0};
    int ret = poll(&pfd, 1, remaining_ms);
    if (ret <= 0) break;

    ssize_t n = read(fd, buf + pos, sizeof(buf) - (size_t)pos - 1);
    if (n <= 0) break;
    pos += (int)n;

    // Scan for "UTC " in the accumulated buffer
    const char *utc = findUtcMarker(buf, pos);
    if (utc) {
      int remaining_bytes = pos - (int)(utc - buf);

      // Need at least "UTC YY.MM.DD HH:MM:SS" = 23 chars
      if (remaining_bytes >= 23) {
        // Copy into a NUL-terminated temp for sscanf
        char tmp[64];
        int copy_len = remaining_bytes < (int)sizeof(tmp) - 1
                       ? remaining_bytes : (int)sizeof(tmp) - 1;
        memcpy(tmp, utc, copy_len);
        tmp[copy_len] = '\0';

        int nn = sscanf(tmp + 4, "%d.%d.%d %d:%d:%d",
                        yy, mm, dd, hh, min, sec);
        if (nn >= 6 &&
            *yy >= 0 && *yy <= 99 &&
            *mm >= 1 && *mm <= 12 &&
            *dd >= 1 && *dd <= 31 &&
            *hh >= 0 && *hh <= 23 &&
            *min >= 0 && *min <= 59 &&
            *sec >= 0 && *sec <= 59) {
          return 1;
        }
      }
      // Partial tag at end of buffer — shift and keep reading
    }

    // Prevent buffer overflow: keep only the tail so a split "UTC " is not lost
    if (pos > (int)sizeof(buf) - 512) {
      int keep = 64;
      memmove(buf, buf + pos - keep, keep);
      pos = keep;
    }
  }

  return 0;
}

/*
 * Convert a parsed Trimble UTC tag + offset + leap seconds to GPS epoch.
 * tagged UTC + offset_sec → target UTC → + leap_sec → GPS time → date2gps.
 */
static void trimbleUtcToGpsEpoch(int yy, int mm, int dd, int hh, int min,
                                 int sec, int offset_sec, int leap_sec,
                                 datetime_t *t0, gpstime_t *g0) {
  struct tm tm_utc = {};
  tm_utc.tm_year = (yy >= 80 ? yy : yy + 100); // 2-digit year
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
          "[TIMING] x300tx wall-clock latch: %lld.%09ld s (CLOCK_REALTIME)\n",
          (long long)floor(wall_clock_sec),
          (long)((wall_clock_sec - floor(wall_clock_sec)) * 1.0e9));

  epochSecondsToUtcDateTime(gps_time_sec, t0);
  date2gps(t0, g0);

  fprintf(stderr,
          "[TIMING] x300tx GPS epoch: week %d  tow %.9f s  "
          "(lead %.3f s included)\n",
          g0->week, g0->sec, lead_sec);

  return wall_clock_sec;
}

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

static gpstime_t incGpsTimePrecise(gpstime_t g0, double dt) {
  gpstime_t g1;

  g1.week = g0.week;
  g1.sec = g0.sec + dt;

  while (g1.sec >= SECONDS_IN_WEEK) {
    g1.sec -= SECONDS_IN_WEEK;
    g1.week++;
  }

  while (g1.sec < 0.0) {
    g1.sec += SECONDS_IN_WEEK;
    g1.week--;
  }

  return g1;
}

static double getGpsTimeScale(double gps_time_ppm) {
  return 1.0 + gps_time_ppm * 1.0e-6;
}

static double getGpsElapsedFromSamples(long long sample_offset,
                                       double sample_rate_hz,
                                       double gps_time_ppm) {
  return ((double)sample_offset / sample_rate_hz) *
         getGpsTimeScale(gps_time_ppm);
}

static gpstime_t getGpsTimeAtSampleOffset(gpstime_t first_sample_gps_time,
                                          long long sample_offset,
                                          double sample_rate_hz,
                                          double gps_time_ppm) {
  return incGpsTimePrecise(
      first_sample_gps_time,
      getGpsElapsedFromSamples(sample_offset, sample_rate_hz, gps_time_ppm));
}

static int hasCloneMode(const synth_config_t *cfg) {
  for (int sv = 0; sv < MAX_SAT; sv++) {
    if (cfg->mode[sv] == SYNTH_CLONE)
      return TRUE;
  }

  return FALSE;
}

static int hasReviveMode(const synth_config_t *cfg) {
  for (int sv = 0; sv < MAX_SAT; sv++) {
    if (cfg->mode[sv] == SYNTH_REVIVE)
      return TRUE;
  }

  return FALSE;
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

  // Per-channel: compute range, code phase, gain
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

  // Per-sample: accumulate IQ from all channels
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

        // Update code phase
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
                            const ephem_t synth_source[][MAX_SAT], int neph,
                            int *ieph, ephem_t *active_eph,
                            synth_ephem_store_t *synth_eph,
                            const synth_config_t *synth_cfg,
                            const ionoutc_t *ionoutc, gpstime_t grx,
                            double elvmask, int trimble_rtcm_mode,
                            int *trimble_rtcm_alive,
                            rtcm3_nav_stream_t *trimble_rtcm_stream,
                            const attack_config_t *attack_cfg) {
  int i;
  int eph_changed = FALSE;

  if (trimble_rtcm_mode == TRUE && trimble_rtcm_alive != NULL &&
      *trimble_rtcm_alive == TRUE && trimble_rtcm_stream != NULL) {
    char err[RTCM3_NAV_ERR_SIZE];
    int rtcm_updated = FALSE;

    if (rtcm3_nav_pump(trimble_rtcm_stream, 0, &rtcm_updated, err,
                       sizeof(err)) == FALSE) {
      fprintf(stderr,
              "\n[RTCM] WARNING: %s. Keeping last cached ephemerides.\n", err);
      rtcm3_nav_close(trimble_rtcm_stream);
      *trimble_rtcm_alive = FALSE;
    } else if (rtcm_updated == TRUE) {
      rtcm3_nav_copy_ephemeris(trimble_rtcm_stream, eph[0]);
      eph_changed = TRUE;
    }
  }

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

    if (refreshSyntheticEphemerisSet(synth_eph, synth_source, neph, eph[*ieph],
                                     ionoutc, synth_cfg, xyz[0],
                                     synth_ref) == TRUE)
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

static void x300_usage(void) {
  fprintf(
      stderr,
      "Usage: x300tx [options]\n"
      "\n"
      "GPS simulation options:\n"
      "  -e <rinex_nav>              Navigation RINEX file\n"
      "  -l <lat,lon,alt>            Static location (deg,deg,m)\n"
      "  -c <x,y,z>                  Static ECEF position (m)\n"
      "  -t <YYYY/MM/DD,hh:mm:ss>   Start time (UTC)\n"
      "  -n                          Stream-now mode (wall clock)\n"
      "  -d <seconds>                Duration\n"
      "  -P <prn[,prn...]>           Partial constellation PRN list\n"
      "  -S <synth_spec>             Synthetic satellites. One family per -S:\n"
      "                              classic: PRN:force | PRN:overhead | "
      "PRN:az/el\n"
      "                              clone:   PRN:clone=<src_prn>\n"
      "                              revive:  PRN:revive (requires -e)\n"
      "  -A <attack_spec>            Attack config\n"
      "  -J <dB>                     Jammer-to-signal ratio\n"
      "  -G <dB>                     Power boost for partial mode\n"
      "  -r <lead_sec>               Future TX start lead for -n mode\n"
      "  -p [fixed_gain]             Disable path loss\n"
      "  -i                          Disable ionospheric correction\n"
      "  -v                          Verbose\n"
      "\n"
      "USRP X300 options:\n"
      "  --gps-week <N>              GPS week number\n"
      "  --gps-tow <sec>             GPS time of week (seconds)\n"
      "  --tx-advance-ns <ns>        Future TX start lead (default 250000000)\n"
      "  --addr <ip>                 USRP address (default 192.168.10.2)\n"
      "  --rate <Hz>                 Requested TX sample rate (default "
      "2500000)\n"
      "  --gps-time-ppm <ppm>        Scale generated GPS elapsed time "
      "(default 0)\n"
      "  --tx-time-scale-ppm <ppm>   Alias for --gps-time-ppm\n"
      "  --gain <dB>                 TX gain (default 0)\n"
      "  --txvga1 <dB>               Accepted BladeRF compatibility option; "
      "use --gain for X300\n"
      "  --txvga2 <dB>               Accepted BladeRF compatibility option; "
      "use --gain for X300\n"
      "  --clock-source <src>        internal/external/gpsdo (default "
      "internal)\n"
      "  --time-source <src>         internal/external/gpsdo (default "
      "internal)\n"
      "  --prebuffer <N>             Pre-buffer epochs (default 5)\n"
      "  --channel <N>               TX channel index (default 0; often 1 "
      "for RF B)\n"
      "  --antenna <name>            TX antenna (default TX/RX)\n"
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
      "Trimble RTCM ephemeris options:\n"
      "  --trimble-rtcm-host <host>      RTCM TCP/NTRIP host\n"
      "  --trimble-rtcm-port <port>      RTCM port (default 5018)\n"
      "  --trimble-rtcm-mount <name>     Optional NTRIP mount point\n"
      "  --trimble-rtcm-user <u[:p]>     Optional NTRIP credentials\n"
      "  --trimble-rtcm-timeout-ms <ms>  RTCM connect/read timeout (default "
      "30000)\n"
      "  --trimble-rtcm-warmup-sec <s>   Warmup before TX (default 30)\n"
      "  --trimble-rtcm-min-prns <n>     Minimum GPS 1019 PRNs (default 16)\n"
      "\n"
      "Examples:\n"
      "  x300tx -e hour0910.26n -l 21.0047844,105.8460541,5 \\\n"
      "    -P 3,4,7,8 -S 3:0/60,4:90/45,7:180/30,8:45/55 \\\n"
      "    --gps-week 2361 --gps-tow 118800.0 --gain 0\n"
      "\n"
      "  x300tx -e hour0910.26n -l 21.0047844,105.8460541,5 \\\n"
      "    -P 3,4,7,8 -S 3:0/60,4:90/45,7:180/30,8:45/55 \\\n"
      "    --trimble-time-tag-host 192.168.5.245 \\\n"
      "    --trimble-time-tag-port 5017 \\\n"
      "    --trimble-start-offset-sec 2 --gain 0\n"
      "\n"
      "  x300tx -e hour1120.26n -l 21.0047844,105.8460541,22 \\\n"
      "    -P 22,14,30 -S 22:revive,14:revive,30:revive \\\n"
      "    --trimble-time-tag-host 192.168.5.245 \\\n"
      "    --trimble-time-tag-port 5017 \\\n"
      "    --trimble-start-offset-sec 2 --txvga1 -35 \\\n"
      "    --trimble-tag-lead-ms 788 --trimble-tx-cal-ns 580000\n"
      "\n");
}

////////////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////////////

int main(int argc, char *argv[]) {
  int sv, i;
  int neph, ieph;
  ephem_t eph[EPHEM_ARRAY_SIZE][MAX_SAT];
  ephem_t revive_scan_eph[EPHEM_ARRAY_SIZE][MAX_SAT];
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
  double gps_time_ppm = GPS_TIME_PPM_DEFAULT;

  int timeoverwrite = FALSE;
  int has_revive_mode = FALSE;
  int attack_enabled = FALSE;
  unsigned int attack_noise_state[MAX_SAT];
  double jam_js_linear = 10.0;

  ionoutc_t ionoutc;
  attack_config_t attack_cfg;
  synth_config_t synth_cfg;
  int path_loss_enable = TRUE;

  // X300-specific options
  int gps_week_set = FALSE;
  int gps_tow_set = FALSE;
  int explicit_gps_week = -1;
  double explicit_gps_tow = 0.0;
  long long tx_advance_ns = TX_START_LEAD_DEFAULT_NS;
  char usrp_addr[64] = "192.168.10.2";
  double tx_gain = 0.0;
  int tx_gain_set = FALSE;
  int compat_txvga1_set = FALSE;
  int compat_txvga2_set = FALSE;
  int compat_txvga1 = 0;
  int compat_txvga2 = 0;
  char clock_source[32] = "internal";
  char time_source[32] = "internal";
  int prebuffer_count = PREBUFFER_DEFAULT;
  size_t tx_channel = 0;
  char tx_antenna[32] = "TX/RX";

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

  // Trimble RTCM ephemeris mode
  char trimble_rtcm_host[256] = "";
  int trimble_rtcm_port = 0;
  char trimble_rtcm_mount[256] = "";
  char trimble_rtcm_user[256] = "";
  int trimble_rtcm_mode = FALSE;
  int trimble_rtcm_alive = FALSE;
  int trimble_rtcm_timeout_ms = TRIMBLE_TIMEOUT_MS_DEFAULT;
  int trimble_rtcm_warmup_sec = TRIMBLE_RTCM_WARMUP_DEFAULT;
  int trimble_rtcm_min_prns = TRIMBLE_RTCM_MIN_PRNS_DEFAULT;
  rtcm3_nav_stream_t trimble_rtcm_stream;

  epoch_plan_t epoch_plan;
  size_t max_samps = 0;
  uhd::tx_streamer::sptr tx_stream;
  gpstime_t first_sample_gps_time;
  long long generated_samples = 0;
  long long emitted_samples = 0;

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
  rtcm3_nav_init(&trimble_rtcm_stream);

  if (argc == 2 &&
      (strcmp(argv[1], "--help") == 0 || strcmp(argv[1], "-h") == 0)) {
    x300_usage();
    return 0;
  }

  if (argc < 3) {
    x300_usage();
    return 1;
  }

  std::vector<char *> short_argv;
  short_argv.push_back(argv[0]);

  for (i = 1; i < argc; i++) {
    if (strncmp(argv[i], "--", 2) != 0) {
      short_argv.push_back(argv[i]);
      continue;
    }

    const char *opt = argv[i] + 2;

    if (strcmp(opt, "help") == 0) {
      x300_usage();
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
    if (strcmp(opt, "addr") == 0) {
      strncpy(usrp_addr, val, sizeof(usrp_addr) - 1);
      usrp_addr[sizeof(usrp_addr) - 1] = '\0';
      continue;
    }
    if (strcmp(opt, "rate") == 0) {
      char *end = NULL;
      errno = 0;
      double parsed = strtod(val, &end);
      if (val[0] == '\0' || errno == ERANGE || end == NULL || *end != '\0' ||
          !std::isfinite(parsed) || parsed <= 0.0) {
        fprintf(stderr, "ERROR: --rate must be a positive number of Hz.\n");
        return 1;
      }
      samp_freq = parsed;
      continue;
    }
    if (strcmp(opt, "gps-time-ppm") == 0 ||
        strcmp(opt, "tx-time-scale-ppm") == 0) {
      char *end = NULL;
      errno = 0;
      double parsed = strtod(val, &end);
      if (val[0] == '\0' || errno == ERANGE || end == NULL || *end != '\0' ||
          !std::isfinite(parsed) || fabs(parsed) > GPS_TIME_PPM_MAX_ABS) {
        fprintf(stderr,
                "ERROR: --%s must be a finite ppm value in the range "
                "-%.0f..%.0f.\n",
                opt, GPS_TIME_PPM_MAX_ABS, GPS_TIME_PPM_MAX_ABS);
        return 1;
      }
      gps_time_ppm = parsed;
      continue;
    }
    if (strcmp(opt, "gain") == 0) {
      tx_gain = atof(val);
      tx_gain_set = TRUE;
      continue;
    }
    if (strcmp(opt, "txvga1") == 0) {
      compat_txvga1 = atoi(val);
      compat_txvga1_set = TRUE;
      if (compat_txvga1 < -35 || compat_txvga1 > -4) {
        fprintf(stderr, "ERROR: --txvga1 must be -35..-4 dB.\n");
        return 1;
      }
      continue;
    }
    if (strcmp(opt, "txvga2") == 0) {
      compat_txvga2 = atoi(val);
      compat_txvga2_set = TRUE;
      if (compat_txvga2 < 0 || compat_txvga2 > 25) {
        fprintf(stderr, "ERROR: --txvga2 must be 0..25 dB.\n");
        return 1;
      }
      continue;
    }
    if (strcmp(opt, "clock-source") == 0) {
      strncpy(clock_source, val, sizeof(clock_source) - 1);
      clock_source[sizeof(clock_source) - 1] = '\0';
      continue;
    }
    if (strcmp(opt, "time-source") == 0) {
      strncpy(time_source, val, sizeof(time_source) - 1);
      time_source[sizeof(time_source) - 1] = '\0';
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
    if (strcmp(opt, "channel") == 0) {
      char *end = NULL;
      errno = 0;
      unsigned long parsed = strtoul(val, &end, 10);
      if (val[0] == '\0' || val[0] == '-' || errno == ERANGE ||
          end == NULL || *end != '\0') {
        fprintf(stderr, "ERROR: --channel must be a non-negative integer.\n");
        return 1;
      }
      tx_channel = (size_t)parsed;
      continue;
    }
    if (strcmp(opt, "antenna") == 0) {
      strncpy(tx_antenna, val, sizeof(tx_antenna) - 1);
      tx_antenna[sizeof(tx_antenna) - 1] = '\0';
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
    if (strcmp(opt, "trimble-rtcm-host") == 0) {
      strncpy(trimble_rtcm_host, val, sizeof(trimble_rtcm_host) - 1);
      trimble_rtcm_host[sizeof(trimble_rtcm_host) - 1] = '\0';
      continue;
    }
    if (strcmp(opt, "trimble-rtcm-port") == 0) {
      trimble_rtcm_port = atoi(val);
      if (trimble_rtcm_port <= 0 || trimble_rtcm_port > 65535) {
        fprintf(stderr, "ERROR: --trimble-rtcm-port must be 1-65535.\n");
        return 1;
      }
      continue;
    }
    if (strcmp(opt, "trimble-rtcm-mount") == 0) {
      strncpy(trimble_rtcm_mount, val, sizeof(trimble_rtcm_mount) - 1);
      trimble_rtcm_mount[sizeof(trimble_rtcm_mount) - 1] = '\0';
      continue;
    }
    if (strcmp(opt, "trimble-rtcm-user") == 0) {
      strncpy(trimble_rtcm_user, val, sizeof(trimble_rtcm_user) - 1);
      trimble_rtcm_user[sizeof(trimble_rtcm_user) - 1] = '\0';
      continue;
    }
    if (strcmp(opt, "trimble-rtcm-timeout-ms") == 0) {
      trimble_rtcm_timeout_ms = atoi(val);
      if (trimble_rtcm_timeout_ms < 100 || trimble_rtcm_timeout_ms > 30000) {
        fprintf(stderr,
                "ERROR: --trimble-rtcm-timeout-ms must be 100-30000.\n");
        return 1;
      }
      continue;
    }
    if (strcmp(opt, "trimble-rtcm-warmup-sec") == 0) {
      trimble_rtcm_warmup_sec = atoi(val);
      if (trimble_rtcm_warmup_sec < 1 || trimble_rtcm_warmup_sec > 300) {
        fprintf(stderr, "ERROR: --trimble-rtcm-warmup-sec must be 1-300.\n");
        return 1;
      }
      continue;
    }
    if (strcmp(opt, "trimble-rtcm-min-prns") == 0) {
      trimble_rtcm_min_prns = atoi(val);
      if (trimble_rtcm_min_prns < 1 || trimble_rtcm_min_prns > MAX_SAT) {
        fprintf(stderr, "ERROR: --trimble-rtcm-min-prns must be 1-%d.\n",
                MAX_SAT);
        return 1;
      }
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
      x300_usage();
      return 0;
    case ':':
    case '?':
      x300_usage();
      return 1;
    default:
      break;
    }
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

  // Activate Trimble RTCM mode
  if (trimble_rtcm_host[0] != '\0') {
    trimble_rtcm_mode = TRUE;
    if (trimble_rtcm_port == 0)
      trimble_rtcm_port = TRIMBLE_RTCM_PORT_DEFAULT;
  } else if (trimble_rtcm_port > 0 || trimble_rtcm_mount[0] != '\0' ||
             trimble_rtcm_user[0] != '\0') {
    fprintf(stderr, "ERROR: --trimble-rtcm-host is required for RTCM mode.\n");
    return 1;
  }

  if (trimble_rtcm_user[0] != '\0' && trimble_rtcm_mount[0] == '\0') {
    fprintf(stderr,
            "ERROR: --trimble-rtcm-user requires --trimble-rtcm-mount.\n");
    return 1;
  }

  if (hasCloneMode(&synth_cfg) == TRUE && trimble_rtcm_mode == FALSE) {
    fprintf(stderr, "ERROR: Clone mode requires --trimble-rtcm-host.\n");
    return 1;
  }

  has_revive_mode = hasReviveMode(&synth_cfg);
  if (has_revive_mode == TRUE) {
    if (navfile[0] == 0) {
      fprintf(stderr, "ERROR: Revive mode requires -e ephemeris file.\n");
      return 1;
    }
    if (trimble_rtcm_mode == TRUE) {
      fprintf(stderr, "ERROR: Revive mode cannot use live RTCM ephemeris; "
                      "provide -e without --trimble-rtcm-host.\n");
      return 1;
    }
  }

  if (navfile[0] == 0 && trimble_rtcm_mode == FALSE) {
    fprintf(stderr, "ERROR: Navigation RINEX file is required (-e) unless "
                    "--trimble-rtcm-host is set.\n");
    return 1;
  }
  if (navfile[0] != 0 && trimble_rtcm_mode == TRUE) {
    fprintf(stderr, "WARNING: Ignoring navigation RINEX file because live RTCM "
                    "ephemeris is enabled.\n");
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

  if (trimble_rtcm_mode == TRUE) {
    char err[RTCM3_NAV_ERR_SIZE];
    rtcm3_nav_options_t rtcm_opt;
    double warmup_deadline;

    memset(&rtcm_opt, 0, sizeof(rtcm_opt));
    rtcm_opt.host = trimble_rtcm_host;
    rtcm_opt.port = trimble_rtcm_port;
    rtcm_opt.timeout_ms = trimble_rtcm_timeout_ms;
    rtcm_opt.mount_point =
        trimble_rtcm_mount[0] != '\0' ? trimble_rtcm_mount : NULL;
    rtcm_opt.credentials =
        trimble_rtcm_user[0] != '\0' ? trimble_rtcm_user : NULL;

    fprintf(stderr, "\n[RTCM] Connecting to %s:%d ...\n", trimble_rtcm_host,
            trimble_rtcm_port);
    if (rtcm_opt.mount_point != NULL) {
      fprintf(stderr, "[RTCM] NTRIP mount: %s\n", trimble_rtcm_mount);
      if (rtcm_opt.credentials != NULL)
        fprintf(stderr, "[RTCM] NTRIP auth: enabled\n");
    }

    if (rtcm3_nav_open(&trimble_rtcm_stream, &rtcm_opt, err, sizeof(err)) ==
        FALSE) {
      fprintf(stderr, "ERROR: %s\n", err);
      return 1;
    }

    trimble_rtcm_alive = TRUE;
    warmup_deadline = getMonotonicSeconds() + (double)trimble_rtcm_warmup_sec;
    fprintf(stderr,
            "[RTCM] Warming up up to %d s for at least %d GPS PRNs...\n",
            trimble_rtcm_warmup_sec, trimble_rtcm_min_prns);

    while (rtcm3_nav_valid_prns(&trimble_rtcm_stream) < trimble_rtcm_min_prns &&
           getMonotonicSeconds() < warmup_deadline) {
      int updated = FALSE;
      int wait_ms = (int)((warmup_deadline - getMonotonicSeconds()) * 1000.0);

      if (wait_ms < 0)
        wait_ms = 0;
      if (wait_ms > 1000)
        wait_ms = 1000;

      if (rtcm3_nav_pump(&trimble_rtcm_stream, wait_ms, &updated, err,
                         sizeof(err)) == FALSE) {
        fprintf(stderr, "ERROR: %s\n", err);
        rtcm3_nav_close(&trimble_rtcm_stream);
        return 1;
      }
    }

    if (rtcm3_nav_valid_prns(&trimble_rtcm_stream) < trimble_rtcm_min_prns) {
      fprintf(stderr,
              "ERROR: Only %d GPS ephemerides available from RTCM after %d s "
              "warmup (need %d).\n",
              rtcm3_nav_valid_prns(&trimble_rtcm_stream),
              trimble_rtcm_warmup_sec, trimble_rtcm_min_prns);
      rtcm3_nav_close(&trimble_rtcm_stream);
      return 1;
    }

    fprintf(stderr, "[RTCM] Warmup complete: %d GPS PRNs loaded.\n",
            rtcm3_nav_valid_prns(&trimble_rtcm_stream));

    memset(eph, 0, sizeof(eph));
    rtcm3_nav_copy_ephemeris(&trimble_rtcm_stream, eph[0]);
    neph = 1;

    {
      int iono_enable = ionoutc.enable;

      memset(&ionoutc, 0, sizeof(ionoutc));
      ionoutc.enable = iono_enable;
      ionoutc.leapen = FALSE;
      ionoutc.vflg = FALSE;
      ionoutc.dtls = trimble_leap_sec;
    }

    if (getSetReferenceToc(eph[0], &gmin) == FALSE) {
      fprintf(stderr, "ERROR: No RTCM ephemeris available after warmup.\n");
      rtcm3_nav_close(&trimble_rtcm_stream);
      return 1;
    }
    gps2date(&gmin, &tmin);
    gmax = gmin;
    tmax = tmin;

    for (sv = 0; sv < MAX_SAT; sv++) {
      if (synth_cfg.mode[sv] != SYNTH_CLONE)
        continue;

      if (rtcm3_nav_has_prn(&trimble_rtcm_stream, synth_cfg.source_prn[sv]) !=
          TRUE) {
        fprintf(stderr,
                "ERROR: Clone donor PRN %d not found in RTCM cache after %d s "
                "warmup.\n",
                synth_cfg.source_prn[sv], trimble_rtcm_warmup_sec);
        rtcm3_nav_close(&trimble_rtcm_stream);
        return 1;
      }

      fprintf(stderr, "Clone PRN %02d <- donor PRN %02d (IODE=%d)\n", sv + 1,
              synth_cfg.source_prn[sv],
              eph[0][synth_cfg.source_prn[sv] - 1].iode);
    }
  } else {
    neph = readRinexNavAll(eph, &ionoutc, navfile);
    if (neph == 0) {
      fprintf(stderr, "ERROR: No ephemeris available.\n");
      return 1;
    } else if (neph == -1) {
      fprintf(stderr, "ERROR: Ephemeris file not found.\n");
      return 1;
    }

    if (has_revive_mode == TRUE)
      memcpy(revive_scan_eph, eph, sizeof(revive_scan_eph));

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
  }

  ////////////////////////////////////////////////////////////
  // Configure USRP X300
  ////////////////////////////////////////////////////////////

  fprintf(stderr, "\n[UHD] Configuring USRP X300 at %s ...\n", usrp_addr);

  uhd::device_addr_t dev_addr;
  dev_addr["addr"] = usrp_addr;

  uhd::usrp::multi_usrp::sptr usrp;
  try {
    usrp = uhd::usrp::multi_usrp::make(dev_addr);
  } catch (const std::exception &e) {
    fprintf(stderr, "ERROR: Failed to open USRP: %s\n", e.what());
    return 1;
  }

  usrp->set_clock_source(clock_source);
  usrp->set_time_source(time_source);

  {
    size_t tx_channels = usrp->get_tx_num_channels();
    if (tx_channel >= tx_channels) {
      fprintf(stderr,
              "ERROR: --channel %zu is invalid; device reports %zu TX "
              "channel(s).\n",
              tx_channel, tx_channels);
      return 1;
    }
  }

  fprintf(stderr, "[UHD] TX channel: %zu\n", tx_channel);

  usrp->set_tx_rate(samp_freq, tx_channel);
  fprintf(stderr, "[UHD] TX rate:    %.0f Hz (actual %.0f Hz)\n", samp_freq,
          usrp->get_tx_rate(tx_channel));

  uhd::tune_request_t tune_req(TX_FREQUENCY);
  usrp->set_tx_freq(tune_req, tx_channel);
  fprintf(stderr, "[UHD] TX freq:    %.0f Hz (actual %.0f Hz)\n", TX_FREQUENCY,
          usrp->get_tx_freq(tx_channel));

  if (compat_txvga1_set || compat_txvga2_set) {
    fprintf(stderr,
            "[UHD] BladeRF gain option(s) accepted for CLI compatibility:");
    if (compat_txvga1_set)
      fprintf(stderr, " txvga1=%d", compat_txvga1);
    if (compat_txvga2_set)
      fprintf(stderr, " txvga2=%d", compat_txvga2);
    fprintf(stderr, ". X300 RF gain is controlled by --gain");
    if (tx_gain_set == FALSE)
      fprintf(stderr, " (using default %.1f dB)", tx_gain);
    fprintf(stderr, ".\n");
  }

  {
    uhd::gain_range_t gain_range = usrp->get_tx_gain_range(tx_channel);
    double min_gain = gain_range.start();
    double max_gain = gain_range.stop();

    if (tx_gain < min_gain) {
      fprintf(stderr,
              "[UHD] WARNING: requested TX gain %.1f dB is below device "
              "minimum %.1f dB; clamping.\n",
              tx_gain, min_gain);
      tx_gain = min_gain;
    } else if (tx_gain > max_gain) {
      fprintf(stderr,
              "[UHD] WARNING: requested TX gain %.1f dB is above device "
              "maximum %.1f dB; clamping.\n",
              tx_gain, max_gain);
      tx_gain = max_gain;
    }
  }

  usrp->set_tx_gain(tx_gain, tx_channel);
  fprintf(stderr, "[UHD] TX gain:    %.1f dB (actual %.1f dB)\n", tx_gain,
          usrp->get_tx_gain(tx_channel));

  usrp->set_tx_antenna(tx_antenna, tx_channel);
  fprintf(stderr, "[UHD] TX antenna: %s\n",
          usrp->get_tx_antenna(tx_channel).c_str());
  fprintf(stderr, "[UHD] Clock src:  %s\n", clock_source);
  fprintf(stderr, "[UHD] Time src:   %s\n", time_source);

  // Create TX streamer — SC16 on both CPU and wire side
  uhd::stream_args_t stream_args("sc16", "sc16");
  stream_args.channels = {tx_channel};
  tx_stream = usrp->get_tx_stream(stream_args);

  max_samps = tx_stream->get_max_num_samps();
  fprintf(stderr, "[UHD] Max samples per send: %zu\n", max_samps);

  samp_freq = usrp->get_tx_rate(tx_channel);
  delt = 1.0 / samp_freq;
  initEpochPlan(&epoch_plan, samp_freq);
  iq_buff_size = epoch_plan.max_samples;

  fprintf(stderr,
          "[TIMING] Generator sample rate locked to actual TX rate: "
          "%.6f Hz\n",
          samp_freq);
  fprintf(stderr, "[TIMING] GPS time scale: %.9f (%+.6f ppm)\n",
          getGpsTimeScale(gps_time_ppm), gps_time_ppm);
  if (epoch_plan.max_samples == epoch_plan.base_samples) {
    fprintf(stderr, "[TIMING] Epoch sample count: %d samples every %.1f ms\n",
            epoch_plan.base_samples, EPOCH_TARGET_SEC * 1000.0);
  } else {
    fprintf(stderr,
            "[TIMING] Epoch sample count dithers between %d and %d samples "
            "to match the actual TX rate.\n",
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
    if (tfd < 0)
      return 1;
    fprintf(stderr, "[TRIMBLE] Connected.\n");

    int tag_yy, tag_mm, tag_dd, tag_hh, tag_min, tag_sec;

    int got_tag = trimbleReadTag(tfd, trimble_timeout_ms,
                                 &tag_yy, &tag_mm, &tag_dd,
                                 &tag_hh, &tag_min, &tag_sec);
    if (got_tag)
      trimble_tag_mono = getMonotonicSeconds();

    close(tfd);

    if (!got_tag) {
      fprintf(stderr,
              "[TRIMBLE] ERROR: No valid time tag received within timeout.\n");
      return 1;
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

    // target UTC = tagged UTC + start_offset
    // GPS time  = target UTC + leap_sec
    trimbleUtcToGpsEpoch(tag_yy, tag_mm, tag_dd, tag_hh, tag_min, tag_sec,
                         trimble_start_offset, trimble_leap_sec, &t0, &g0);

    // Apply calibration term
    if (trimble_tx_cal_ns != 0) {
      double cal_sec = (double)trimble_tx_cal_ns * 1.0e-9;
      g0 = incGpsTimePrecise(g0, cal_sec);
      gps2date(&g0, &t0);
      fprintf(stderr, "[TRIMBLE] Applied calibration: %+.3f us\n",
              cal_sec * 1.0e6);
    }

    fprintf(stderr, "[TRIMBLE] Target GPS epoch: week %d  tow %.9f\n", g0.week,
            g0.sec);
    fprintf(stderr,
            "[TRIMBLE] Target datetime:  %04d/%02d/%02d,%02d:%02d:%09.6f\n",
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
    if (trimble_rtcm_mode == TRUE) {
      ionoutc.wnt = gmin.week;
      ionoutc.tot = (int)gmin.sec;
    } else if (timeoverwrite == TRUE) {
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
        return 1;
      }
    }
  } else {
    g0 = gmin;
    t0 = tmin;
  }

  fprintf(stderr, "Start time = %4d/%02d/%02d,%02d:%02d:%09.6f (%d:%.9f)\n",
          t0.y, t0.m, t0.d, t0.hh, t0.mm, t0.sec, g0.week, g0.sec);
  if (stream_forever)
    fprintf(stderr, "Duration = streaming until interrupted\n");
  else
    fprintf(stderr, "Duration = %.1f [sec]\n", (double)numd / 10.0);

  ////////////////////////////////////////////////////////////
  // Select ephemeris set
  ////////////////////////////////////////////////////////////

  if (trimble_rtcm_mode == TRUE) {
    ieph = 0;
  } else {
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
      return 1;
    }
  }

  ////////////////////////////////////////////////////////////
  // Build synthetic satellite overlay
  ////////////////////////////////////////////////////////////

  if (synth_cfg.enabled) {
    gpstime_t synth_ref = quantizeSynthReferenceTime(g0);
    const ephem_t(*synth_source)[MAX_SAT] =
        has_revive_mode == TRUE ? revive_scan_eph : eph;

    for (sv = 0; sv < MAX_SAT; sv++) {
      if (synth_cfg.mode[sv] == SYNTH_REVIVE) {
        ephem_t revive_template;
        gpstime_t template_toe;
        double delta_sec;
        double elev_deg;
        int found_ephem = FALSE;

        if (scanEphemerisForRevive(synth_source, neph, sv + 1, synth_ref,
                                   xyz[0], &revive_template, &template_toe,
                                   &delta_sec, &elev_deg,
                                   &found_ephem) == FALSE) {
          if (found_ephem == TRUE) {
            fprintf(stderr,
                    "ERROR: Revive PRN %d: not above %.1f deg at any point "
                    "in lookback window.\n",
                    sv + 1, SYNTH_REVIVE_MIN_ELEVATION_DEG);
          } else {
            fprintf(stderr,
                    "ERROR: Revive PRN %d: no ephemeris found within %.1fh "
                    "lookback.\n",
                    sv + 1, SYNTH_REVIVE_MAX_LOOKBACK_SEC / 3600.0);
          }
          return 1;
        }
      } else if (synth_cfg.mode[sv] == SYNTH_OVERHEAD ||
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

    refreshSyntheticEphemerisSet(&synth_eph, synth_source, neph, eph[ieph],
                                 &ionoutc, &synth_cfg, xyz[0], synth_ref);
  }

  overlaySyntheticEphemerisSet(active_eph, eph[ieph], &synth_cfg, &synth_eph);

  ////////////////////////////////////////////////////////////
  // Allocate IQ buffer and channels
  ////////////////////////////////////////////////////////////

  iq_buff = (short *)calloc(2 * iq_buff_size, sizeof(short));
  if (!iq_buff) {
    fprintf(stderr, "ERROR: Failed to allocate IQ buffer.\n");
    return 1;
  }

  for (i = 0; i < MAX_CHAN; i++)
    chan[i].prn = 0;
  for (sv = 0; sv < MAX_SAT; sv++)
    allocatedSat[sv] = -1;

  grx = g0;
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

  first_sample_gps_time = g0;
  generated_samples = 0;
  emitted_samples = 0;

  // Generate pre-buffer epochs
  for (int pb = 0; pb < prebuffer_count && !stop_requested; pb++) {
    int sample_count = nextEpochSampleCount(&epoch_plan);
    gpstime_t block_start_gps_time =
        getGpsTimeAtSampleOffset(first_sample_gps_time, generated_samples,
                                 samp_freq, gps_time_ppm);
    gpstime_t block_end_gps_time =
        getGpsTimeAtSampleOffset(first_sample_gps_time,
                                 generated_samples + sample_count, samp_freq,
                                 gps_time_ppm);

    current_epoch_duration =
        subGpsTime(block_end_gps_time, block_start_gps_time);
    grx = block_end_gps_time;

    // generateEpoch() expects the GPS time at the end of the block.
    if (generated_samples == 0) {
      fprintf(stderr,
              "[TIMING] First RF sample GPS epoch: week %d tow %.9f\n",
              first_sample_gps_time.week, first_sample_gps_time.sec);
      fprintf(stderr,
              "[TIMING] First generated epoch end: week %d tow %.9f\n",
              block_end_gps_time.week, block_end_gps_time.sec);
      fprintf(stderr, "[TIMING] TX sample clock: %.6f Hz\n", samp_freq);
    }

    ring_sample_counts[ring_write] = sample_count;
    generateEpoch(ring[ring_write], sample_count, chan, gain, active_eph,
                  &ionoutc, grx, staticLocationMode, current_epoch_duration,
                  delt, path_loss_enable, fixed_gain, ant_pat, attack_enabled,
                  &attack_cfg, attack_noise_state, jam_js_linear);
    generated_samples += sample_count;

    ring_write = (ring_write + 1) % ring_size;
    ring_count++;

    // 30-second nav/channel refresh
    igrx = (int)(grx.sec * 10.0 + 0.5);
    if (igrx % (int)(SYNTH_EPHEM_REFRESH_SEC * 10.0 + 0.5) == 0)
      refreshNavState(chan, eph, has_revive_mode == TRUE ? revive_scan_eph : eph,
                      neph, &ieph, active_eph, &synth_eph, &synth_cfg,
                      &ionoutc, grx, elvmask, trimble_rtcm_mode,
                      &trimble_rtcm_alive, &trimble_rtcm_stream, &attack_cfg);

    iumd = pb + 2; // Preserve the existing finite-duration loop convention.
  }

  fprintf(stderr, "[TX] Pre-buffer complete.\n");

  ////////////////////////////////////////////////////////////
  // Schedule timed TX start
  ////////////////////////////////////////////////////////////

  double tx_start_delay_sec = (double)tx_advance_ns * 1.0e-9;
  double first_send_timeout = 3.0;

  if (trimble_mode && trimble_tag_mono >= 0.0) {
    ////////////////////////////////////////////////////////////////
    // Trimble mode: compute remaining delay from tag receipt
    ////////////////////////////////////////////////////////////////
    double tag_lead_sec = (double)trimble_tag_lead_ms * 1.0e-3;
    double planned_delay = tag_lead_sec + (double)trimble_start_offset;
    double prep_elapsed = getMonotonicSeconds() - trimble_tag_mono;
    double remaining = planned_delay - prep_elapsed;

    fprintf(stderr, "[TRIMBLE] Tag lead estimate:     %.3f s\n", tag_lead_sec);
    fprintf(stderr, "[TRIMBLE] Planned delay (lead + offset): %.3f s\n",
            planned_delay);
    fprintf(stderr, "[TRIMBLE] Prep elapsed since tag: %.3f ms\n",
            prep_elapsed * 1000.0);
    fprintf(stderr, "[TRIMBLE] Remaining delay:        %.3f ms\n",
            remaining * 1000.0);

    if (remaining < TX_START_LEAD_MIN_SEC) {
      fprintf(stderr,
              "[TRIMBLE] ERROR: Target second is stale (remaining %.3f ms "
              "< minimum %.3f ms).\n"
              "  Increase --trimble-start-offset-sec or reduce --prebuffer.\n",
              remaining * 1000.0, TX_START_LEAD_MIN_SEC * 1000.0);
      // Cleanup ring buffer before exit
      for (i = 0; i < ring_size; i++)
        free(ring[i]);
      free(ring);
      free(ring_sample_counts);
      free(iq_buff);
      return 1;
    }

    tx_start_delay_sec = remaining;
    first_send_timeout = remaining + 1.0;
    fprintf(stderr, "[TRIMBLE] Accepted: scheduling TX at +%.3f ms\n",
            tx_start_delay_sec * 1000.0);

  } else if (wall_clock_latch_sec >= 0.0) {
    double prep_elapsed_sec =
        getWallClockRealtimeSeconds() - wall_clock_latch_sec;

    fprintf(stderr, "[TIMING] Startup prep after wall-clock latch: %.3f ms\n",
            prep_elapsed_sec * 1000.0);

    tx_start_delay_sec -= prep_elapsed_sec;
    if (tx_start_delay_sec < TX_START_LEAD_MIN_SEC) {
      double overrun_sec = prep_elapsed_sec - (double)tx_advance_ns * 1.0e-9;
      fprintf(stderr,
              "[TIMING] WARNING: startup prep exceeded requested TX lead by "
              "%.3f ms; scheduling with minimum %.3f ms margin. Increase "
              "--tx-advance-ns or -r.\n",
              overrun_sec > 0.0 ? overrun_sec * 1000.0 : 0.0,
              TX_START_LEAD_MIN_SEC * 1000.0);
      tx_start_delay_sec = TX_START_LEAD_MIN_SEC;
    }
  }

  // Reset device time to zero for clean scheduling
  usrp->set_time_now(uhd::time_spec_t(0.0));

  uhd::time_spec_t tx_start_time(tx_start_delay_sec);

  fprintf(stderr, "[TX] Scheduling first TX at hardware time +%.3f ms\n",
          tx_start_delay_sec * 1000.0);

  // Send first pre-buffered epoch with timed start
  uhd::tx_metadata_t md;
  md.start_of_burst = true;
  md.end_of_burst = false;
  md.has_time_spec = true;
  md.time_spec = tx_start_time;

  // Send in chunks respecting max_samps
  {
    short *buf = ring[ring_read];
    size_t total = (size_t)ring_sample_counts[ring_read];
    size_t sent = 0;

    while (sent < total) {
      size_t chunk = std::min(max_samps, total - sent);
      size_t n = tx_stream->send(&buf[sent * 2], chunk, md, first_send_timeout);
      sent += n;

      // Only first chunk has time spec
      md.has_time_spec = false;
      md.start_of_burst = false;
    }
    emitted_samples += (long long)total;
  }
  ring_read = (ring_read + 1) % ring_size;
  ring_count--;

  fprintf(stderr, "[TX] First epoch sent (timed). Streaming ...\n\n");

  ////////////////////////////////////////////////////////////
  // Main streaming loop
  ////////////////////////////////////////////////////////////

  md.start_of_burst = false;
  md.end_of_burst = false;
  md.has_time_spec = false;

  long long underflow_count = 0;

  while (!stop_requested && (stream_forever == TRUE || iumd < numd)) {
    int sample_count = nextEpochSampleCount(&epoch_plan);
    gpstime_t block_start_gps_time =
        getGpsTimeAtSampleOffset(first_sample_gps_time, generated_samples,
                                 samp_freq, gps_time_ppm);
    gpstime_t block_end_gps_time =
        getGpsTimeAtSampleOffset(first_sample_gps_time,
                                 generated_samples + sample_count, samp_freq,
                                 gps_time_ppm);

    current_epoch_duration =
        subGpsTime(block_end_gps_time, block_start_gps_time);
    grx = block_end_gps_time;
    ring_sample_counts[ring_write] = sample_count;

    // Generate next epoch into ring buffer
    generateEpoch(ring[ring_write], sample_count, chan, gain, active_eph,
                  &ionoutc, grx, staticLocationMode, current_epoch_duration,
                  delt, path_loss_enable, fixed_gain, ant_pat, attack_enabled,
                  &attack_cfg, attack_noise_state, jam_js_linear);
    generated_samples += sample_count;
    ring_write = (ring_write + 1) % ring_size;
    ring_count++;

    // Send oldest buffered epoch
    if (ring_count > 0) {
      short *buf = ring[ring_read];
      size_t total = (size_t)ring_sample_counts[ring_read];
      size_t sent = 0;

      while (sent < total) {
        size_t chunk = std::min(max_samps, total - sent);
        size_t n = tx_stream->send(&buf[sent * 2], chunk, md, 3.0);
        sent += n;
      }
      emitted_samples += (long long)total;
      ring_read = (ring_read + 1) % ring_size;
      ring_count--;
    }

    // Check for async messages (underflow and time-error detection)
    uhd::async_metadata_t async_md;
    if (tx_stream->recv_async_msg(async_md, 0.0)) {
      if (async_md.event_code == uhd::async_metadata_t::EVENT_CODE_UNDERFLOW) {
        underflow_count++;
        if (verb)
          fprintf(stderr, "\n[UHD] WARNING: TX underflow #%lld at t=%.1f\n",
                  underflow_count,
                  getGpsElapsedFromSamples(emitted_samples, samp_freq,
                                           gps_time_ppm));
      } else if (async_md.event_code ==
                 uhd::async_metadata_t::EVENT_CODE_TIME_ERROR) {
        fprintf(stderr, "\n[UHD] FATAL: Timed TX start failed "
                        "(EVENT_CODE_TIME_ERROR). The scheduled start time was "
                        "missed. Aborting.\n");
        stop_requested = 1;
      }
    }

    // 30-second nav/channel refresh
    igrx = (int)(grx.sec * 10.0 + 0.5);
    if (igrx % (int)(SYNTH_EPHEM_REFRESH_SEC * 10.0 + 0.5) == 0) {
      refreshNavState(chan, eph, has_revive_mode == TRUE ? revive_scan_eph : eph,
                      neph, &ieph, active_eph, &synth_eph, &synth_cfg,
                      &ionoutc, grx, elvmask, trimble_rtcm_mode,
                      &trimble_rtcm_alive, &trimble_rtcm_stream, &attack_cfg);

      if (verb) {
        fprintf(stderr, "\n");
        for (i = 0; i < MAX_CHAN; i++)
          if (chan[i].prn > 0)
            fprintf(stderr, "%02d %6.1f %5.1f %11.1f %5.1f\n", chan[i].prn,
                    chan[i].azel[0] * R2D, chan[i].azel[1] * R2D,
                    chan[i].rho0.d, chan[i].rho0.iono_delay);
      }
    }

    iumd++;

    fprintf(stderr, "\rTime into run = %7.3f",
            getGpsElapsedFromSamples(emitted_samples, samp_freq,
                                     gps_time_ppm));
    fflush(stderr);
  }

  ////////////////////////////////////////////////////////////
  // Shutdown
  ////////////////////////////////////////////////////////////

  fprintf(stderr, "\n\n[TX] Shutting down ...\n");

  // Send end-of-burst
  md.end_of_burst = true;
  tx_stream->send("", 0, md, 3.0);

  // Drain async messages
  {
    uhd::async_metadata_t async_md;
    while (tx_stream->recv_async_msg(async_md, 0.5)) {
      if (async_md.event_code == uhd::async_metadata_t::EVENT_CODE_UNDERFLOW)
        underflow_count++;
    }
  }

  fprintf(stderr, "[TX] Done. Total underflows: %lld\n", underflow_count);

  // Cleanup
  for (i = 0; i < ring_size; i++)
    free(ring[i]);
  free(ring);
  free(ring_sample_counts);
  free(iq_buff);
  rtcm3_nav_close(&trimble_rtcm_stream);

  return 0;
}
