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

#include <uhd/device.hpp>
#include <uhd/stream.hpp>
#include <uhd/types/metadata.hpp>
#include <uhd/usrp/multi_usrp.hpp>

#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <unistd.h>
#include <utility>
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
#include "tools/matched_code_plan.h"
#include "tools/matched_code_source.h"
#include "tools/sha256.h"
}

#include "player/matched_code_alignment.h"

////////////////////////////////////////////////////////////
// Constants
////////////////////////////////////////////////////////////

#define TX_FREQUENCY 1575420000.0 // GPS L1 (Hz)
#define TX_SAMPLERATE 2500000.0   // 2.5 Msps, exact from X300 200 MHz clock
#define PREBUFFER_DEFAULT 5       // 0.5 s pre-buffer
#define TX_START_LEAD_DEFAULT_NS 250000000LL // 250 ms startup lead
#define TX_START_LEAD_MAX_SEC 60.0
#define TX_START_LEAD_MIN_SEC 0.02
#define EPOCH_TARGET_SEC 0.1
#define GPS_TIME_PPM_DEFAULT 0.0
#define GPS_TIME_PPM_MAX_ABS 1000.0
#define MATCHED_RATE_TOLERANCE_HZ 0.5
#define MATCHED_DRY_RUN_VALIDATION_SEC 0.1
#define MATCHED_MIN_HEADROOM_DB 1.0
#define FNV1A64_OFFSET_BASIS UINT64_C(14695981039346656037)
#define FNV1A64_PRIME UINT64_C(1099511628211)

struct MatchedCodeOptions {
  bool enabled = false;
  bool dry_run = false;
  bool controlled_rf_confirmed = false;
  bool amplitude_set = false;
  bool phase_seed_set = false;
  bool address_set = false;
  bool channel_set = false;
  bool antenna_set = false;
  bool legacy_js_set = false;
  bool legacy_gain_boost_set = false;
  std::string target_prns;
  std::string manifest_path;
  std::string trajectory_path;
  std::string calibration_id;
  double amplitude = 0.0;
  uint64_t phase_seed = 0;
};

struct MatchedCodeRunResult {
  std::string status = "incomplete";
  std::string failure_reason;
  std::string device_type;
  std::string device_product;
  std::string device_serial;
  std::string device_address;
  std::string actual_antenna;
  std::string start_mode;
  size_t tx_channel_count = 0;
  double actual_rate_hz = 0.0;
  double actual_frequency_hz = 0.0;
  double actual_gain_db = 0.0;
  double uhd_start_time_seconds = 0.0;
  bool start_margin_met = false;
  bool target_allocation_passed = false;
  uint64_t internal_alignment_samples = 0;
  uint64_t rendered_jammer_samples = 0;
  uint64_t quantized_samples = 0;
  uint64_t sent_samples = 0;
  uint64_t underflows = 0;
  uint64_t sequence_errors = 0;
  uint64_t time_errors = 0;
  bool interrupted = false;
  matched_code_source_metrics_t source_metrics{};
  uint64_t jammer_iq_fnv1a64 = FNV1A64_OFFSET_BASIS;
  std::string trajectory_sha256;
  int exit_status = 1;
};

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
    if (buf[i] == 'U' && buf[i + 1] == 'T' && buf[i + 2] == 'C' &&
        buf[i + 3] == ' ')
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

    // Scan for "UTC " in the accumulated buffer
    const char *utc = findUtcMarker(buf, pos);
    if (utc) {
      int remaining_bytes = pos - (int)(utc - buf);

      // Need at least "UTC YY.MM.DD HH:MM:SS" = 23 chars
      if (remaining_bytes >= 23) {
        // Copy into a NUL-terminated temp for sscanf
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

static void prepareEpoch(channel_t chan[MAX_CHAN], int gain[MAX_CHAN],
                         ephem_t *active_eph, ionoutc_t *ionoutc, gpstime_t grx,
                         double epoch_duration, double delt,
                         int path_loss_enable, int fixed_gain,
                         double ant_pat[37], int attack_enabled,
                         const attack_config_t *attack_cfg) {
  int i;

  for (i = 0; i < MAX_CHAN; i++) {
    if (chan[i].prn > 0) {
      range_t rho;
      int sv = chan[i].prn - 1;
      double path_loss;
      double ant_gain;
      int ibs;

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
      gain[i] = path_loss_enable == TRUE ? (int)(path_loss * ant_gain * 128.0)
                                         : fixed_gain;
      if (attack_enabled == TRUE)
        applyGainAttack(attack_cfg, chan[i].prn, &gain[i]);
      if (attack_cfg != nullptr && attack_cfg->partial_mode &&
          attack_cfg->gain_boost_db != 0.0)
        gain[i] = (int)(gain[i] * pow(10.0, attack_cfg->gain_boost_db / 20.0));
    }
  }
}

static int cleanCarrierTableIndex(const channel_t *channel) {
#ifdef FLOAT_CARR_PHASE
  return (int)floor(channel->carr_phase * 512.0);
#else
  return (channel->carr_phase >> 16) & 0x1ff;
#endif
}

static void advanceCleanChannelSample(channel_t *channel, double delt) {
  channel->code_phase += channel->f_code * delt;
  if (channel->code_phase >= CA_SEQ_LEN) {
    channel->code_phase -= CA_SEQ_LEN;
    ++channel->icode;
    if (channel->icode >= 20) {
      channel->icode = 0;
      ++channel->ibit;
      if (channel->ibit >= 30) {
        channel->ibit = 0;
        ++channel->iword;
      }
      channel->dataBit =
          (int)((channel->dwrd[channel->iword] >> (29 - channel->ibit)) &
                0x1UL) *
              2 -
          1;
    }
  }
  channel->codeCA = channel->ca[(int)channel->code_phase] * 2 - 1;
#ifdef FLOAT_CARR_PHASE
  channel->carr_phase += channel->f_carr * delt;
  if (channel->carr_phase >= 1.0)
    channel->carr_phase -= 1.0;
  else if (channel->carr_phase < 0.0)
    channel->carr_phase += 1.0;
#else
  channel->carr_phase += channel->carr_phasestep;
#endif
}

static void
generateEpoch(short *iq_buff, int sample_count, channel_t chan[MAX_CHAN],
              int gain[MAX_CHAN], ephem_t *active_eph, ionoutc_t *ionoutc,
              gpstime_t grx, int staticLocationMode, double epoch_duration,
              double delt, int path_loss_enable, int fixed_gain,
              double ant_pat[37], int attack_enabled,
              const attack_config_t *attack_cfg,
              unsigned int attack_noise_state[MAX_SAT], double jam_js_linear) {
  int i, isamp;
  int ip, qp, iTable;

  prepareEpoch(chan, gain, active_eph, ionoutc, grx, epoch_duration, delt,
               path_loss_enable, fixed_gain, ant_pat, attack_enabled,
               attack_cfg);

  // Per-sample: accumulate IQ from all channels
  for (isamp = 0; isamp < sample_count; isamp++) {
    int i_acc = 0;
    int q_acc = 0;

    for (i = 0; i < MAX_CHAN; i++) {
      if (chan[i].prn > 0) {
        attack_method_t attack_method = ATTACK_METHOD_NONE;
        if (attack_enabled == TRUE)
          attack_method = getAttackMethod(attack_cfg, chan[i].prn);

        iTable = cleanCarrierTableIndex(&chan[i]);
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

        advanceCleanChannelSample(&chan[i], delt);
      }
    }

    i_acc = (i_acc + 64) >> 7;
    q_acc = (q_acc + 64) >> 7;

    iq_buff[isamp * 2] = clipInt16(i_acc);
    iq_buff[isamp * 2 + 1] = clipInt16(q_acc);
  }
}

static void renderCleanEpochWide(double *iq_buff, int sample_count,
                                 channel_t chan[MAX_CHAN], int gain[MAX_CHAN],
                                 double delt) {
  int isamp;

  for (isamp = 0; isamp < sample_count; ++isamp) {
    int i_acc = 0;
    int q_acc = 0;
    int i;

    for (i = 0; i < MAX_CHAN; ++i) {
      int i_table;
      int ip;
      int qp;

      if (chan[i].prn <= 0)
        continue;
      i_table = cleanCarrierTableIndex(&chan[i]);
      ip = chan[i].dataBit * chan[i].codeCA * cosTable512[i_table] * gain[i];
      qp = chan[i].dataBit * chan[i].codeCA * sinTable512[i_table] * gain[i];
      i_acc += ip;
      q_acc += qp;

      advanceCleanChannelSample(&chan[i], delt);
    }

    iq_buff[2 * isamp] = (double)i_acc / 128.0;
    iq_buff[2 * isamp + 1] = (double)q_acc / 128.0;
  }
}

static bool captureMatchedTargetStates(
    const matched_code_plan_t &plan, const channel_t chan[MAX_CHAN],
    const int gain[MAX_CHAN], uint64_t sample_offset,
    matched_code_target_state_t states[MATCHED_CODE_MAX_TARGETS],
    std::string *error) {
  for (size_t index = 0; index < plan.target_count; ++index) {
    int prn = plan.target_prns[index];
    int channel = allocatedSat[prn - 1];

    if (channel < 0 || channel >= MAX_CHAN || chan[channel].prn != prn ||
        gain[channel] <= 0) {
      std::ostringstream message;
      message << "target PRN " << prn
              << " is not allocated with positive clean gain at sample "
              << sample_offset;
      *error = message.str();
      return false;
    }
    matched_code_capture_channel_state(&chan[channel], gain[channel],
                                       sample_offset, &states[index]);
  }
  return true;
}

static bool validateMatchedTargetUsability(
    const matched_code_plan_t &plan, const ephem_t active_ephemeris[MAX_SAT],
    const synth_config_t *synth_config, gpstime_t boundary_time,
    double elevation_mask, uint64_t sample_offset, std::string *error) {
  for (size_t index = 0; index < plan.target_count; ++index) {
    int prn = plan.target_prns[index];
    int satellite = prn - 1;
    bool synthetic = synth_config != nullptr && synth_config->enabled &&
                     synth_config->mode[satellite] != SYNTH_NONE;
    double azimuth_elevation[2];
    int usable =
        synthetic
            ? (active_ephemeris[satellite].vflg == 1 ? TRUE : FALSE)
            : checkSatVisibility(active_ephemeris[satellite], boundary_time,
                                 xyz[0], elevation_mask, azimuth_elevation);

    if (usable != TRUE) {
      std::ostringstream message;
      message << "target PRN " << prn
              << " is not geometrically usable at sample " << sample_offset;
      *error = message.str();
      return false;
    }
  }
  return true;
}

static void writeTrajectoryHeader(std::ofstream &trajectory,
                                  const matched_code_plan_t &plan,
                                  gpstime_t start_time) {
  trajectory << std::setprecision(17);
  trajectory << "# schema=gps-sdr-sim.target-trajectory.v1\n";
  trajectory << "# sample_rate_hz=" << plan.sample_rate_hz << "\n";
  trajectory << "# epoch_cadence_samples="
             << static_cast<uint64_t>(
                    std::llround(plan.sample_rate_hz * EPOCH_TARGET_SEC))
             << "\n";
  trajectory << "# gps_week=" << start_time.week << "\n";
  trajectory << "# gps_tow=" << start_time.sec << "\n";
  trajectory << "# boundary=first_sample\n";
  trajectory << "sample_offset,prn,code_phase_chips,carrier_doppler_hz,"
                "code_rate_chips_per_s,clean_gain\n";
}

static void writeTrajectoryStates(std::ofstream &trajectory,
                                  const matched_code_target_state_t *states,
                                  size_t state_count) {
  for (size_t index = 0; index < state_count; ++index) {
    const matched_code_target_state_t &state = states[index];
    trajectory << state.sample_offset << ',' << state.prn << ','
               << state.code_phase_chips << ',' << state.carrier_doppler_hz
               << ',' << state.code_rate_chips_per_s << ',' << state.clean_gain
               << '\n';
  }
}

static void
refreshNavState(channel_t chan[MAX_CHAN], ephem_t eph[][MAX_SAT],
                const ephem_t synth_source[][MAX_SAT], int neph, int *ieph,
                ephem_t *active_eph, synth_ephem_store_t *synth_eph,
                const synth_config_t *synth_cfg, const ionoutc_t *ionoutc,
                gpstime_t grx, double elvmask, int trimble_rtcm_mode,
                int *trimble_rtcm_alive,
                rtcm3_nav_stream_t *trimble_rtcm_stream,
                const attack_config_t *attack_cfg, const int *required_prns) {
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
                  synth_cfg, required_prns);
}

static std::string jsonEscape(const std::string &value) {
  std::ostringstream escaped;

  for (unsigned char character : value) {
    switch (character) {
    case '"':
      escaped << "\\\"";
      break;
    case '\\':
      escaped << "\\\\";
      break;
    case '\b':
      escaped << "\\b";
      break;
    case '\f':
      escaped << "\\f";
      break;
    case '\n':
      escaped << "\\n";
      break;
    case '\r':
      escaped << "\\r";
      break;
    case '\t':
      escaped << "\\t";
      break;
    default:
      if (character < 0x20) {
        escaped << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                << static_cast<int>(character) << std::dec;
      } else {
        escaped << static_cast<char>(character);
      }
    }
  }
  return escaped.str();
}

static uint64_t updateFnv1a64Sc16(uint64_t hash, const int16_t *samples,
                                  size_t sample_count) {
  for (size_t index = 0; index < sample_count * 2; ++index) {
    uint16_t value = static_cast<uint16_t>(samples[index]);
    hash ^= static_cast<uint8_t>(value & 0xffU);
    hash *= FNV1A64_PRIME;
    hash ^= static_cast<uint8_t>((value >> 8) & 0xffU);
    hash *= FNV1A64_PRIME;
  }
  return hash;
}

static std::string fnv1a64Hex(uint64_t value) {
  std::ostringstream output;
  output << std::hex << std::setfill('0') << std::setw(16) << value;
  return output.str();
}

static bool initializeMatchedJammerPlan(matched_code_plan_t *plan,
                                        double sample_rate_hz,
                                        const std::string &target_prns,
                                        double amplitude, uint64_t phase_seed,
                                        char *error, size_t error_size) {
  long double validation_samples_exact;
  long double validation_samples_rounded;

  if (plan == nullptr || !std::isfinite(sample_rate_hz) ||
      sample_rate_hz <= 0.0 || !std::isfinite(amplitude) || amplitude <= 0.0 ||
      amplitude > 1.0) {
    snprintf(error, error_size,
             "sample rate and matched-code amplitude in (0, 1] are required");
    return false;
  }

  memset(plan, 0, sizeof(*plan));
  plan->sample_rate_hz = sample_rate_hz;
  plan->phase_seed = phase_seed;
  plan->jammer_scale = amplitude;
  if (matched_code_parse_targets(target_prns.c_str(), plan->target_prns,
                                 &plan->target_count, error,
                                 error_size) != 0)
    return false;

  validation_samples_exact =
      static_cast<long double>(sample_rate_hz) *
      static_cast<long double>(MATCHED_DRY_RUN_VALIDATION_SEC);
  validation_samples_rounded = roundl(validation_samples_exact);
  if (validation_samples_rounded <= 0.0L ||
      validation_samples_rounded > static_cast<long double>(UINT64_MAX) ||
      fabsl(validation_samples_exact - validation_samples_rounded) > 1.0e-6L) {
    snprintf(error, error_size,
             "the 100 ms validation window is not an integral sample count");
    return false;
  }

  plan->total_samples = static_cast<uint64_t>(validation_samples_rounded);
  plan->onset_sample = 0;
  plan->offset_sample = plan->total_samples;
  plan->ramp_samples = 0;
  plan->reference_start_sample = 0;
  plan->reference_end_sample = plan->total_samples;
  plan->jammer_component_bound =
      amplitude * sqrt(static_cast<double>(plan->target_count));
  plan->predicted_composite_bound = plan->jammer_component_bound;
  plan->predicted_headroom_db =
      20.0 * log10(1.0 / plan->predicted_composite_bound);
  if (plan->predicted_headroom_db + 1.0e-12 < MATCHED_MIN_HEADROOM_DB) {
    snprintf(error, error_size,
             "matched-code amplitude leaves %.3f dB predicted headroom; at "
             "least %.1f dB is required for %zu target(s)",
             plan->predicted_headroom_db, MATCHED_MIN_HEADROOM_DB,
             plan->target_count);
    return false;
  }
  return true;
}

static bool writeMatchedManifestAtomic(
    const MatchedCodeOptions &options, const matched_code_plan_t &plan,
    const MatchedCodeRunResult &result, const char *navfile,
    const std::string &ephemeris_sha256, const std::string &scenario_sha256,
    gpstime_t sample_zero, const double reference_xyz[3],
    double requested_rate_hz, const char *device_address, size_t tx_channel,
    const char *tx_antenna, double requested_gain_db, const char *clock_source,
    const char *time_source, int prebuffer_count, long long tx_delay_cal_ns,
    double gps_time_ppm, bool trimble_mode) {
  std::string temporary_path = options.manifest_path + ".tmp";
  std::ofstream manifest(temporary_path, std::ios::out | std::ios::trunc);
  matched_code_source_config_t source_config{};
  matched_code_source_t phase_source;
  char source_error[128];

  if (!manifest)
    return false;
  source_config.sample_rate_hz = plan.sample_rate_hz;
  source_config.total_samples = plan.total_samples;
  source_config.onset_sample = plan.onset_sample;
  source_config.offset_sample = plan.offset_sample;
  source_config.ramp_samples = plan.ramp_samples;
  source_config.amplitude = plan.jammer_scale;
  source_config.phase_seed = plan.phase_seed;
  source_config.target_count = plan.target_count;
  for (size_t index = 0; index < plan.target_count; ++index)
    source_config.target_prns[index] = plan.target_prns[index];
  bool phases_available =
      matched_code_source_init(&phase_source, &source_config, source_error,
                               sizeof(source_error)) == 0;

  manifest << std::setprecision(17);
  manifest << "{\n";
  manifest << "  \"schema\": \"gps-sdr-sim.x300tx-matched-code.v2\",\n";
  manifest << "  \"tool\": \"x300tx\",\n";
  manifest << "  \"status\": \"" << jsonEscape(result.status) << "\",\n";
  manifest << "  \"failure_reason\": ";
  if (result.failure_reason.empty())
    manifest << "null";
  else
    manifest << "\"" << jsonEscape(result.failure_reason) << "\"";
  manifest << ",\n";
  manifest << "  \"exit_status\": " << result.exit_status << ",\n";
  manifest << "  \"acceptance_scope\": "
              "\"transmitter-only; RF waveform fidelity, propagation, "
              "receiver-input power, RF code alignment, and receiver behavior "
              "were not measured\",\n";
  manifest << "  \"rf_output\": {\"contains\": "
              "\"matched_code_interference_only\", "
              "\"clean_gps_transmitted\": false},\n";
  manifest << "  \"safety\": {\"controlled_rf_only\": true, "
              "\"controlled_rf_confirmed\": "
           << (options.controlled_rf_confirmed ? "true" : "false")
           << ", \"calibration_id\": ";
  if (options.calibration_id.empty())
    manifest << "null";
  else
    manifest << "\"" << jsonEscape(options.calibration_id) << "\"";
  manifest << "},\n";
  manifest << "  \"scenario\": {\"schema\": "
              "\"gps-sdr-sim.x300tx-frozen-scenario.v1\", "
              "\"sha256\": \""
           << jsonEscape(scenario_sha256) << "\", \"ephemeris_path\": \""
           << jsonEscape(navfile != nullptr ? navfile : "")
           << "\", \"ephemeris_sha256\": \"" << jsonEscape(ephemeris_sha256)
           << "\", \"reference_ecef_m\": [" << reference_xyz[0] << ", "
           << reference_xyz[1] << ", " << reference_xyz[2]
           << "], \"sample_zero_gps_week\": " << sample_zero.week
           << ", \"sample_zero_gps_tow\": " << sample_zero.sec
           << ", \"internal_alignment_source\": "
              "\"synthetic_clean_state_discarded_not_transmitted\""
           << ", \"requested_target_prns\": [";
  for (size_t index = 0; index < plan.target_count; ++index)
    manifest << (index == 0 ? "" : ", ") << plan.target_prns[index];
  manifest << "], \"selected_target_prns\": [";
  for (size_t index = 0; index < plan.target_count; ++index)
    manifest << (index == 0 ? "" : ", ") << plan.target_prns[index];
  manifest << "], \"startup_target_allocation_passed\": "
           << (result.target_allocation_passed ? "true" : "false") << "},\n";
  manifest << "  \"trajectory\": {\"schema\": "
              "\"gps-sdr-sim.target-trajectory.v1\", \"path\": \""
           << jsonEscape(options.trajectory_path) << "\", \"sha256\": \""
           << jsonEscape(result.trajectory_sha256) << "\"},\n";
  manifest << "  \"waveform\": {\"taxonomy\": "
              "\"navigation-data-free-gps-l1-ca-matched-code\", "
              "\"data_symbol_policy\": \"constant_positive\", "
              "\"carrier_phase_policy\": \"independent_deterministic\", "
              "\"phase_seed\": "
           << plan.phase_seed << ", \"equal_component_weight\": "
           << 1.0 / sqrt((double)plan.target_count)
           << ", \"target_count_normalization\": \"sqrt_n\", "
              "\"initial_carrier_phases_rad\": [";
  for (size_t index = 0; index < plan.target_count; ++index) {
    double phase =
        phases_available
            ? matched_code_source_initial_phase_rad(&phase_source, index)
            : 0.0;
    manifest << (index == 0 ? "" : ", ") << phase;
  }
  manifest << "], \"output_amplitude_full_scale\": " << options.amplitude
           << ", \"predicted_peak_full_scale\": "
           << plan.predicted_composite_bound
           << ", \"predicted_headroom_db\": " << plan.predicted_headroom_db
           << ", \"jammer_iq_fnv1a64\": ";
  if (result.quantized_samples == 0)
    manifest << "null";
  else
    manifest << "\"" << fnv1a64Hex(result.jammer_iq_fnv1a64) << "\"";
  manifest << "},\n";
  manifest << "  \"sample_contract\": {\"requested_rate_hz\": "
           << requested_rate_hz
           << ", \"actual_rate_hz\": " << result.actual_rate_hz
           << ", \"format\": \"sc16_le\", \"iq_order\": \"IQ\", "
              "\"continuous\": true, \"planned_samples\": null, "
              "\"dry_run_validation_samples\": "
           << plan.total_samples << ", \"internal_alignment_samples\": "
           << result.internal_alignment_samples
           << ", \"rendered_jammer_samples\": "
           << result.rendered_jammer_samples
           << ", \"quantized_jammer_samples\": " << result.quantized_samples
           << ", \"sent_jammer_samples\": " << result.sent_samples << "},\n";
  manifest << "  \"activation\": {\"start_sample\": 0, "
              "\"stop_condition\": \"SIGINT_or_SIGTERM\""
           << "},\n";
  manifest << "  \"timing\": {\"start_mode\": \""
           << jsonEscape(result.start_mode.empty()
                             ? (trimble_mode ? "trimble_time_tag"
                                             : "explicit_gps_time")
                             : result.start_mode)
           << "\", \"gps_week\": " << sample_zero.week
           << ", \"gps_tow\": " << sample_zero.sec
           << ", \"tx_delay_calibration_ns\": " << tx_delay_cal_ns
           << ", \"gps_time_ppm\": " << gps_time_ppm
           << ", \"prebuffer_epochs\": " << prebuffer_count
           << ", \"prebuffer_samples\": "
           << (uint64_t)prebuffer_count *
                  (uint64_t)llround(plan.sample_rate_hz * EPOCH_TARGET_SEC)
           << ", \"uhd_timed_start_s\": " << result.uhd_start_time_seconds
           << ", \"start_margin_met\": "
           << (result.start_margin_met ? "true" : "false") << "},\n";
  manifest << "  \"hardware\": {\"device_type\": \""
           << jsonEscape(result.device_type) << "\", \"product\": \""
           << jsonEscape(result.device_product) << "\", \"serial\": \""
           << jsonEscape(result.device_serial) << "\", \"address\": \""
           << jsonEscape(device_address)
           << "\", \"tx_channel_count\": " << result.tx_channel_count
           << ", \"selected_channel\": " << tx_channel
           << ", \"requested_antenna\": \"" << jsonEscape(tx_antenna)
           << "\", \"actual_antenna\": \"" << jsonEscape(result.actual_antenna)
           << "\", \"requested_center_frequency_hz\": " << TX_FREQUENCY
           << ", \"actual_center_frequency_hz\": " << result.actual_frequency_hz
           << ", \"clock_source\": \"" << jsonEscape(clock_source)
           << "\", \"time_source\": \"" << jsonEscape(time_source)
           << "\", \"requested_gain_db\": " << requested_gain_db
           << ", \"actual_gain_db\": " << result.actual_gain_db << "},\n";
  manifest << "  \"measurements\": {\"jammer_rms_full_scale\": "
           << result.source_metrics.active_plateau_rms
           << ", \"jammer_peak_full_scale\": "
           << result.source_metrics.peak_component
           << ", \"clipped_components\": "
           << result.source_metrics.clipped_components
           << ", \"underflows\": " << result.underflows
           << ", \"sequence_errors\": " << result.sequence_errors
           << ", \"time_errors\": " << result.time_errors
           << ", \"operator_stopped\": "
           << (result.interrupted ? "true" : "false")
           << "}\n";
  manifest << "}\n";
  manifest.close();
  if (!manifest ||
      std::rename(temporary_path.c_str(), options.manifest_path.c_str()) != 0) {
    std::remove(temporary_path.c_str());
    return false;
  }
  return true;
}

struct MatchedSimulationState {
  channel_t channels[MAX_CHAN];
  int gains[MAX_CHAN];
  ephem_t active_ephemeris[MAX_SAT];
  synth_ephem_store_t synthetic_ephemeris;
  int ephemeris_index = 0;
  epoch_plan_t epoch_plan{};
  uint64_t sample_offset = 0;
  gpstime_t receiver_time{};
};

static void initializeMatchedSimulationState(
    MatchedSimulationState *state, const channel_t channels[MAX_CHAN],
    const int gains[MAX_CHAN], const ephem_t active_ephemeris[MAX_SAT],
    const synth_ephem_store_t *synthetic_ephemeris, int ephemeris_index,
    const epoch_plan_t *epoch_plan, gpstime_t start_time) {
  memcpy(state->channels, channels, sizeof(state->channels));
  memcpy(state->gains, gains, sizeof(state->gains));
  memcpy(state->active_ephemeris, active_ephemeris,
         sizeof(state->active_ephemeris));
  state->synthetic_ephemeris = *synthetic_ephemeris;
  state->ephemeris_index = ephemeris_index;
  state->epoch_plan = *epoch_plan;
  state->sample_offset = 0;
  state->receiver_time = start_time;
}

static int nextMatchedFrameSampleCount(MatchedSimulationState *state,
                                       uint64_t total_samples) {
  uint64_t remaining = total_samples - state->sample_offset;
  int epoch_samples = nextEpochSampleCount(&state->epoch_plan);
  return remaining < (uint64_t)epoch_samples ? (int)remaining : epoch_samples;
}

static bool prepareMatchedFrame(
    MatchedSimulationState *state, const matched_code_plan_t &plan,
    uint64_t sample_limit,
    gpstime_t sample_zero, ionoutc_t *ionoutc, double gps_time_ppm, double delt,
    int path_loss_enable, int fixed_gain, double ant_pat[37],
    const synth_config_t *synth_config, double elevation_mask,
    matched_code_target_state_t states[MATCHED_CODE_MAX_TARGETS],
    int *sample_count, std::string *error) {
  gpstime_t block_start =
      getGpsTimeAtSampleOffset(sample_zero, (long long)state->sample_offset,
                               plan.sample_rate_hz, gps_time_ppm);
  gpstime_t block_end;

  *sample_count = nextMatchedFrameSampleCount(state, sample_limit);
  if (!validateMatchedTargetUsability(plan, state->active_ephemeris,
                                      synth_config, block_start, elevation_mask,
                                      state->sample_offset, error))
    return false;
  block_end = getGpsTimeAtSampleOffset(
      sample_zero, (long long)(state->sample_offset + *sample_count),
      plan.sample_rate_hz, gps_time_ppm);
  state->receiver_time = block_end;
  prepareEpoch(state->channels, state->gains, state->active_ephemeris, ionoutc,
               block_end, subGpsTime(block_end, block_start), delt,
               path_loss_enable, fixed_gain, ant_pat, FALSE, nullptr);
  return captureMatchedTargetStates(plan, state->channels, state->gains,
                                    state->sample_offset, states, error);
}

static void refreshMatchedStateIfNeeded(
    MatchedSimulationState *state, ephem_t eph[][MAX_SAT],
    const ephem_t synth_source[][MAX_SAT], int neph, ionoutc_t *ionoutc,
    const synth_config_t *synth_config, const attack_config_t *attack_config,
    double elevation_mask, const int *required_prns, double sample_rate_hz) {
  uint64_t refresh_samples =
      (uint64_t)llround(SYNTH_EPHEM_REFRESH_SEC * sample_rate_hz);

  if (refresh_samples == 0 || state->sample_offset == 0 ||
      state->sample_offset % refresh_samples != 0)
    return;
  int rtcm_alive = FALSE;
  refreshNavState(state->channels, eph, synth_source, neph,
                  &state->ephemeris_index, state->active_ephemeris,
                  &state->synthetic_ephemeris, synth_config, ionoutc,
                  state->receiver_time, elevation_mask, FALSE, &rtcm_alive,
                  nullptr, attack_config, required_prns);
}

static matched_code_source_config_t
matchedSourceConfig(const matched_code_plan_t &plan, uint64_t sample_limit) {
  matched_code_source_config_t config{};

  config.sample_rate_hz = plan.sample_rate_hz;
  config.total_samples = sample_limit;
  config.onset_sample = 0;
  config.offset_sample = sample_limit;
  config.ramp_samples = 0;
  config.amplitude = plan.jammer_scale;
  config.phase_seed = plan.phase_seed;
  config.target_count = plan.target_count;
  for (size_t index = 0; index < plan.target_count; ++index)
    config.target_prns[index] = plan.target_prns[index];
  return config;
}

static bool
renderMatchedFrame(MatchedSimulationState *state, matched_code_source_t *source,
                   const matched_code_plan_t &plan, uint64_t sample_limit,
                   gpstime_t sample_zero, ionoutc_t *ionoutc,
                   double gps_time_ppm, double delt, int path_loss_enable,
                   int fixed_gain, double ant_pat[37],
                   const synth_config_t *synth_config, double elevation_mask,
                   std::vector<double> *alignment_discard,
                   std::vector<int16_t> *jammer_output,
                   std::ofstream *trajectory, MatchedCodeRunResult *result,
                   int *sample_count, std::string *error) {
  matched_code_target_state_t states[MATCHED_CODE_MAX_TARGETS];
  char source_error[256];

  if (!prepareMatchedFrame(state, plan, sample_limit, sample_zero, ionoutc,
                           gps_time_ppm, delt, path_loss_enable, fixed_gain,
                           ant_pat, synth_config, elevation_mask, states,
                           sample_count, error))
    return false;
  if (trajectory != nullptr)
    writeTrajectoryStates(*trajectory, states, plan.target_count);
  if (matched_code_source_set_epoch(source, states, plan.target_count,
                                    source_error, sizeof(source_error)) != 0) {
    *error = source_error;
    return false;
  }
  alignment_discard->resize((size_t)*sample_count * 2);
  jammer_output->resize((size_t)*sample_count * 2);
  renderCleanEpochWide(alignment_discard->data(), *sample_count, state->channels,
                       state->gains, delt);
  if (matched_code_source_render_sc16(source, jammer_output->data(),
                                      (size_t)*sample_count) !=
      (size_t)*sample_count) {
    *error = "shared matched-code renderer stopped before the epoch ended";
    return false;
  }
  result->jammer_iq_fnv1a64 =
      updateFnv1a64Sc16(result->jammer_iq_fnv1a64, jammer_output->data(),
                        (size_t)*sample_count);
  result->internal_alignment_samples += (uint64_t)*sample_count;
  result->rendered_jammer_samples += (uint64_t)*sample_count;
  result->quantized_samples += (uint64_t)*sample_count;
  matched_code_source_get_metrics(source, &result->source_metrics);
  state->sample_offset += (uint64_t)*sample_count;
  return true;
}

static bool runMatchedPreflight(
    matched_code_plan_t *plan, MatchedCodeRunResult *result,
    const MatchedCodeOptions &options,
    const channel_t initial_channels[MAX_CHAN],
    const int initial_gains[MAX_CHAN],
    const ephem_t initial_active_ephemeris[MAX_SAT],
    const synth_ephem_store_t *initial_synthetic_ephemeris,
    int initial_ephemeris_index, const epoch_plan_t *initial_epoch_plan,
    ephem_t eph[][MAX_SAT], const ephem_t synth_source[][MAX_SAT], int neph,
    gpstime_t sample_zero, ionoutc_t *ionoutc,
    const synth_config_t *synth_config, const attack_config_t *attack_config,
    double elevation_mask, const int *required_prns, double gps_time_ppm,
    double delt, int path_loss_enable, int fixed_gain, double ant_pat[37],
    std::string *error) {
  MatchedSimulationState state;
  matched_code_target_state_t states[MATCHED_CODE_MAX_TARGETS];
  int saved_allocated[MAX_SAT];
  std::vector<double> alignment_discard;
  std::string trajectory_temporary = options.trajectory_path + ".tmp";
  std::ofstream trajectory(trajectory_temporary,
                           std::ios::out | std::ios::trunc);

  if (!trajectory) {
    *error = "cannot create target trajectory artifact";
    return false;
  }
  writeTrajectoryHeader(trajectory, *plan, sample_zero);
  memcpy(saved_allocated, allocatedSat, sizeof(saved_allocated));
  initializeMatchedSimulationState(
      &state, initial_channels, initial_gains, initial_active_ephemeris,
      initial_synthetic_ephemeris, initial_ephemeris_index, initial_epoch_plan,
      sample_zero);

  while (state.sample_offset < plan->total_samples) {
    int sample_count;
    if (!prepareMatchedFrame(&state, *plan, plan->total_samples, sample_zero,
                             ionoutc, gps_time_ppm, delt, path_loss_enable,
                             fixed_gain, ant_pat, synth_config, elevation_mask,
                             states, &sample_count, error)) {
      memcpy(allocatedSat, saved_allocated, sizeof(saved_allocated));
      return false;
    }
    writeTrajectoryStates(trajectory, states, plan->target_count);
    alignment_discard.resize((size_t)sample_count * 2);
    renderCleanEpochWide(alignment_discard.data(), sample_count, state.channels,
                         state.gains, delt);
    state.sample_offset += (uint64_t)sample_count;
    refreshMatchedStateIfNeeded(&state, eph, synth_source, neph, ionoutc,
                                synth_config, attack_config, elevation_mask,
                                required_prns, plan->sample_rate_hz);
  }
  memcpy(allocatedSat, saved_allocated, sizeof(saved_allocated));
  trajectory.close();
  if (!trajectory || std::rename(trajectory_temporary.c_str(),
                                 options.trajectory_path.c_str()) != 0) {
    std::remove(trajectory_temporary.c_str());
    *error = "cannot finalize target trajectory artifact";
    return false;
  }
  char trajectory_hash[SHA256_HEX_SIZE];
  if (sha256_file_hex(options.trajectory_path.c_str(), trajectory_hash) != 0) {
    *error = "cannot checksum target trajectory artifact";
    return false;
  }
  result->trajectory_sha256 = trajectory_hash;
  result->target_allocation_passed = true;
  return true;
}

static void accountMatchedAsyncEvent(const uhd::async_metadata_t &metadata,
                                     MatchedCodeRunResult *result,
                                     bool *fatal) {
  if (metadata.event_code == uhd::async_metadata_t::EVENT_CODE_UNDERFLOW ||
      metadata.event_code ==
          uhd::async_metadata_t::EVENT_CODE_UNDERFLOW_IN_PACKET) {
    ++result->underflows;
    *fatal = true;
    fprintf(stderr, "[UHD] ERROR: matched-code TX underflow #%llu\n",
            (unsigned long long)result->underflows);
  } else if (metadata.event_code ==
                 uhd::async_metadata_t::EVENT_CODE_SEQ_ERROR ||
             metadata.event_code ==
                 uhd::async_metadata_t::EVENT_CODE_SEQ_ERROR_IN_BURST) {
    ++result->sequence_errors;
    *fatal = true;
    fprintf(stderr, "[UHD] ERROR: matched-code TX sequence error #%llu\n",
            (unsigned long long)result->sequence_errors);
  } else if (metadata.event_code ==
             uhd::async_metadata_t::EVENT_CODE_TIME_ERROR) {
    ++result->time_errors;
    *fatal = true;
    fprintf(stderr, "[UHD] ERROR: matched-code timed start was missed\n");
  }
}

static void drainMatchedAsync(const uhd::tx_streamer::sptr &stream,
                              double first_timeout,
                              MatchedCodeRunResult *result, bool *fatal) {
  uhd::async_metadata_t metadata;
  double timeout = first_timeout;

  while (stream->recv_async_msg(metadata, timeout)) {
    accountMatchedAsyncEvent(metadata, result, fatal);
    timeout = 0.0;
  }
}

static bool runMatchedDryRender(
    const matched_code_plan_t &plan, MatchedCodeRunResult *result,
    const channel_t initial_channels[MAX_CHAN],
    const int initial_gains[MAX_CHAN],
    const ephem_t initial_active_ephemeris[MAX_SAT],
    const synth_ephem_store_t *initial_synthetic_ephemeris,
    int initial_ephemeris_index, const epoch_plan_t *initial_epoch_plan,
    ephem_t eph[][MAX_SAT], const ephem_t synth_source[][MAX_SAT], int neph,
    gpstime_t sample_zero, ionoutc_t *ionoutc,
    const synth_config_t *synth_config, const attack_config_t *attack_config,
    double elevation_mask, const int *required_prns, double gps_time_ppm,
    double delt, int path_loss_enable, int fixed_gain, double ant_pat[37],
    std::string *error) {
  MatchedSimulationState state;
  int saved_allocated[MAX_SAT];
  matched_code_source_config_t source_config =
      matchedSourceConfig(plan, plan.total_samples);
  matched_code_source_t source;
  char source_error[256];
  std::vector<double> alignment_discard;
  std::vector<int16_t> jammer_output;

  if (matched_code_source_init(&source, &source_config, source_error,
                               sizeof(source_error)) != 0) {
    *error = source_error;
    return false;
  }
  result->internal_alignment_samples = 0;
  result->rendered_jammer_samples = 0;
  result->quantized_samples = 0;
  result->jammer_iq_fnv1a64 = FNV1A64_OFFSET_BASIS;
  result->source_metrics = {};
  memcpy(saved_allocated, allocatedSat, sizeof(saved_allocated));
  initializeMatchedSimulationState(
      &state, initial_channels, initial_gains, initial_active_ephemeris,
      initial_synthetic_ephemeris, initial_ephemeris_index, initial_epoch_plan,
      sample_zero);

  while (state.sample_offset < plan.total_samples) {
    int sample_count;
    if (!renderMatchedFrame(
            &state, &source, plan, plan.total_samples, sample_zero, ionoutc,
            gps_time_ppm, delt, path_loss_enable, fixed_gain, ant_pat,
            synth_config, elevation_mask, &alignment_discard, &jammer_output,
            nullptr, result, &sample_count, error)) {
      memcpy(allocatedSat, saved_allocated, sizeof(saved_allocated));
      return false;
    }
    refreshMatchedStateIfNeeded(&state, eph, synth_source, neph, ionoutc,
                                synth_config, attack_config, elevation_mask,
                                required_prns, plan.sample_rate_hz);
    if (result->source_metrics.clipped_components > 0) {
      *error = "unexpected SC16 clipping during dry-run render";
      memcpy(allocatedSat, saved_allocated, sizeof(saved_allocated));
      return false;
    }
  }
  memcpy(allocatedSat, saved_allocated, sizeof(saved_allocated));
  matched_code_source_get_metrics(&source, &result->source_metrics);
  return matched_code_source_done(&source) &&
         result->internal_alignment_samples == plan.total_samples &&
         result->rendered_jammer_samples == plan.total_samples &&
         result->quantized_samples == plan.total_samples;
}

struct MatchedFrame {
  std::vector<int16_t> samples;
  size_t sample_count = 0;
};

static bool runMatchedTransmitter(
    const matched_code_plan_t &plan, const MatchedCodeOptions &options,
    MatchedCodeRunResult *result, const uhd::usrp::multi_usrp::sptr &usrp,
    const uhd::tx_streamer::sptr &stream, size_t max_send_samples,
    double requested_start_delay_seconds, double trimble_tag_monotonic,
    double trimble_tag_lead_seconds, double trimble_start_offset_seconds,
    int prebuffer_count, const channel_t initial_channels[MAX_CHAN],
    const int initial_gains[MAX_CHAN],
    const ephem_t initial_active_ephemeris[MAX_SAT],
    const synth_ephem_store_t *initial_synthetic_ephemeris,
    int initial_ephemeris_index, const epoch_plan_t *initial_epoch_plan,
    ephem_t eph[][MAX_SAT], const ephem_t synth_source[][MAX_SAT], int neph,
    gpstime_t sample_zero, ionoutc_t *ionoutc,
    const synth_config_t *synth_config, const attack_config_t *attack_config,
    double elevation_mask, const int *required_prns, double gps_time_ppm,
    double delt, int path_loss_enable, int fixed_gain, double ant_pat[37],
  std::string *error) {
  const uint64_t continuous_sample_limit =
      std::numeric_limits<uint64_t>::max();
  MatchedSimulationState state;
  int saved_allocated[MAX_SAT];
  matched_code_source_config_t source_config =
      matchedSourceConfig(plan, continuous_sample_limit);
  matched_code_source_t source;
  char source_error[256];
  std::vector<double> alignment_discard;
  std::vector<int16_t> jammer_output;
  std::deque<MatchedFrame> queue;
  std::string trajectory_temporary = options.trajectory_path + ".tmp";
  std::ofstream trajectory(trajectory_temporary,
                           std::ios::out | std::ios::trunc);
  bool fatal = false;

  if (!trajectory) {
    *error = "cannot create live target trajectory artifact";
    return false;
  }
  writeTrajectoryHeader(trajectory, plan, sample_zero);
  if (matched_code_source_init(&source, &source_config, source_error,
                               sizeof(source_error)) != 0) {
    *error = source_error;
    return false;
  }
  result->internal_alignment_samples = 0;
  result->rendered_jammer_samples = 0;
  result->quantized_samples = 0;
  result->sent_samples = 0;
  result->jammer_iq_fnv1a64 = FNV1A64_OFFSET_BASIS;
  result->source_metrics = {};
  memcpy(saved_allocated, allocatedSat, sizeof(saved_allocated));
  initializeMatchedSimulationState(
      &state, initial_channels, initial_gains, initial_active_ephemeris,
      initial_synthetic_ephemeris, initial_ephemeris_index, initial_epoch_plan,
      sample_zero);
  installSignalHandlers();

  for (int buffered = 0; buffered < prebuffer_count && !stop_requested;
       ++buffered) {
    MatchedFrame frame;
    int sample_count;
    if (!renderMatchedFrame(
            &state, &source, plan, continuous_sample_limit, sample_zero,
            ionoutc, gps_time_ppm, delt, path_loss_enable, fixed_gain, ant_pat,
            synth_config, elevation_mask, &alignment_discard, &jammer_output,
            &trajectory, result, &sample_count, error)) {
      memcpy(allocatedSat, saved_allocated, sizeof(saved_allocated));
      return false;
    }
    frame.sample_count = (size_t)sample_count;
    frame.samples.assign(jammer_output.begin(), jammer_output.end());
    queue.push_back(std::move(frame));
    refreshMatchedStateIfNeeded(&state, eph, synth_source, neph, ionoutc,
                                synth_config, attack_config, elevation_mask,
                                required_prns, plan.sample_rate_hz);
  }
  if (queue.empty()) {
    *error = stop_requested ? "interrupted during prebuffer"
                            : "prebuffer produced no samples";
    result->interrupted = stop_requested != 0;
    memcpy(allocatedSat, saved_allocated, sizeof(saved_allocated));
    return false;
  }
  if (result->source_metrics.clipped_components > 0) {
    *error = "unexpected SC16 clipping during prebuffer";
    memcpy(allocatedSat, saved_allocated, sizeof(saved_allocated));
    return false;
  }

  double start_delay_seconds = requested_start_delay_seconds;
  if (trimble_tag_monotonic >= 0.0) {
    double planned_delay =
        trimble_tag_lead_seconds + trimble_start_offset_seconds;
    double elapsed = getMonotonicSeconds() - trimble_tag_monotonic;
    start_delay_seconds = planned_delay - elapsed;
    fprintf(stderr,
            "[TRIMBLE] Matched-code preflight+prebuffer elapsed %.3f ms; "
            "remaining start margin %.3f ms\n",
            elapsed * 1000.0, start_delay_seconds * 1000.0);
    if (start_delay_seconds < TX_START_LEAD_MIN_SEC) {
      *error = "calibrated Trimble start became stale during preflight";
      memcpy(allocatedSat, saved_allocated, sizeof(saved_allocated));
      return false;
    }
  }
  if (start_delay_seconds < TX_START_LEAD_MIN_SEC) {
    *error =
        "hardware timed start became stale or lacks the minimum future margin";
    memcpy(allocatedSat, saved_allocated, sizeof(saved_allocated));
    return false;
  }

  bool burst_started = false;
  bool burst_finished = false;
  auto best_effort_end_of_burst = [&]() {
    if (!burst_started || burst_finished)
      return;
    try {
      uhd::tx_metadata_t end_metadata;
      end_metadata.start_of_burst = false;
      end_metadata.end_of_burst = true;
      end_metadata.has_time_spec = false;
      stream->send("", 0, end_metadata, 3.0);
      burst_finished = true;
      bool ignored_fatal = false;
      drainMatchedAsync(stream, 0.5, result, &ignored_fatal);
    } catch (const std::exception &shutdown_error) {
      fprintf(stderr, "[UHD] ERROR: best-effort end-of-burst failed: %s\n",
              shutdown_error.what());
    } catch (...) {
      fprintf(stderr,
              "[UHD] ERROR: best-effort end-of-burst failed unexpectedly\n");
    }
  };

  try {
    usrp->set_time_now(uhd::time_spec_t(0.0));
    result->uhd_start_time_seconds = start_delay_seconds;
    result->start_margin_met = start_delay_seconds >= TX_START_LEAD_MIN_SEC;
    uhd::tx_metadata_t metadata;
    metadata.start_of_burst = true;
    metadata.end_of_burst = false;
    metadata.has_time_spec = true;
    metadata.time_spec = uhd::time_spec_t(start_delay_seconds);
    fprintf(stderr,
            "[TX] Matched-code jammer-only timed start in %.3f ms with %zu "
            "prebuffered frame(s)\n",
            start_delay_seconds * 1000.0, queue.size());

    while (!queue.empty() && !fatal && !stop_requested) {
      MatchedFrame &frame = queue.front();
      size_t sent_from_frame = 0;
      double timeout = metadata.has_time_spec ? start_delay_seconds + 1.0 : 3.0;

      while (sent_from_frame < frame.sample_count && !fatal &&
             !stop_requested) {
        size_t request =
            std::min(max_send_samples, frame.sample_count - sent_from_frame);
        burst_started = true;
        size_t sent = stream->send(&frame.samples[sent_from_frame * 2], request,
                                   metadata, timeout);
        if (sent == 0) {
          *error = "UHD send returned zero samples";
          fatal = true;
          break;
        }
        sent_from_frame += sent;
        result->sent_samples += (uint64_t)sent;
        metadata.start_of_burst = false;
        metadata.has_time_spec = false;
        timeout = 3.0;
      }
      drainMatchedAsync(stream, 0.0, result, &fatal);
      queue.pop_front();

      if (!fatal && !stop_requested) {
        MatchedFrame next_frame;
        int sample_count;
        if (!renderMatchedFrame(
                &state, &source, plan, continuous_sample_limit, sample_zero,
                ionoutc, gps_time_ppm, delt, path_loss_enable, fixed_gain,
                ant_pat, synth_config, elevation_mask, &alignment_discard,
                &jammer_output, &trajectory, result, &sample_count, error)) {
          fatal = true;
          break;
        }
        next_frame.sample_count = (size_t)sample_count;
        next_frame.samples.assign(jammer_output.begin(), jammer_output.end());
        queue.push_back(std::move(next_frame));
        refreshMatchedStateIfNeeded(&state, eph, synth_source, neph, ionoutc,
                                    synth_config, attack_config, elevation_mask,
                                    required_prns, plan.sample_rate_hz);
        if (result->source_metrics.clipped_components > 0) {
          *error = "unexpected SC16 clipping during live render";
          fatal = true;
        }
      }
    }

    metadata.start_of_burst = false;
    metadata.end_of_burst = true;
    metadata.has_time_spec = false;
    stream->send("", 0, metadata, 3.0);
    burst_finished = true;
    drainMatchedAsync(stream, 0.5, result, &fatal);
  } catch (const std::exception &transmit_error) {
    fatal = true;
    if (error->empty())
      *error =
          std::string("UHD live transmission failed: ") + transmit_error.what();
    fprintf(stderr, "[UHD] ERROR: %s\n", transmit_error.what());
    best_effort_end_of_burst();
  } catch (...) {
    fatal = true;
    if (error->empty())
      *error = "UHD live transmission failed with an unknown exception";
    fprintf(stderr, "[UHD] ERROR: live transmission failed unexpectedly\n");
    best_effort_end_of_burst();
  }
  result->interrupted = stop_requested != 0;
  matched_code_source_get_metrics(&source, &result->source_metrics);
  memcpy(allocatedSat, saved_allocated, sizeof(saved_allocated));
  trajectory.close();
  if (!trajectory || std::rename(trajectory_temporary.c_str(),
                                 options.trajectory_path.c_str()) != 0) {
    *error = "cannot finalize live target trajectory artifact";
    fatal = true;
  } else {
    char trajectory_hash[SHA256_HEX_SIZE];
    if (sha256_file_hex(options.trajectory_path.c_str(), trajectory_hash) !=
        0) {
      *error = "cannot checksum live target trajectory artifact";
      fatal = true;
    } else {
      result->trajectory_sha256 = trajectory_hash;
    }
  }

  if (error->empty()) {
    if (result->time_errors > 0)
      *error = "UHD reported a timed-transmission error";
    else if (result->sequence_errors > 0)
      *error = "UHD reported a TX sequence error";
    else if (result->underflows > 0)
      *error = "UHD reported a TX underflow";
    else if (result->source_metrics.clipped_components > 0)
      *error = "the matched-code jammer clipped during live rendering";
    else if (!result->interrupted)
      *error = "the continuous matched-code source ended before operator stop";
    else if (result->sent_samples == 0)
      *error = "operator stop occurred before any jammer samples were sent";
  }
  return !fatal && result->interrupted && result->sent_samples > 0 &&
         result->underflows == 0 && result->sequence_errors == 0 &&
         result->time_errors == 0 &&
         result->source_metrics.clipped_components == 0;
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
      "Continuous matched-code jammer-only mode:\n"
      "  --matched-code-target-prns <list>  Sole jammer PRN selector\n"
      "  --matched-code-amplitude <value>   Jammer RMS/full-scale amplitude\n"
      "  --matched-code-phase-seed <N>      Independent carrier-phase seed\n"
      "  --manifest <file>                  Required run manifest output\n"
      "  --trajectory <file>                Target-state artifact output\n"
      "  --calibration-id <text>            Controlled setup identity\n"
      "  --confirm-controlled-rf            Required for live matched-code TX\n"
      "  --dry-run                          Validate 100 ms without opening UHD\n"
      "  Mode starts at sample zero and runs until SIGINT/SIGTERM; -P, -d,\n"
      "  -n, J/S, onset, offset, and ramp controls are not accepted.\n"
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
  int partial_prns_set = FALSE;
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
  double requested_samp_freq;
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
  MatchedCodeOptions matched_options;
  matched_code_plan_t matched_plan{};
  MatchedCodeRunResult matched_result;
  int matched_required_prns[MAX_SAT] = {0};
  std::string matched_scenario_sha256;
  std::string matched_ephemeris_sha256;

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
  uhd::usrp::multi_usrp::sptr usrp;
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
  requested_samp_freq = TX_SAMPLERATE;
  g0.week = -1;
  g0.sec = 0.0;
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
    if (strcmp(opt, "dry-run") == 0) {
      matched_options.dry_run = true;
      continue;
    }
    if (strcmp(opt, "confirm-controlled-rf") == 0) {
      matched_options.controlled_rf_confirmed = true;
      continue;
    }

    if (i + 1 >= argc) {
      fprintf(stderr, "ERROR: Missing value for option --%s.\n", opt);
      return 1;
    }

    const char *val = argv[++i];

    if (strcmp(opt, "gps-week") == 0) {
      char *end = NULL;
      errno = 0;
      long parsed = strtol(val, &end, 10);
      if (val[0] == '\0' || errno == ERANGE || end == NULL || *end != '\0' ||
          parsed < 0 || parsed > 65535) {
        fprintf(stderr, "ERROR: --gps-week must be an integer in 0..65535.\n");
        return 1;
      }
      explicit_gps_week = (int)parsed;
      gps_week_set = TRUE;
      continue;
    }
    if (strcmp(opt, "gps-tow") == 0) {
      char *end = NULL;
      errno = 0;
      double parsed = strtod(val, &end);
      if (val[0] == '\0' || errno == ERANGE || end == NULL || *end != '\0' ||
          !std::isfinite(parsed) || parsed < 0.0 || parsed >= 604800.0) {
        fprintf(stderr,
                "ERROR: --gps-tow must be a finite value in [0, 604800).\n");
        return 1;
      }
      explicit_gps_tow = parsed;
      gps_tow_set = TRUE;
      continue;
    }
    if (strcmp(opt, "tx-advance-ns") == 0) {
      char *end = NULL;
      errno = 0;
      long long parsed = strtoll(val, &end, 10);
      if (val[0] == '\0' || errno == ERANGE || end == NULL || *end != '\0' ||
          parsed < 0 || parsed > (long long)(TX_START_LEAD_MAX_SEC * 1.0e9)) {
        fprintf(stderr, "ERROR: --tx-advance-ns must be 0-%.0f seconds.\n",
                TX_START_LEAD_MAX_SEC);
        return 1;
      }
      tx_advance_ns = parsed;
      continue;
    }
    if (strcmp(opt, "addr") == 0) {
      strncpy(usrp_addr, val, sizeof(usrp_addr) - 1);
      usrp_addr[sizeof(usrp_addr) - 1] = '\0';
      matched_options.address_set = true;
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
      requested_samp_freq = parsed;
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
      char *end = NULL;
      errno = 0;
      double parsed = strtod(val, &end);
      if (val[0] == '\0' || errno == ERANGE || end == NULL || *end != '\0' ||
          !std::isfinite(parsed)) {
        fprintf(stderr, "ERROR: --gain must be a finite number of dB.\n");
        return 1;
      }
      tx_gain = parsed;
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
      if (val[0] == '\0' || val[0] == '-' || errno == ERANGE || end == NULL ||
          *end != '\0') {
        fprintf(stderr, "ERROR: --channel must be a non-negative integer.\n");
        return 1;
      }
      tx_channel = (size_t)parsed;
      matched_options.channel_set = true;
      continue;
    }
    if (strcmp(opt, "antenna") == 0) {
      strncpy(tx_antenna, val, sizeof(tx_antenna) - 1);
      tx_antenna[sizeof(tx_antenna) - 1] = '\0';
      matched_options.antenna_set = true;
      continue;
    }
    if (strcmp(opt, "matched-code-target-prns") == 0) {
      matched_options.enabled = true;
      matched_options.target_prns = val;
      continue;
    }
    if (strcmp(opt, "matched-code-amplitude") == 0) {
      char *end = NULL;
      errno = 0;
      matched_options.amplitude = strtod(val, &end);
      if (val[0] == '\0' || errno == ERANGE || end == NULL || *end != '\0' ||
          !std::isfinite(matched_options.amplitude) ||
          matched_options.amplitude <= 0.0 || matched_options.amplitude > 1.0) {
        fprintf(stderr,
                "ERROR: --matched-code-amplitude must be in (0, 1].\n");
        return 1;
      }
      matched_options.amplitude_set = true;
      continue;
    }
    if (strcmp(opt, "matched-code-phase-seed") == 0) {
      char *end = NULL;
      unsigned long long parsed;
      errno = 0;
      parsed = strtoull(val, &end, 10);
      if (val[0] == '\0' || val[0] == '-' || errno == ERANGE || end == NULL ||
          *end != '\0') {
        fprintf(
            stderr,
            "ERROR: --matched-code-phase-seed must be an unsigned integer.\n");
        return 1;
      }
      matched_options.phase_seed = (uint64_t)parsed;
      matched_options.phase_seed_set = true;
      continue;
    }
    if (strcmp(opt, "manifest") == 0) {
      matched_options.manifest_path = val;
      continue;
    }
    if (strcmp(opt, "trajectory") == 0) {
      matched_options.trajectory_path = val;
      continue;
    }
    if (strcmp(opt, "calibration-id") == 0) {
      matched_options.calibration_id = val;
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
      partial_prns_set = TRUE;
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
      matched_options.legacy_js_set = true;
      attack_cfg.jam_js_db = atof(optarg);
      jam_js_linear = pow(10.0, attack_cfg.jam_js_db / 20.0);
      break;
    case 'G':
      matched_options.legacy_gain_boost_set = true;
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

  if (!matched_options.enabled &&
      (matched_options.dry_run || matched_options.controlled_rf_confirmed ||
       matched_options.amplitude_set || matched_options.phase_seed_set ||
       !matched_options.manifest_path.empty() ||
       !matched_options.trajectory_path.empty() ||
       !matched_options.calibration_id.empty())) {
    fprintf(stderr, "ERROR: matched-code-only options require "
                    "--matched-code-target-prns.\n");
    return 1;
  }

  if (matched_options.enabled) {
    char plan_error[256] = "";

    if (!matched_options.amplitude_set || !matched_options.phase_seed_set ||
        matched_options.manifest_path.empty()) {
      fprintf(stderr,
              "ERROR: matched-code mode requires target PRNs, output "
              "amplitude, phase seed, and --manifest.\n");
      return 1;
    }
    if (duration_specified) {
      fprintf(stderr, "ERROR: continuous matched-code mode does not accept -d.\n");
      return 1;
    }
    if (current_time_mode == TRUE) {
      fprintf(stderr, "ERROR: continuous matched-code mode does not accept -n; "
                      "use the calibrated timed-start options.\n");
      return 1;
    }
    if (partial_prns_set == TRUE) {
      fprintf(stderr, "ERROR: matched-code jammer-only mode rejects -P; "
                      "--matched-code-target-prns is the sole jammer selector.\n");
      return 1;
    }
    if (attack_enabled || matched_options.legacy_js_set ||
        matched_options.legacy_gain_boost_set) {
      fprintf(stderr,
              "ERROR: matched-code mode is mutually exclusive with -A, -J, "
              "and -G legacy attack controls.\n");
      return 1;
    }
    if (trimble_rtcm_mode == TRUE) {
      fprintf(stderr,
              "ERROR: matched-code mode requires frozen RINEX ephemeris; "
              "live RTCM is not allowed after preflight.\n");
      return 1;
    }
    if (fabs(samp_freq * EPOCH_TARGET_SEC -
             round(samp_freq * EPOCH_TARGET_SEC)) > 1.0e-6) {
      fprintf(stderr,
              "ERROR: matched-code mode requires a rate with an integer "
              "100 ms epoch sample count.\n");
      return 1;
    }
    if (!initializeMatchedJammerPlan(
            &matched_plan, samp_freq, matched_options.target_prns,
            matched_options.amplitude, matched_options.phase_seed, plan_error,
            sizeof(plan_error))) {
      fprintf(stderr, "ERROR: invalid matched-code plan: %s\n", plan_error);
      return 1;
    }
    for (size_t target = 0; target < matched_plan.target_count; ++target) {
      int prn = matched_plan.target_prns[target];
      matched_required_prns[prn - 1] = 1;
    }
    if (matched_options.trajectory_path.empty()) {
      std::string base = matched_options.manifest_path;
      if (base.size() >= 5 && base.substr(base.size() - 5) == ".json")
        base.resize(base.size() - 5);
      matched_options.trajectory_path = base + ".trajectory.csv";
    }
    {
      std::error_code manifest_path_error;
      std::error_code trajectory_path_error;
      std::filesystem::path manifest_absolute = std::filesystem::absolute(
          matched_options.manifest_path, manifest_path_error);
      std::filesystem::path trajectory_absolute = std::filesystem::absolute(
          matched_options.trajectory_path, trajectory_path_error);
      if (manifest_path_error || trajectory_path_error) {
        fprintf(stderr,
                "ERROR: matched-code artifact paths cannot be resolved.\n");
        return 1;
      }
      if (manifest_absolute.lexically_normal() ==
          trajectory_absolute.lexically_normal()) {
        fprintf(stderr, "ERROR: --manifest and --trajectory must use different "
                        "paths.\n");
        return 1;
      }
    }
    if (!matched_options.dry_run) {
      if (!matched_options.address_set || !matched_options.channel_set ||
          !matched_options.antenna_set || !tx_gain_set ||
          !std::isfinite(tx_gain) || matched_options.calibration_id.empty() ||
          !matched_options.controlled_rf_confirmed) {
        fprintf(stderr, "ERROR: live matched-code TX requires explicit --addr, "
                        "--channel, --antenna, --gain, --calibration-id, and "
                        "--confirm-controlled-rf.\n");
        return 1;
      }
      if (!trimble_mode && (!(gps_week_set && gps_tow_set) ||
                            strcmp(time_source, "internal") == 0)) {
        fprintf(stderr,
                "ERROR: accepted live matched-code TX requires the Trimble "
                "time-tag path or explicit GPS week/TOW with an external/"
                "GPSDO time source.\n");
        return 1;
      }
    }
    {
      std::string probe_path = matched_options.manifest_path + ".probe";
      std::ofstream probe(probe_path, std::ios::out | std::ios::trunc);
      if (!probe) {
        fprintf(stderr, "ERROR: manifest path is not writable: %s\n",
                matched_options.manifest_path.c_str());
        return 1;
      }
      probe.close();
      if (!probe || std::remove(probe_path.c_str()) != 0) {
        fprintf(stderr, "ERROR: manifest path cannot be finalized: %s\n",
                matched_options.manifest_path.c_str());
        return 1;
      }
    }
  }

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

  if (matched_options.enabled) {
    matched_result.status = "incomplete";
    matched_result.exit_status = 1;
    matched_result.device_address = usrp_addr;
    if (trimble_mode)
      matched_result.start_mode = "trimble_time_tag";
    else if (gps_week_set && gps_tow_set)
      matched_result.start_mode = "explicit_gps_time";
    else if (timeoverwrite)
      matched_result.start_mode = "explicit_utc_time";
    else
      matched_result.start_mode = "frozen_rinex_epoch";
    if (matched_options.dry_run) {
      matched_result.device_type = "not_opened";
      matched_result.device_product = "not_opened";
      matched_result.actual_rate_hz = requested_samp_freq;
      matched_result.actual_frequency_hz = TX_FREQUENCY;
      matched_result.actual_gain_db = tx_gain_set ? tx_gain : 0.0;
      matched_result.actual_antenna = tx_antenna;
    }
    if (!writeMatchedManifestAtomic(
            matched_options, matched_plan, matched_result, navfile,
            matched_ephemeris_sha256, matched_scenario_sha256, g0, xyz[0],
            requested_samp_freq, usrp_addr, tx_channel, tx_antenna, tx_gain,
            clock_source, time_source, prebuffer_count, trimble_tx_cal_ns,
            gps_time_ppm, trimble_mode)) {
      fprintf(stderr,
              "ERROR: cannot write initial matched-code manifest before "
              "preflight.\n");
      return 1;
    }
  }

  auto finish_matched_early_failure = [&](const char *status,
                                          const std::string &reason) -> int {
    if (matched_options.enabled) {
      matched_result.status = status;
      matched_result.failure_reason = reason;
      matched_result.exit_status = 1;
      if (!writeMatchedManifestAtomic(
              matched_options, matched_plan, matched_result, navfile,
              matched_ephemeris_sha256, matched_scenario_sha256, g0, xyz[0],
              requested_samp_freq, usrp_addr, tx_channel, tx_antenna, tx_gain,
              clock_source, time_source, prebuffer_count, trimble_tx_cal_ns,
              gps_time_ppm, trimble_mode)) {
        fprintf(stderr,
                "ERROR: cannot finalize failed matched-code manifest.\n");
      }
    }
    return 1;
  };

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
      return finish_matched_early_failure(
          "preflight_error", "frozen navigation file contains no ephemeris");
    } else if (neph == -1) {
      fprintf(stderr, "ERROR: Ephemeris file not found.\n");
      return finish_matched_early_failure(
          "preflight_error", "frozen navigation file was not found");
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

  if (matched_options.enabled) {
    char digest[SHA256_HEX_SIZE];
    if (sha256_file_hex(navfile, digest) != 0) {
      fprintf(stderr, "ERROR: cannot checksum frozen navigation file '%s'.\n",
              navfile);
      return finish_matched_early_failure(
          "preflight_error", "cannot checksum frozen navigation file");
    }
    matched_ephemeris_sha256 = digest;
  }

  ////////////////////////////////////////////////////////////
  // Configure USRP X300
  ////////////////////////////////////////////////////////////

  if (matched_options.enabled && matched_options.dry_run) {
    fprintf(stderr,
            "\n[DRY-RUN] UHD discovery/open/configuration is intentionally "
            "skipped.\n");
    matched_result.device_type = "not_opened";
    matched_result.device_product = "not_opened";
    matched_result.device_address = usrp_addr;
    matched_result.actual_rate_hz = samp_freq;
    matched_result.actual_frequency_hz = TX_FREQUENCY;
    matched_result.actual_gain_db = tx_gain_set ? tx_gain : 0.0;
    matched_result.actual_antenna = tx_antenna;
  } else {
    fprintf(stderr, "\n[UHD] Configuring USRP X300 at %s ...\n", usrp_addr);

    try {
      uhd::device_addr_t dev_addr;
      dev_addr["addr"] = usrp_addr;
      if (matched_options.enabled) {
        uhd::device_addrs_t discovered = uhd::device::find(dev_addr);
        if (discovered.size() != 1) {
          fprintf(stderr,
                  "ERROR: matched-code preflight found %zu UHD devices at "
                  "explicit address %s; exactly one is required.\n",
                  discovered.size(), usrp_addr);
          return finish_matched_early_failure(
              "device_error",
              "device discovery did not return exactly one device");
        }
        std::string discovered_type =
            discovered[0].has_key("type") ? discovered[0]["type"] : "";
        std::string discovered_product =
            discovered[0].has_key("product") ? discovered[0]["product"] : "";
        if (discovered_type != "x300" ||
            (discovered_product != "X300" && discovered_product != "X310")) {
          fprintf(stderr,
                  "ERROR: matched-code mode requires an X300/X310, discovered "
                  "type='%s' product='%s'.\n",
                  discovered_type.c_str(), discovered_product.c_str());
          return finish_matched_early_failure(
              "device_error", "discovered device is not an X300/X310");
        }
        matched_result.device_type = discovered_type;
        matched_result.device_product = discovered_product;
        matched_result.device_serial =
            discovered[0].has_key("serial") ? discovered[0]["serial"] : "";
        matched_result.device_address =
            discovered[0].has_key("addr") ? discovered[0]["addr"] : usrp_addr;
        fprintf(stderr, "[UHD] Discovered %s serial %s at %s\n",
                discovered_product.c_str(),
                matched_result.device_serial.c_str(),
                matched_result.device_address.c_str());
      }

      usrp = uhd::usrp::multi_usrp::make(dev_addr);

      usrp->set_clock_source(clock_source);
      usrp->set_time_source(time_source);

      {
        size_t tx_channels = usrp->get_tx_num_channels();
        matched_result.tx_channel_count = tx_channels;
        if (tx_channel >= tx_channels) {
          fprintf(stderr,
                  "ERROR: --channel %zu is invalid; device reports %zu TX "
                  "channel(s).\n",
                  tx_channel, tx_channels);
          return finish_matched_early_failure(
              "device_error", "selected TX channel does not exist");
        }
      }

      fprintf(stderr, "[UHD] TX channel: %zu\n", tx_channel);

      usrp->set_tx_rate(samp_freq, tx_channel);
      fprintf(stderr, "[UHD] TX rate:    %.0f Hz (actual %.0f Hz)\n", samp_freq,
              usrp->get_tx_rate(tx_channel));

      uhd::tune_request_t tune_req(TX_FREQUENCY);
      usrp->set_tx_freq(tune_req, tx_channel);
      fprintf(stderr, "[UHD] TX freq:    %.0f Hz (actual %.0f Hz)\n",
              TX_FREQUENCY, usrp->get_tx_freq(tx_channel));

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

        if (matched_options.enabled &&
            (tx_gain < min_gain || tx_gain > max_gain)) {
          fprintf(stderr,
                  "ERROR: matched-code TX gain %.1f dB is outside device range "
                  "[%.1f, %.1f] dB.\n",
                  tx_gain, min_gain, max_gain);
          return finish_matched_early_failure(
              "device_error", "requested TX gain is outside the device range");
        }
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
      if (matched_options.enabled &&
          fabs(samp_freq - matched_plan.sample_rate_hz) >
              MATCHED_RATE_TOLERANCE_HZ) {
        fprintf(stderr,
                "ERROR: actual TX rate %.9f differs from requested %.9f by "
                "more than %.1f Hz.\n",
                samp_freq, matched_plan.sample_rate_hz,
                MATCHED_RATE_TOLERANCE_HZ);
        return finish_matched_early_failure(
            "device_error", "actual TX rate differs from the requested rate");
      }
      matched_result.actual_rate_hz = samp_freq;
      matched_result.actual_frequency_hz = usrp->get_tx_freq(tx_channel);
      matched_result.actual_gain_db = usrp->get_tx_gain(tx_channel);
      matched_result.actual_antenna = usrp->get_tx_antenna(tx_channel);
    } catch (const std::exception &e) {
      fprintf(stderr, "ERROR: Failed to configure USRP: %s\n", e.what());
      if (matched_options.enabled)
        return finish_matched_early_failure("device_error", e.what());
      return 1;
    }
  }

  delt = 1.0 / samp_freq;
  initEpochPlan(&epoch_plan, samp_freq);
  iq_buff_size = epoch_plan.max_samples;
  if (matched_options.enabled) {
    char plan_error[256] = "";
    if (fabs(samp_freq * EPOCH_TARGET_SEC -
             round(samp_freq * EPOCH_TARGET_SEC)) > 1.0e-6 ||
        !initializeMatchedJammerPlan(
            &matched_plan, samp_freq, matched_options.target_prns,
            matched_options.amplitude, matched_options.phase_seed, plan_error,
            sizeof(plan_error))) {
      fprintf(stderr,
              "ERROR: actual-rate matched-code plan is unsupported: %s\n",
              plan_error[0] != '\0' ? plan_error : "non-integral 100 ms epoch");
      return finish_matched_early_failure(
          "preflight_error", "actual-rate matched-code plan is unsupported");
    }
  }

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
      return finish_matched_early_failure(
          "preflight_error", "failed to connect to the calibrated time tag");
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
      return finish_matched_early_failure(
          "preflight_error", "no valid calibrated time tag was received");
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
  if (matched_options.enabled)
    fprintf(stderr, "Duration = continuous until SIGINT/SIGTERM\n");
  else if (stream_forever)
    fprintf(stderr, "Duration = streaming until interrupted\n");
  else
    fprintf(stderr, "Duration = %.1f [sec]\n", (double)numd / 10.0);

  if (matched_options.enabled) {
    std::ostringstream scenario;
    scenario << std::setprecision(17) << matched_ephemeris_sha256 << '|'
             << xyz[0][0] << '|' << xyz[0][1] << '|' << xyz[0][2] << '|'
             << g0.week << '|' << g0.sec << '|' << matched_plan.sample_rate_hz
             << "|continuous|" << matched_options.target_prns << '|'
             << matched_options.amplitude << '|' << matched_options.phase_seed
             << '|';
    for (sv = 0; sv < MAX_SAT; ++sv) {
      scenario << (int)synth_cfg.mode[sv] << ':' << synth_cfg.source_prn[sv]
               << ':' << synth_cfg.azimuth[sv] << ':' << synth_cfg.elevation[sv]
               << ';';
    }
    std::string canonical = scenario.str();
    char digest[SHA256_HEX_SIZE];
    sha256_bytes_hex(canonical.data(), canonical.size(), digest);
    matched_scenario_sha256 = digest;
  }

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
      return finish_matched_early_failure(
          "preflight_error", "no current frozen ephemeris set was found");
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

        if (scanEphemerisForRevive(
                synth_source, neph, sv + 1, synth_ref, xyz[0], &revive_template,
                &template_toe, &delta_sec, &elev_deg, &found_ephem) == FALSE) {
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
          return finish_matched_early_failure(
              "preflight_error", "revive target scenario is not usable");
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
    return finish_matched_early_failure(
        "preflight_error", "failed to allocate the clean IQ buffer");
  }

  for (i = 0; i < MAX_CHAN; i++)
    chan[i].prn = 0;
  for (sv = 0; sv < MAX_SAT; sv++)
    allocatedSat[sv] = -1;

  grx = g0;
  allocateChannel(chan, active_eph, ionoutc, grx, xyz[0], elvmask, &attack_cfg,
                  &synth_cfg,
                  matched_options.enabled ? matched_required_prns : nullptr);

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

  if (matched_options.enabled) {
    const ephem_t(*matched_synth_source)[MAX_SAT] =
        has_revive_mode == TRUE ? revive_scan_eph : eph;
    std::string matched_error;

    matched_result.status = "incomplete";
    matched_result.exit_status = 1;
    if (!writeMatchedManifestAtomic(
            matched_options, matched_plan, matched_result, navfile,
            matched_ephemeris_sha256, matched_scenario_sha256, g0, xyz[0],
            requested_samp_freq, usrp_addr, tx_channel, tx_antenna, tx_gain,
            clock_source, time_source, prebuffer_count, trimble_tx_cal_ns,
            gps_time_ppm, trimble_mode)) {
      fprintf(stderr,
              "ERROR: cannot write incomplete matched-code manifest.\n");
      return 1;
    }

    fprintf(stderr,
            "\n[MATCHED] Jammer-only preflight: targets=%s validation-samples="
            "%llu amplitude=%.9f predicted-headroom=%.3f dB\n",
            matched_options.target_prns.c_str(),
            (unsigned long long)matched_plan.total_samples,
            matched_options.amplitude, matched_plan.predicted_headroom_db);
    if (!runMatchedPreflight(
            &matched_plan, &matched_result, matched_options, chan, gain,
            active_eph, &synth_eph, ieph, &epoch_plan, eph,
            matched_synth_source, neph, g0, &ionoutc, &synth_cfg, &attack_cfg,
            elvmask, matched_required_prns, gps_time_ppm, delt,
            path_loss_enable, fixed_gain, ant_pat, &matched_error)) {
      matched_result.status = "preflight_error";
      matched_result.failure_reason = matched_error;
      matched_result.exit_status = 1;
      writeMatchedManifestAtomic(
          matched_options, matched_plan, matched_result, navfile,
          matched_ephemeris_sha256, matched_scenario_sha256, g0, xyz[0],
          requested_samp_freq, usrp_addr, tx_channel, tx_antenna, tx_gain,
          clock_source, time_source, prebuffer_count, trimble_tx_cal_ns,
          gps_time_ppm, trimble_mode);
      fprintf(stderr, "ERROR: matched-code preflight failed: %s\n",
              matched_error.c_str());
      return 1;
    }
    fprintf(stderr,
            "[MATCHED] Clean simulator IQ is internal alignment state only and "
            "will be discarded before the RF adapter.\n");

    if (matched_options.dry_run) {
      if (runMatchedDryRender(
              matched_plan, &matched_result, chan, gain, active_eph, &synth_eph,
              ieph, &epoch_plan, eph, matched_synth_source, neph, g0, &ionoutc,
              &synth_cfg, &attack_cfg, elvmask, matched_required_prns,
              gps_time_ppm, delt, path_loss_enable, fixed_gain, ant_pat,
              &matched_error)) {
        matched_result.status = "dry_run";
        matched_result.exit_status = 0;
      } else {
        matched_result.status =
            matched_result.source_metrics.clipped_components > 0
                ? "clipping"
                : "preflight_error";
        matched_result.failure_reason = matched_error;
        matched_result.exit_status = 1;
      }
      if (!writeMatchedManifestAtomic(
              matched_options, matched_plan, matched_result, navfile,
              matched_ephemeris_sha256, matched_scenario_sha256, g0, xyz[0],
              requested_samp_freq, usrp_addr, tx_channel, tx_antenna, tx_gain,
              clock_source, time_source, prebuffer_count, trimble_tx_cal_ns,
              gps_time_ppm, trimble_mode)) {
        fprintf(stderr, "ERROR: cannot finalize dry-run manifest.\n");
        return 1;
      }
      fprintf(
          stderr,
          "[DRY-RUN] status=%s rendered=%llu clipping=%llu; UHD was "
          "never opened.\n",
          matched_result.status.c_str(),
          (unsigned long long)matched_result.quantized_samples,
          (unsigned long long)matched_result.source_metrics.clipped_components);
      return matched_result.exit_status;
    }

    if (!writeMatchedManifestAtomic(
            matched_options, matched_plan, matched_result, navfile,
            matched_ephemeris_sha256, matched_scenario_sha256, g0, xyz[0],
            requested_samp_freq, usrp_addr, tx_channel, tx_antenna, tx_gain,
            clock_source, time_source, prebuffer_count, trimble_tx_cal_ns,
            gps_time_ppm, trimble_mode)) {
      fprintf(stderr,
              "ERROR: cannot update matched-code manifest before arming.\n");
      return 1;
    }

    bool completed = false;
    try {
      completed = runMatchedTransmitter(
          matched_plan, matched_options, &matched_result, usrp, tx_stream,
          max_samps, (double)tx_advance_ns * 1.0e-9, trimble_tag_mono,
          (double)trimble_tag_lead_ms * 1.0e-3, (double)trimble_start_offset,
          prebuffer_count, chan, gain, active_eph, &synth_eph, ieph,
          &epoch_plan, eph, matched_synth_source, neph, g0, &ionoutc,
          &synth_cfg, &attack_cfg, elvmask, matched_required_prns, gps_time_ppm,
          delt, path_loss_enable, fixed_gain, ant_pat, &matched_error);
    } catch (const std::exception &transmit_error) {
      matched_error = std::string("matched-code transmitter failed: ") +
                      transmit_error.what();
    } catch (...) {
      matched_error = "matched-code transmitter failed unexpectedly";
    }
    if (completed) {
      matched_result.status = "stopped";
      matched_result.exit_status = 0;
    } else {
      matched_result.failure_reason = matched_error;
      matched_result.exit_status = 1;
      if (matched_result.time_errors > 0)
        matched_result.status = "time_error";
      else if (matched_result.sequence_errors > 0)
        matched_result.status = "sequence_error";
      else if (matched_result.underflows > 0)
        matched_result.status = "underflow";
      else if (matched_result.source_metrics.clipped_components > 0)
        matched_result.status = "clipping";
      else if (matched_result.interrupted)
        matched_result.status = "interrupted";
      else if (matched_error.find("stale") != std::string::npos)
        matched_result.status = "preflight_error";
      else
        matched_result.status = "device_error";
    }
    if (!writeMatchedManifestAtomic(
            matched_options, matched_plan, matched_result, navfile,
            matched_ephemeris_sha256, matched_scenario_sha256, g0, xyz[0],
            requested_samp_freq, usrp_addr, tx_channel, tx_antenna, tx_gain,
            clock_source, time_source, prebuffer_count, trimble_tx_cal_ns,
            gps_time_ppm, trimble_mode)) {
      fprintf(stderr, "ERROR: cannot finalize matched-code run manifest.\n");
      return 1;
    }
    fprintf(stderr,
            "[TX] Matched-code status=%s sent=%llu underflows=%llu "
            "sequence-errors=%llu time-errors=%llu clipping=%llu\n",
            matched_result.status.c_str(),
            (unsigned long long)matched_result.sent_samples,
            (unsigned long long)matched_result.underflows,
            (unsigned long long)matched_result.sequence_errors,
            (unsigned long long)matched_result.time_errors,
            (unsigned long long)matched_result.source_metrics.clipped_components);
    return matched_result.exit_status;
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
    gpstime_t block_start_gps_time = getGpsTimeAtSampleOffset(
        first_sample_gps_time, generated_samples, samp_freq, gps_time_ppm);
    gpstime_t block_end_gps_time = getGpsTimeAtSampleOffset(
        first_sample_gps_time, generated_samples + sample_count, samp_freq,
        gps_time_ppm);

    current_epoch_duration =
        subGpsTime(block_end_gps_time, block_start_gps_time);
    grx = block_end_gps_time;

    // generateEpoch() expects the GPS time at the end of the block.
    if (generated_samples == 0) {
      fprintf(stderr, "[TIMING] First RF sample GPS epoch: week %d tow %.9f\n",
              first_sample_gps_time.week, first_sample_gps_time.sec);
      fprintf(stderr, "[TIMING] First generated epoch end: week %d tow %.9f\n",
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
      refreshNavState(chan, eph,
                      has_revive_mode == TRUE ? revive_scan_eph : eph, neph,
                      &ieph, active_eph, &synth_eph, &synth_cfg, &ionoutc, grx,
                      elvmask, trimble_rtcm_mode, &trimble_rtcm_alive,
                      &trimble_rtcm_stream, &attack_cfg, nullptr);

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
    gpstime_t block_start_gps_time = getGpsTimeAtSampleOffset(
        first_sample_gps_time, generated_samples, samp_freq, gps_time_ppm);
    gpstime_t block_end_gps_time = getGpsTimeAtSampleOffset(
        first_sample_gps_time, generated_samples + sample_count, samp_freq,
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
      refreshNavState(chan, eph,
                      has_revive_mode == TRUE ? revive_scan_eph : eph, neph,
                      &ieph, active_eph, &synth_eph, &synth_cfg, &ionoutc, grx,
                      elvmask, trimble_rtcm_mode, &trimble_rtcm_alive,
                      &trimble_rtcm_stream, &attack_cfg, nullptr);

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
            getGpsElapsedFromSamples(emitted_samples, samp_freq, gps_time_ppm));
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
