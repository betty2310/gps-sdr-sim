/*
 * jammertx.cpp - Jammer-only real-time transmitter for bladeRF devices.
 *
 * This program intentionally has no GPS ephemeris, navigation-message, PRN,
 * receiver-location, or GPS-time inputs. It renders the same independent
 * source as tools/jammergen.c and streams only that source to the SDR.
 */

#include <libbladeRF.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

extern "C" {
#include "tools/jammer_source.h"
}

namespace {

constexpr double kDefaultRateHz = 2500000.0;
constexpr double kDefaultCenterFrequencyHz = 1575420000.0;
constexpr double kDefaultToneOffsetHz = 500000.0;
constexpr double kDefaultAmplitude = 0.5;
constexpr double kDefaultNoiseAmplitude = 0.15;
constexpr double kDefaultBandwidthHz = 100000.0;
constexpr uint64_t kDefaultSeed = 20260712;
constexpr double kDefaultChirpPeriodSeconds = 0.1;
constexpr double kDefaultPulsePeriodSeconds = 0.01;
constexpr double kDefaultPulseDutyCycle = 0.1;
constexpr double kDefaultPulseRampSeconds = 0.00005;
constexpr double kDefaultRampSeconds = 0.01;
constexpr double kDefaultStartDelaySeconds = 0.25;
constexpr double kMinimumStartDelaySeconds = 0.02;
constexpr unsigned int kNumBuffers = 32;
constexpr unsigned int kSamplesPerBuffer = 32 * 1024;
constexpr unsigned int kNumTransfers = 16;
constexpr unsigned int kStreamTimeoutMs = 3000;
constexpr std::size_t kBladeRfSampleMultiple = 1024;
// The shared renderer is sample-count bounded. Continuous live mode uses the
// largest representable interval and exposes it as unbounded in the manifest.
constexpr uint64_t kContinuousSampleLimit =
    std::numeric_limits<uint64_t>::max();

volatile std::sig_atomic_t stop_requested = 0;

struct Options {
  jammer_source_type_t source_type = JAMMER_SOURCE_CW;
  std::string device;
  std::string manifest_path;
  std::string calibration_id;
  std::size_t channel = 0;
  double requested_rate_hz = kDefaultRateHz;
  double center_frequency_hz = kDefaultCenterFrequencyHz;
  double tone_offset_hz = std::numeric_limits<double>::quiet_NaN();
  double end_frequency_hz = std::numeric_limits<double>::quiet_NaN();
  double bandwidth_hz = kDefaultBandwidthHz;
  double amplitude = std::numeric_limits<double>::quiet_NaN();
  uint64_t seed = kDefaultSeed;
  double phase_rad = 0.0;
  double duration_seconds = -1.0;
  double onset_seconds = 0.0;
  double offset_seconds = std::numeric_limits<double>::quiet_NaN();
  double ramp_seconds = kDefaultRampSeconds;
  double chirp_period_seconds = kDefaultChirpPeriodSeconds;
  double pulse_period_seconds = kDefaultPulsePeriodSeconds;
  double pulse_duty_cycle = kDefaultPulseDutyCycle;
  double pulse_ramp_seconds = kDefaultPulseRampSeconds;
  double gain_db = 0.0;
  double start_delay_seconds = kDefaultStartDelaySeconds;
  bool gain_set = false;
  bool duration_set = false;
  bool continuous = false;
  bool dry_run = false;
  bool controlled_rf_confirmed = false;
};

struct RunResult {
  double actual_rate_hz = 0.0;
  double actual_center_frequency_hz = 0.0;
  double actual_gain_db = 0.0;
  uint64_t sent_samples = 0;
  uint64_t underflows = 0;
  uint64_t sequence_errors = 0;
  uint64_t time_errors = 0;
  uint64_t transport_errors = 0;
  bool completed = false;
  bool interrupted = false;
  bool dry_run = false;
  jammer_source_metrics_t source_metrics{};
};

void signal_handler(int) { stop_requested = 1; }

void install_signal_handlers() {
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  std::signal(SIGPIPE, SIG_IGN);
}

void usage(FILE *stream) {
  std::fprintf(
      stream,
      "Usage: jammertx (--duration SEC | --continuous) [options]\n"
      "\n"
      "Generate and transmit jammer-only complex IQ through a bladeRF device.\n"
      "No RINEX, ephemeris, GPS time, location, PRN, or authentic-GPS input is "
      "used.\n"
      "\n"
      "Waveform options:\n"
      "  --type TYPE               cw, narrowband, wideband, chirp, or pulsed\n"
      "  --rate HZ                 Requested TX rate (default 2500000)\n"
      "  --center-frequency HZ     RF center (default 1575420000)\n"
      "  --tone-offset HZ          Fixed/center/chirp-start offset\n"
      "  --frequency HZ            Alias for --tone-offset\n"
      "  --end-frequency HZ        Chirp end offset (default +500000)\n"
      "  --bandwidth HZ            Narrowband two-sided ENBW (default 100000)\n"
      "  --amplitude VALUE         Source RMS or on-pulse amplitude/full-scale\n"
      "  --seed N                  Noise RNG seed (default 20260712)\n"
      "  --chirp-period SEC        Repeating linear-sweep period (default 0.1)\n"
      "  --pulse-period SEC        Pulse repetition period (default 0.01)\n"
      "  --pulse-duty VALUE        On-time fraction in (0,1] (default 0.1)\n"
      "  --pulse-ramp SEC          Per-pulse rise/fall (default 0.00005)\n"
      "  --phase RAD               Phase at onset (default 0)\n"
      "  --duration SEC            Total finite run duration\n"
      "  --continuous              Transmit from sample zero until SIGINT/SIGTERM\n"
      "  --onset SEC               Finite-run jammer onset (default 0)\n"
      "  --offset SEC              Finite-run jammer stop (default: duration)\n"
      "  --ramp SEC                Global edge ramp (default 0.01; start only in continuous mode)\n"
      "\n"
      "bladeRF options:\n"
      "  --device STRING           libbladeRF device identifier (default: first device)\n"
      "  --channel N               TX channel (bladeRF 1.0 supports only 0)\n"
      "  --gain DB                 Explicit TX gain (required for live TX)\n"
      "  --start-delay SEC         Timed-start lead (default 0.25, min 0.02)\n"
      "\n"
      "Evidence and safety options:\n"
      "  --manifest FILE           Live-run JSON manifest (required for TX)\n"
      "  --calibration-id TEXT     Conducted/shielded RF calibration "
      "identifier\n"
      "  --confirm-controlled-rf   Required acknowledgement for live TX\n"
      "  --dry-run                 Validate and print the plan; do not open "
      "bladeRF\n"
      "  --help                    Show this help\n"
      "\n"
      "Live transmission is for an authorized conducted path or verified "
      "shielded enclosure only.\n");
}

bool parse_double(const char *text, double *value) {
  char *end = nullptr;
  errno = 0;
  *value = std::strtod(text, &end);
  return errno == 0 && end != text && *end == '\0' && std::isfinite(*value);
}

bool parse_size(const char *text, std::size_t *value) {
  char *end = nullptr;
  unsigned long long parsed;
  if (text == nullptr || text[0] == '\0' || text[0] == '-') {
    return false;
  }
  errno = 0;
  parsed = std::strtoull(text, &end, 10);
  if (errno != 0 || end == text || *end != '\0' ||
      parsed > std::numeric_limits<std::size_t>::max()) {
    return false;
  }
  *value = static_cast<std::size_t>(parsed);
  return true;
}

bool parse_uint64(const char *text, uint64_t *value) {
  char *end = nullptr;
  unsigned long long parsed;
  if (text == nullptr || text[0] == '\0' || text[0] == '-') {
    return false;
  }
  errno = 0;
  parsed = std::strtoull(text, &end, 10);
  if (errno != 0 || end == text || *end != '\0') {
    return false;
  }
  *value = static_cast<uint64_t>(parsed);
  return true;
}

bool parse_options(int argc, char **argv, Options *options) {
  for (int index = 1; index < argc; ++index) {
    std::string argument = argv[index];

    if (argument == "--help") {
      usage(stdout);
      std::exit(0);
    }
    if (argument == "--confirm-controlled-rf") {
      options->controlled_rf_confirmed = true;
      continue;
    }
    if (argument == "--dry-run") {
      options->dry_run = true;
      continue;
    }
    if (argument == "--continuous") {
      options->continuous = true;
      continue;
    }
    if (argument.rfind("--", 0) != 0) {
      std::fprintf(stderr, "jammertx: unexpected positional argument '%s'\n",
                   argument.c_str());
      return false;
    }
    if (index + 1 >= argc) {
      std::fprintf(stderr, "jammertx: missing value for '%s'\n",
                   argument.c_str());
      return false;
    }

    const char *value = argv[++index];
    if (argument == "--type") {
      if (jammer_source_parse_type(value, &options->source_type) != 0) {
        std::fprintf(stderr,
                     "jammertx: unsupported source type '%s' (supported: cw, narrowband, wideband, chirp, pulsed)\n",
                     value);
        return false;
      }
    } else if (argument == "--device" || argument == "--addr") {
      options->device = value;
    } else if (argument == "--channel") {
      if (!parse_size(value, &options->channel)) {
        std::fprintf(stderr, "jammertx: invalid channel '%s'\n", value);
        return false;
      }
    } else if (argument == "--rate") {
      if (!parse_double(value, &options->requested_rate_hz)) {
        std::fprintf(stderr, "jammertx: invalid rate '%s'\n", value);
        return false;
      }
    } else if (argument == "--center-frequency") {
      if (!parse_double(value, &options->center_frequency_hz)) {
        std::fprintf(stderr, "jammertx: invalid center frequency '%s'\n",
                     value);
        return false;
      }
    } else if (argument == "--tone-offset" || argument == "--frequency") {
      if (!parse_double(value, &options->tone_offset_hz)) {
        std::fprintf(stderr, "jammertx: invalid frequency offset '%s'\n", value);
        return false;
      }
    } else if (argument == "--end-frequency") {
      if (!parse_double(value, &options->end_frequency_hz)) {
        std::fprintf(stderr, "jammertx: invalid end frequency '%s'\n", value);
        return false;
      }
    } else if (argument == "--bandwidth") {
      if (!parse_double(value, &options->bandwidth_hz)) {
        std::fprintf(stderr, "jammertx: invalid bandwidth '%s'\n", value);
        return false;
      }
    } else if (argument == "--amplitude") {
      if (!parse_double(value, &options->amplitude)) {
        std::fprintf(stderr, "jammertx: invalid amplitude '%s'\n", value);
        return false;
      }
    } else if (argument == "--phase") {
      if (!parse_double(value, &options->phase_rad)) {
        std::fprintf(stderr, "jammertx: invalid phase '%s'\n", value);
        return false;
      }
    } else if (argument == "--seed") {
      if (!parse_uint64(value, &options->seed)) {
        std::fprintf(stderr, "jammertx: invalid seed '%s'\n", value);
        return false;
      }
    } else if (argument == "--chirp-period") {
      if (!parse_double(value, &options->chirp_period_seconds)) {
        std::fprintf(stderr, "jammertx: invalid chirp period '%s'\n", value);
        return false;
      }
    } else if (argument == "--pulse-period") {
      if (!parse_double(value, &options->pulse_period_seconds)) {
        std::fprintf(stderr, "jammertx: invalid pulse period '%s'\n", value);
        return false;
      }
    } else if (argument == "--pulse-duty") {
      if (!parse_double(value, &options->pulse_duty_cycle)) {
        std::fprintf(stderr, "jammertx: invalid pulse duty '%s'\n", value);
        return false;
      }
    } else if (argument == "--pulse-ramp") {
      if (!parse_double(value, &options->pulse_ramp_seconds)) {
        std::fprintf(stderr, "jammertx: invalid pulse ramp '%s'\n", value);
        return false;
      }
    } else if (argument == "--duration") {
      if (!parse_double(value, &options->duration_seconds)) {
        std::fprintf(stderr, "jammertx: invalid duration '%s'\n", value);
        return false;
      }
      options->duration_set = true;
    } else if (argument == "--onset") {
      if (!parse_double(value, &options->onset_seconds)) {
        std::fprintf(stderr, "jammertx: invalid onset '%s'\n", value);
        return false;
      }
    } else if (argument == "--offset") {
      if (!parse_double(value, &options->offset_seconds)) {
        std::fprintf(stderr, "jammertx: invalid offset '%s'\n", value);
        return false;
      }
    } else if (argument == "--ramp") {
      if (!parse_double(value, &options->ramp_seconds)) {
        std::fprintf(stderr, "jammertx: invalid ramp '%s'\n", value);
        return false;
      }
    } else if (argument == "--gain") {
      if (!parse_double(value, &options->gain_db)) {
        std::fprintf(stderr, "jammertx: invalid gain '%s'\n", value);
        return false;
      }
      options->gain_set = true;
    } else if (argument == "--start-delay") {
      if (!parse_double(value, &options->start_delay_seconds)) {
        std::fprintf(stderr, "jammertx: invalid start delay '%s'\n", value);
        return false;
      }
    } else if (argument == "--manifest") {
      options->manifest_path = value;
    } else if (argument == "--calibration-id") {
      options->calibration_id = value;
    } else {
      std::fprintf(stderr, "jammertx: unknown option '%s'\n", argument.c_str());
      return false;
    }
  }
  return true;
}

bool seconds_to_samples(double seconds, double sample_rate_hz,
                        uint64_t *samples) {
  long double exact;
  long double rounded;

  if (!std::isfinite(seconds) || seconds < 0.0 ||
      !std::isfinite(sample_rate_hz) || sample_rate_hz <= 0.0) {
    return false;
  }
  exact = static_cast<long double>(seconds) * sample_rate_hz;
  rounded = std::round(exact);
  if (rounded < 0.0L || rounded > static_cast<long double>(
                                      std::numeric_limits<uint64_t>::max())) {
    return false;
  }
  *samples = static_cast<uint64_t>(rounded);
  return true;
}

bool make_source_config(const Options &options, double sample_rate_hz,
                        jammer_source_config_t *config, std::string *error) {
  double offset_seconds = std::isnan(options.offset_seconds)
                              ? options.duration_seconds
                              : options.offset_seconds;
  double frequency_hz = options.tone_offset_hz;
  double end_frequency_hz = options.end_frequency_hz;
  double amplitude = options.amplitude;
  long double pulse_on_exact;
  char source_error[256];
  jammer_source_t validation_source;

  if (std::isnan(frequency_hz)) {
    if (options.source_type == JAMMER_SOURCE_CHIRP) {
      frequency_hz = -kDefaultToneOffsetHz;
    } else if (options.source_type == JAMMER_SOURCE_WIDEBAND) {
      frequency_hz = 0.0;
    } else {
      frequency_hz = kDefaultToneOffsetHz;
    }
  }
  if (std::isnan(end_frequency_hz)) {
    end_frequency_hz = options.source_type == JAMMER_SOURCE_CHIRP
                           ? kDefaultToneOffsetHz
                           : frequency_hz;
  }
  if (std::isnan(amplitude)) {
    amplitude = options.source_type == JAMMER_SOURCE_NARROWBAND ||
                        options.source_type == JAMMER_SOURCE_WIDEBAND
                    ? kDefaultNoiseAmplitude
                    : kDefaultAmplitude;
  }

  std::memset(config, 0, sizeof(*config));
  config->type = options.source_type;
  config->sample_rate_hz = sample_rate_hz;
  config->frequency_hz = frequency_hz;
  config->end_frequency_hz = end_frequency_hz;
  config->bandwidth_hz = options.bandwidth_hz;
  config->phase_rad = options.phase_rad;
  config->amplitude = amplitude;
  config->seed = options.seed;

  if (options.continuous) {
    config->total_samples = kContinuousSampleLimit;
    config->onset_sample = 0;
    config->offset_sample = kContinuousSampleLimit;
  } else if (!seconds_to_samples(options.duration_seconds, sample_rate_hz,
                                 &config->total_samples) ||
             !seconds_to_samples(options.onset_seconds, sample_rate_hz,
                                 &config->onset_sample) ||
             !seconds_to_samples(offset_seconds, sample_rate_hz,
                                 &config->offset_sample)) {
    *error = "duration or envelope timing cannot be represented as samples";
    return false;
  }
  if (!seconds_to_samples(options.ramp_seconds, sample_rate_hz,
                          &config->ramp_samples) ||
      !seconds_to_samples(options.chirp_period_seconds, sample_rate_hz,
                          &config->sweep_samples) ||
      !seconds_to_samples(options.pulse_period_seconds, sample_rate_hz,
                          &config->pulse_period_samples) ||
      !seconds_to_samples(options.pulse_ramp_seconds, sample_rate_hz,
                          &config->pulse_ramp_samples)) {
    *error = "duration or envelope timing cannot be represented as samples";
    return false;
  }
  pulse_on_exact = static_cast<long double>(config->pulse_period_samples) *
                   options.pulse_duty_cycle;
  if (pulse_on_exact < 1.0L ||
      pulse_on_exact >
          static_cast<long double>(std::numeric_limits<uint64_t>::max())) {
    *error = "pulse duty produces an invalid on-time";
    return false;
  }
  config->pulse_on_samples =
      static_cast<uint64_t>(std::round(pulse_on_exact));

  if (jammer_source_init(&validation_source, config, source_error,
                         sizeof(source_error)) != 0) {
    *error = source_error;
    return false;
  }
  return true;
}

bool validate_options(const Options &options, std::string *error) {
  jammer_source_config_t config;

  if (options.channel != 0) {
    *error = "bladeRF 1.0 supports only TX channel 0";
    return false;
  }
  if (options.requested_rate_hz < 1.0 ||
      options.requested_rate_hz >
          static_cast<double>(std::numeric_limits<unsigned int>::max())) {
    *error = "--rate is outside the libbladeRF sample-rate range";
    return false;
  }
  if (options.center_frequency_hz < 1.0 ||
      options.center_frequency_hz >
          static_cast<double>(
              std::numeric_limits<bladerf_frequency>::max())) {
    *error = "--center-frequency is outside the libbladeRF frequency range";
    return false;
  }
  if (options.gain_db < static_cast<double>(std::numeric_limits<int>::min()) ||
      options.gain_db > static_cast<double>(std::numeric_limits<int>::max())) {
    *error = "--gain cannot be represented by libbladeRF";
    return false;
  }
  if (options.continuous) {
    if (options.duration_set) {
      *error = "--continuous and --duration are mutually exclusive";
      return false;
    }
    if (options.onset_seconds != 0.0 ||
        !std::isnan(options.offset_seconds)) {
      *error = "--continuous requires onset 0 and does not accept --offset";
      return false;
    }
  } else {
    if (!options.duration_set || options.duration_seconds <= 0.0) {
      *error = "specify either a positive --duration or --continuous";
      return false;
    }
  }
  if (options.onset_seconds < 0.0 || options.ramp_seconds < 0.0) {
    *error = "--onset and --ramp must be non-negative";
    return false;
  }
  if (options.chirp_period_seconds <= 0.0 ||
      options.pulse_period_seconds <= 0.0 ||
      options.pulse_duty_cycle <= 0.0 || options.pulse_duty_cycle > 1.0 ||
      options.pulse_ramp_seconds < 0.0) {
    *error =
        "chirp/pulse periods must be positive, pulse duty in (0,1], and pulse ramp non-negative";
    return false;
  }
  if (!std::isnan(options.offset_seconds) && options.offset_seconds < 0.0) {
    *error = "--offset must be non-negative";
    return false;
  }
  if (options.start_delay_seconds < kMinimumStartDelaySeconds) {
    std::ostringstream message;
    message << "--start-delay must be at least " << kMinimumStartDelaySeconds
            << " seconds";
    *error = message.str();
    return false;
  }
  if (!options.dry_run) {
    if (!options.controlled_rf_confirmed) {
      *error = "live TX requires --confirm-controlled-rf";
      return false;
    }
    if (!options.gain_set) {
      *error = "live TX requires an explicit --gain";
      return false;
    }
    if (options.manifest_path.empty()) {
      *error = "live TX requires --manifest FILE";
      return false;
    }
  }
  return make_source_config(options, options.requested_rate_hz, &config, error);
}

std::string json_escape(const std::string &value) {
  std::ostringstream escaped;
  for (unsigned char character : value) {
    switch (character) {
    case '\"':
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

const char *run_status(const RunResult &result) {
  if (result.dry_run) {
    return "dry_run";
  }
  if (result.transport_errors > 0) {
    return "device_error";
  }
  if (result.time_errors > 0) {
    return "time_error";
  }
  if (result.sequence_errors > 0) {
    return "sequence_error";
  }
  if (result.underflows > 0) {
    return "underflow";
  }
  if (result.completed) {
    return "complete";
  }
  if (result.interrupted) {
    return "interrupted";
  }
  return "incomplete";
}

bool write_manifest(const Options &options,
                    const jammer_source_config_t &config,
                    const RunResult &result) {
  if (options.manifest_path.empty()) {
    return true;
  }

  std::ofstream manifest(options.manifest_path,
                         std::ios::out | std::ios::trunc);
  if (!manifest) {
    std::fprintf(stderr, "jammertx: cannot open manifest '%s'\n",
                 options.manifest_path.c_str());
    return false;
  }

  manifest << std::setprecision(17);
  manifest << "{\n";
  manifest << "  \"schema\": \"gps-sdr-sim.jammertx.v2\",\n";
  manifest << "  \"tool\": \"jammertx\",\n";
  manifest << "  \"status\": \"" << run_status(result) << "\",\n";
  manifest << "  \"offline_only\": false,\n";
  manifest << "  \"controlled_rf_only\": true,\n";
  manifest << "  \"controlled_rf_confirmed\": "
           << (options.controlled_rf_confirmed ? "true" : "false") << ",\n";
  manifest << "  \"sample_contract\": {\"requested_rate_hz\": "
           << options.requested_rate_hz
           << ", \"actual_rate_hz\": " << result.actual_rate_hz
           << ", \"format\": \"sc16_q11_le\", \"iq_order\": \"IQ\", "
              "\"continuous\": "
           << (options.continuous ? "true" : "false")
           << ", \"planned_samples\": ";
  if (options.continuous) {
    manifest << "null";
  } else {
    manifest << config.total_samples;
  }
  manifest << "},\n";
  manifest << "  \"waveform\": {\"source_type\": \""
           << jammer_source_type_name(config.type)
           << "\", \"frequency_hz\": " << config.frequency_hz
           << ", \"end_frequency_hz\": " << config.end_frequency_hz
           << ", \"bandwidth_hz\": " << config.bandwidth_hz
           << ", \"phase_at_onset_rad\": " << config.phase_rad
           << ", \"amplitude_full_scale\": " << config.amplitude
           << ", \"seed\": " << config.seed
           << ", \"onset_sample\": " << config.onset_sample
           << ", \"offset_sample\": ";
  if (options.continuous) {
    manifest << "null";
  } else {
    manifest << config.offset_sample;
  }
  manifest << ", \"ramp_samples\": " << config.ramp_samples
           << ", \"sweep_samples\": " << config.sweep_samples
           << ", \"pulse_period_samples\": "
           << config.pulse_period_samples
           << ", \"pulse_on_samples\": " << config.pulse_on_samples
           << ", \"pulse_ramp_samples\": " << config.pulse_ramp_samples
           << "},\n";
  manifest << "  \"rf\": {\"device_identifier\": ";
  if (options.device.empty()) {
    manifest << "null";
  } else {
    manifest << "\"" << json_escape(options.device) << "\"";
  }
  manifest << ", \"channel\": " << options.channel
           << ", \"requested_center_frequency_hz\": "
           << options.center_frequency_hz
           << ", \"actual_center_frequency_hz\": "
           << result.actual_center_frequency_hz
           << ", \"ideal_reference_start_frequency_hz\": "
           << result.actual_center_frequency_hz + config.frequency_hz
           << ", \"ideal_reference_end_frequency_hz\": "
           << result.actual_center_frequency_hz + config.end_frequency_hz
           << ", \"requested_gain_db\": " << options.gain_db
           << ", \"actual_gain_db\": " << result.actual_gain_db
           << "},\n";
  manifest << "  \"timing\": {\"start_delay_s\": "
           << options.start_delay_seconds << "},\n";
  manifest << "  \"calibration\": {\"id\": ";
  if (options.calibration_id.empty()) {
    manifest << "null";
  } else {
    manifest << "\"" << json_escape(options.calibration_id) << "\"";
  }
  manifest << ", \"receiver_input_power_calibrated_by_tool\": false},\n";
  manifest << "  \"measurements\": {\"rendered_samples\": "
           << result.source_metrics.rendered_samples
           << ", \"sent_samples\": " << result.sent_samples
           << ", \"active_plateau_rms_full_scale\": "
           << result.source_metrics.active_plateau_rms_full_scale
           << ", \"peak_component_full_scale\": "
           << result.source_metrics.peak_component_full_scale
           << ", \"clipped_components\": "
           << result.source_metrics.clipped_components
           << ", \"source_on_samples\": "
           << result.source_metrics.source_on_samples
           << ", \"transport_errors\": " << result.transport_errors
           << ", \"underflows\": null"
           << ", \"sequence_errors\": null"
           << ", \"time_errors\": null"
           << ", \"transport_observability\": \"synchronous_call_status_only\""
           << ", \"interrupted\": " << (result.interrupted ? "true" : "false")
           << "}\n";
  manifest << "}\n";

  if (!manifest) {
    std::fprintf(stderr, "jammertx: failed writing manifest '%s'\n",
                 options.manifest_path.c_str());
    return false;
  }
  return true;
}

void print_plan(const Options &options, const jammer_source_config_t &config,
                double actual_center_frequency_hz) {
  std::fprintf(stderr, "[JAMMER] Type:            %s\n",
               jammer_source_type_name(config.type));
  std::fprintf(stderr, "[JAMMER] Sample rate:     %.6f Hz\n",
               config.sample_rate_hz);
  std::fprintf(stderr, "[JAMMER] RF center:       %.3f Hz\n",
               actual_center_frequency_hz);
  if (config.type == JAMMER_SOURCE_WIDEBAND) {
    std::fprintf(stderr,
                 "[JAMMER] Ideal RF span:     %.3f to %.3f Hz (complex Nyquist band)\n",
                 actual_center_frequency_hz - config.sample_rate_hz / 2.0,
                 actual_center_frequency_hz + config.sample_rate_hz / 2.0);
  } else if (config.type == JAMMER_SOURCE_CHIRP) {
    std::fprintf(stderr,
                 "[JAMMER] Chirp RF sweep:    %.3f to %.3f Hz every %.9f s\n",
                 actual_center_frequency_hz + config.frequency_hz,
                 actual_center_frequency_hz + config.end_frequency_hz,
                 static_cast<double>(config.sweep_samples) /
                     config.sample_rate_hz);
  } else {
    std::fprintf(stderr, "[JAMMER] Complex offset:  %+.3f Hz\n",
                 config.frequency_hz);
    std::fprintf(stderr, "[JAMMER] RF reference:    %.3f Hz\n",
                 actual_center_frequency_hz + config.frequency_hz);
  }
  if (config.type == JAMMER_SOURCE_NARROWBAND) {
    std::fprintf(stderr, "[JAMMER] Two-sided ENBW:  %.3f Hz\n",
                 config.bandwidth_hz);
  }
  if (config.type == JAMMER_SOURCE_PULSED) {
    std::fprintf(stderr,
                 "[JAMMER] Pulse: period=%llu on=%llu ramp=%llu samples (duty %.6f)\n",
                 static_cast<unsigned long long>(config.pulse_period_samples),
                 static_cast<unsigned long long>(config.pulse_on_samples),
                 static_cast<unsigned long long>(config.pulse_ramp_samples),
                 static_cast<double>(config.pulse_on_samples) /
                     config.pulse_period_samples);
  }
  std::fprintf(stderr, "[JAMMER] Digital amplitude: %.6f full-scale\n",
               config.amplitude);
  if (options.continuous) {
    std::fprintf(stderr,
                 "[JAMMER] Samples: total=continuous onset=0 "
                 "offset=operator-stop ramp=%llu\n",
                 static_cast<unsigned long long>(config.ramp_samples));
    std::fprintf(stderr,
                 "[JAMMER] Times: duration=continuous onset=0 "
                 "offset=SIGINT/SIGTERM ramp=%.9f s\n",
                 static_cast<double>(config.ramp_samples) /
                     config.sample_rate_hz);
  } else {
    std::fprintf(
        stderr,
        "[JAMMER] Samples: total=%llu onset=%llu offset=%llu ramp=%llu\n",
        static_cast<unsigned long long>(config.total_samples),
        static_cast<unsigned long long>(config.onset_sample),
        static_cast<unsigned long long>(config.offset_sample),
        static_cast<unsigned long long>(config.ramp_samples));
    std::fprintf(
        stderr,
        "[JAMMER] Times: duration=%.9f onset=%.9f offset=%.9f "
        "ramp=%.9f s\n",
        static_cast<double>(config.total_samples) / config.sample_rate_hz,
        static_cast<double>(config.onset_sample) / config.sample_rate_hz,
        static_cast<double>(config.offset_sample) / config.sample_rate_hz,
        static_cast<double>(config.ramp_samples) / config.sample_rate_hz);
  }
  if (!options.calibration_id.empty()) {
    std::fprintf(stderr, "[RF] Calibration ID:    %s\n",
                 options.calibration_id.c_str());
  } else {
    std::fprintf(stderr,
                 "[RF] Calibration ID:    not supplied; digital amplitude is "
                 "not receiver-input J/S\n");
  }
}

int run_transmitter(const Options &options,
                    jammer_source_config_t *source_config, RunResult *result) {
  struct bladerf *device = nullptr;
  jammer_source_t source;
  char source_error[256];
  std::string config_error;
  bool module_enabled = false;
  bool device_error = false;
  int status;

  const char *device_identifier =
      options.device.empty() ? nullptr : options.device.c_str();
  std::fprintf(stderr, "[bladeRF] Opening %s\n",
               device_identifier == nullptr ? "first available device"
                                            : device_identifier);
  status = bladerf_open(&device, device_identifier);
  if (status != 0) {
    std::fprintf(stderr, "jammertx: cannot open bladeRF: %s\n",
                 bladerf_strerror(status));
    return 1;
  }

  auto close_device = [&]() {
    if (module_enabled) {
      bladerf_enable_module(device, BLADERF_MODULE_TX, false);
      module_enabled = false;
    }
    if (device != nullptr) {
      bladerf_close(device);
      device = nullptr;
    }
  };

  struct bladerf_devinfo device_info;
  if (bladerf_get_devinfo(device, &device_info) == 0) {
    std::fprintf(stderr, "[bladeRF] Backend: %s; serial: %s\n",
                 bladerf_backend_str(device_info.backend), device_info.serial);
  }

  unsigned int requested_rate =
      static_cast<unsigned int>(std::llround(options.requested_rate_hz));
  unsigned int actual_rate = 0;
  status = bladerf_set_sample_rate(device, BLADERF_MODULE_TX, requested_rate,
                                   &actual_rate);
  if (status != 0) {
    std::fprintf(stderr, "jammertx: cannot set bladeRF TX rate: %s\n",
                 bladerf_strerror(status));
    close_device();
    return 1;
  }
  result->actual_rate_hz = static_cast<double>(actual_rate);
  if (!make_source_config(options, result->actual_rate_hz, source_config,
                          &config_error)) {
    std::fprintf(stderr, "jammertx: actual-rate waveform is invalid: %s\n",
                 config_error.c_str());
    close_device();
    return 1;
  }

  const bladerf_frequency requested_frequency =
      static_cast<bladerf_frequency>(std::llround(options.center_frequency_hz));
  status = bladerf_set_frequency(device, BLADERF_MODULE_TX,
                                 requested_frequency);
  if (status != 0) {
    std::fprintf(stderr, "jammertx: cannot tune bladeRF TX: %s\n",
                 bladerf_strerror(status));
    close_device();
    return 1;
  }
  bladerf_frequency actual_frequency = 0;
  status = bladerf_get_frequency(device, BLADERF_MODULE_TX, &actual_frequency);
  if (status != 0) {
    std::fprintf(stderr, "jammertx: cannot read bladeRF TX frequency: %s\n",
                 bladerf_strerror(status));
    close_device();
    return 1;
  }
  result->actual_center_frequency_hz =
      static_cast<double>(actual_frequency);

  unsigned int actual_bandwidth = 0;
  status = bladerf_set_bandwidth(device, BLADERF_MODULE_TX, actual_rate,
                                 &actual_bandwidth);
  if (status != 0) {
    std::fprintf(stderr, "jammertx: cannot set bladeRF TX bandwidth: %s\n",
                 bladerf_strerror(status));
    close_device();
    return 1;
  }

  const bladerf_channel tx_channel = BLADERF_CHANNEL_TX(options.channel);
  const int requested_gain = static_cast<int>(std::lround(options.gain_db));
  status = bladerf_set_gain(device, tx_channel, requested_gain);
  if (status != 0) {
    std::fprintf(stderr,
                 "jammertx: cannot set bladeRF TX gain to %d dB: %s\n",
                 requested_gain, bladerf_strerror(status));
    close_device();
    return 1;
  }
  int actual_gain = 0;
  status = bladerf_get_gain(device, tx_channel, &actual_gain);
  if (status != 0) {
    std::fprintf(stderr, "jammertx: cannot read bladeRF TX gain: %s\n",
                 bladerf_strerror(status));
    close_device();
    return 1;
  }
  result->actual_gain_db = static_cast<double>(actual_gain);

  status = bladerf_sync_config(device, BLADERF_TX_X1,
                               BLADERF_FORMAT_SC16_Q11_META, kNumBuffers,
                               kSamplesPerBuffer, kNumTransfers,
                               kStreamTimeoutMs);
  if (status != 0) {
    std::fprintf(stderr, "jammertx: cannot configure bladeRF TX stream: %s\n",
                 bladerf_strerror(status));
    close_device();
    return 1;
  }
  status = bladerf_enable_module(device, BLADERF_MODULE_TX, true);
  if (status != 0) {
    std::fprintf(stderr, "jammertx: cannot enable bladeRF TX: %s\n",
                 bladerf_strerror(status));
    close_device();
    return 1;
  }
  module_enabled = true;

  if (jammer_source_init(&source, source_config, source_error,
                         sizeof(source_error)) != 0) {
    std::fprintf(stderr, "jammertx: %s\n", source_error);
    close_device();
    return 1;
  }

  std::fprintf(stderr,
               "[bladeRF] Requested/actual rate: %.6f / %.6f Hz\n",
               options.requested_rate_hz, result->actual_rate_hz);
  std::fprintf(stderr,
               "[bladeRF] Requested/actual center: %.3f / %.3f Hz\n",
               options.center_frequency_hz,
               result->actual_center_frequency_hz);
  std::fprintf(stderr,
               "[bladeRF] Requested/actual gain: %.3f / %.3f dB\n",
               options.gain_db, result->actual_gain_db);
  std::fprintf(stderr, "[bladeRF] TX bandwidth: %u Hz\n", actual_bandwidth);
  print_plan(options, *source_config, result->actual_center_frequency_hz);

  bladerf_timestamp device_now = 0;
  status = bladerf_get_timestamp(device, BLADERF_TX, &device_now);
  if (status != 0) {
    std::fprintf(stderr, "jammertx: cannot read bladeRF TX timestamp: %s\n",
                 bladerf_strerror(status));
    close_device();
    return 1;
  }
  long double delay_samples_exact =
      static_cast<long double>(options.start_delay_seconds) * actual_rate;
  if (delay_samples_exact < 0.0L ||
      delay_samples_exact >
          static_cast<long double>(
              std::numeric_limits<bladerf_timestamp>::max() - device_now)) {
    std::fprintf(stderr, "jammertx: timed-start delay overflows timestamp\n");
    close_device();
    return 1;
  }
  const bladerf_timestamp start_timestamp =
      device_now + static_cast<bladerf_timestamp>(
                       std::llround(delay_samples_exact));

  std::vector<int16_t> buffer(kSamplesPerBuffer * 2);
  install_signal_handlers();
  bool first_buffer = true;
  std::fprintf(stderr,
               "[TX] bladeRF timed start in %.3f seconds at timestamp %llu\n",
               options.start_delay_seconds,
               static_cast<unsigned long long>(start_timestamp));

  while (!stop_requested && !device_error) {
    const std::size_t count =
        jammer_source_render_sc16(&source, buffer.data(), kSamplesPerBuffer);
    if (count == 0) {
      break;
    }

    for (std::size_t index = 0; index < count * 2; ++index) {
      buffer[index] = static_cast<int16_t>(buffer[index] / 16);
    }
    const std::size_t padded_count =
        ((count + kBladeRfSampleMultiple - 1) / kBladeRfSampleMultiple) *
        kBladeRfSampleMultiple;
    std::fill(buffer.begin() + count * 2,
              buffer.begin() + padded_count * 2, 0);

    struct bladerf_metadata metadata;
    std::memset(&metadata, 0, sizeof(metadata));
    if (first_buffer) {
      metadata.flags = BLADERF_META_FLAG_TX_BURST_START;
      metadata.timestamp = start_timestamp;
    }
    unsigned int timeout_ms = kStreamTimeoutMs;
    if (first_buffer) {
      timeout_ms += static_cast<unsigned int>(
          std::ceil(options.start_delay_seconds * 1000.0));
    }
    status = bladerf_sync_tx(device, buffer.data(),
                             static_cast<unsigned int>(padded_count),
                             &metadata, timeout_ms);
    if (status != 0) {
      std::fprintf(stderr, "[bladeRF] TX failed: %s\n",
                   bladerf_strerror(status));
      ++result->transport_errors;
      device_error = true;
      break;
    }
    first_buffer = false;
    result->sent_samples += count;
  }

  if (!first_buffer) {
    struct bladerf_metadata end_metadata;
    std::memset(&end_metadata, 0, sizeof(end_metadata));
    end_metadata.flags = BLADERF_META_FLAG_TX_BURST_END;
    int16_t zero_sample[2] = {0, 0};
    status = bladerf_sync_tx(device, zero_sample, 1, &end_metadata,
                             kStreamTimeoutMs);
    if (status != 0 && !device_error) {
      std::fprintf(stderr, "[bladeRF] TX burst end failed: %s\n",
                   bladerf_strerror(status));
      ++result->transport_errors;
      device_error = true;
    }
  }

  result->interrupted = stop_requested != 0;
  result->completed = jammer_source_done(&source) &&
                      result->sent_samples == source.config.total_samples &&
                      !device_error;
  jammer_source_get_metrics(&source, &result->source_metrics);

  if (options.continuous) {
    std::fprintf(stderr, "[TX] Sent %llu samples before operator stop\n",
                 static_cast<unsigned long long>(result->sent_samples));
  } else {
    std::fprintf(stderr, "[TX] Sent %llu/%llu samples\n",
                 static_cast<unsigned long long>(result->sent_samples),
                 static_cast<unsigned long long>(source.config.total_samples));
  }

  close_device();
  return result->completed || (result->interrupted && !device_error) ? 0 : 1;
}

} // namespace

int main(int argc, char **argv) {
  Options options;
  RunResult result;
  jammer_source_config_t source_config{};
  std::string error;
  int exit_code = 0;

  if (!parse_options(argc, argv, &options)) {
    usage(stderr);
    return 2;
  }
  if (!validate_options(options, &error)) {
    std::fprintf(stderr, "jammertx: %s\n", error.c_str());
    return 2;
  }
  if (!make_source_config(options, options.requested_rate_hz, &source_config,
                          &error)) {
    std::fprintf(stderr, "jammertx: %s\n", error.c_str());
    return 2;
  }

  if (options.dry_run) {
    result.dry_run = true;
    result.actual_rate_hz = options.requested_rate_hz;
    result.actual_center_frequency_hz = options.center_frequency_hz;
    result.actual_gain_db = options.gain_db;
    print_plan(options, source_config, options.center_frequency_hz);
    std::fprintf(stderr, "[DRY-RUN] bladeRF was not opened and no samples were "
                         "transmitted.\n");
  } else {
    RunResult starting_result;
    starting_result.actual_rate_hz = options.requested_rate_hz;
    starting_result.actual_center_frequency_hz = options.center_frequency_hz;
    starting_result.actual_gain_db = options.gain_db;
    if (!write_manifest(options, source_config, starting_result)) {
      return 1;
    }
    try {
      exit_code = run_transmitter(options, &source_config, &result);
    } catch (const std::exception &exception) {
      std::fprintf(stderr, "jammertx: bladeRF failure: %s\n",
                   exception.what());
      exit_code = 1;
    }
  }

  if (!write_manifest(options, source_config, result)) {
    exit_code = 1;
  }

  return exit_code;
}
