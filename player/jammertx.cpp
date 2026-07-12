/*
 * jammertx.cpp - Jammer-only real-time transmitter for UHD devices.
 *
 * This program intentionally has no GPS ephemeris, navigation-message, PRN,
 * receiver-location, or GPS-time inputs. It renders the same independent
 * source as tools/jammergen.c and streams only that source to the SDR.
 */

#include <uhd/stream.hpp>
#include <uhd/types/metadata.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/usrp/multi_usrp.hpp>

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
// The shared renderer is sample-count bounded. Continuous live mode uses the
// largest representable interval and exposes it as unbounded in the manifest.
constexpr uint64_t kContinuousSampleLimit =
    std::numeric_limits<uint64_t>::max();

volatile std::sig_atomic_t stop_requested = 0;

struct Options {
  jammer_source_type_t source_type = JAMMER_SOURCE_CW;
  std::string address = "192.168.10.2";
  std::string antenna = "TX/RX";
  std::string clock_source = "internal";
  std::string time_source = "internal";
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
      "Generate and transmit jammer-only complex IQ through a UHD device.\n"
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
      "UHD options:\n"
      "  --addr IP                 Device address (default 192.168.10.2)\n"
      "  --channel N               TX channel (default 0)\n"
      "  --antenna NAME            TX antenna (default TX/RX)\n"
      "  --gain DB                 Explicit TX gain (required for live TX)\n"
      "  --clock-source SOURCE     internal/external/gpsdo (default internal)\n"
      "  --time-source SOURCE      internal/external/gpsdo (default internal)\n"
      "  --start-delay SEC         Timed-start lead (default 0.25, min 0.02)\n"
      "\n"
      "Evidence and safety options:\n"
      "  --manifest FILE           Live-run JSON manifest (required for TX)\n"
      "  --calibration-id TEXT     Conducted/shielded RF calibration "
      "identifier\n"
      "  --confirm-controlled-rf   Required acknowledgement for live TX\n"
      "  --dry-run                 Validate and print the plan; do not open "
      "UHD\n"
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
    } else if (argument == "--addr") {
      options->address = value;
    } else if (argument == "--channel") {
      if (!parse_size(value, &options->channel)) {
        std::fprintf(stderr, "jammertx: invalid channel '%s'\n", value);
        return false;
      }
    } else if (argument == "--antenna") {
      options->antenna = value;
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
    } else if (argument == "--clock-source") {
      options->clock_source = value;
    } else if (argument == "--time-source") {
      options->time_source = value;
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
           << ", \"format\": \"sc16_le\", \"iq_order\": \"IQ\", "
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
  manifest << "  \"rf\": {\"device_address\": \""
           << json_escape(options.address)
           << "\", \"channel\": " << options.channel << ", \"antenna\": \""
           << json_escape(options.antenna)
           << "\", \"requested_center_frequency_hz\": "
           << options.center_frequency_hz
           << ", \"actual_center_frequency_hz\": "
           << result.actual_center_frequency_hz
           << ", \"ideal_reference_start_frequency_hz\": "
           << result.actual_center_frequency_hz + config.frequency_hz
           << ", \"ideal_reference_end_frequency_hz\": "
           << result.actual_center_frequency_hz + config.end_frequency_hz
           << ", \"requested_gain_db\": " << options.gain_db
           << ", \"actual_gain_db\": " << result.actual_gain_db
           << ", \"clock_source\": \"" << json_escape(options.clock_source)
           << "\", \"time_source\": \"" << json_escape(options.time_source)
           << "\"},\n";
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
           << ", \"underflows\": " << result.underflows
           << ", \"sequence_errors\": " << result.sequence_errors
           << ", \"time_errors\": " << result.time_errors
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

void account_async_event(const uhd::async_metadata_t &metadata,
                         RunResult *result, bool *fatal) {
  if (metadata.event_code == uhd::async_metadata_t::EVENT_CODE_UNDERFLOW ||
      metadata.event_code ==
          uhd::async_metadata_t::EVENT_CODE_UNDERFLOW_IN_PACKET) {
    ++result->underflows;
    std::fprintf(stderr, "[UHD] WARNING: TX underflow #%llu\n",
                 static_cast<unsigned long long>(result->underflows));
  } else if (metadata.event_code ==
                 uhd::async_metadata_t::EVENT_CODE_SEQ_ERROR ||
             metadata.event_code ==
                 uhd::async_metadata_t::EVENT_CODE_SEQ_ERROR_IN_BURST) {
    ++result->sequence_errors;
    *fatal = true;
    std::fprintf(stderr, "[UHD] ERROR: TX sequence error #%llu\n",
                 static_cast<unsigned long long>(result->sequence_errors));
  } else if (metadata.event_code ==
             uhd::async_metadata_t::EVENT_CODE_TIME_ERROR) {
    ++result->time_errors;
    *fatal = true;
    std::fprintf(stderr, "[UHD] ERROR: timed TX start was missed\n");
  }
}

void drain_async(const uhd::tx_streamer::sptr &stream, double first_timeout,
                 RunResult *result, bool *fatal) {
  uhd::async_metadata_t metadata;
  double timeout = first_timeout;
  while (stream->recv_async_msg(metadata, timeout)) {
    account_async_event(metadata, result, fatal);
    timeout = 0.0;
  }
}

int run_transmitter(const Options &options,
                    jammer_source_config_t *source_config, RunResult *result) {
  uhd::device_addr_t device_address;
  uhd::usrp::multi_usrp::sptr usrp;
  uhd::tx_streamer::sptr stream;
  jammer_source_t source;
  char source_error[256];
  std::string config_error;
  bool fatal = false;

  device_address["addr"] = options.address;
  std::fprintf(stderr, "[UHD] Opening device at %s\n", options.address.c_str());
  usrp = uhd::usrp::multi_usrp::make(device_address);
  usrp->set_clock_source(options.clock_source);
  usrp->set_time_source(options.time_source);

  if (options.channel >= usrp->get_tx_num_channels()) {
    std::fprintf(stderr,
                 "jammertx: channel %zu is invalid; device reports %zu TX "
                 "channel(s)\n",
                 options.channel, usrp->get_tx_num_channels());
    return 1;
  }

  usrp->set_tx_rate(options.requested_rate_hz, options.channel);
  result->actual_rate_hz = usrp->get_tx_rate(options.channel);
  if (!make_source_config(options, result->actual_rate_hz, source_config,
                          &config_error)) {
    std::fprintf(stderr, "jammertx: actual-rate waveform is invalid: %s\n",
                 config_error.c_str());
    return 1;
  }

  uhd::tune_request_t tune_request(options.center_frequency_hz);
  usrp->set_tx_freq(tune_request, options.channel);
  result->actual_center_frequency_hz = usrp->get_tx_freq(options.channel);

  uhd::gain_range_t gain_range = usrp->get_tx_gain_range(options.channel);
  if (options.gain_db < gain_range.start() ||
      options.gain_db > gain_range.stop()) {
    std::fprintf(stderr,
                 "jammertx: requested gain %.3f dB is outside device range "
                 "[%.3f, %.3f] dB\n",
                 options.gain_db, gain_range.start(), gain_range.stop());
    return 1;
  }
  usrp->set_tx_gain(options.gain_db, options.channel);
  result->actual_gain_db = usrp->get_tx_gain(options.channel);
  usrp->set_tx_antenna(options.antenna, options.channel);

  if (jammer_source_init(&source, source_config, source_error,
                         sizeof(source_error)) != 0) {
    std::fprintf(stderr, "jammertx: %s\n", source_error);
    return 1;
  }

  std::fprintf(stderr, "[UHD] Requested/actual rate: %.6f / %.6f Hz\n",
               options.requested_rate_hz, result->actual_rate_hz);
  std::fprintf(stderr, "[UHD] Requested/actual center: %.3f / %.3f Hz\n",
               options.center_frequency_hz, result->actual_center_frequency_hz);
  std::fprintf(stderr, "[UHD] Requested/actual gain: %.3f / %.3f dB\n",
               options.gain_db, result->actual_gain_db);
  std::fprintf(stderr, "[UHD] Channel/antenna: %zu / %s\n", options.channel,
               usrp->get_tx_antenna(options.channel).c_str());
  std::fprintf(stderr, "[UHD] Clock/time source: %s / %s\n",
               options.clock_source.c_str(), options.time_source.c_str());
  print_plan(options, *source_config, result->actual_center_frequency_hz);

  uhd::stream_args_t stream_args("sc16", "sc16");
  stream_args.channels = {options.channel};
  stream = usrp->get_tx_stream(stream_args);

  const std::size_t max_samples = stream->get_max_num_samps();
  std::vector<int16_t> buffer(max_samples * 2);
  size_t count = jammer_source_render_sc16(&source, buffer.data(), max_samples);
  if (count == 0) {
    std::fprintf(stderr, "jammertx: source produced no samples\n");
    return 1;
  }

  install_signal_handlers();

  uhd::tx_metadata_t metadata;
  metadata.start_of_burst = true;
  metadata.end_of_burst = false;
  metadata.has_time_spec = true;
  metadata.time_spec = uhd::time_spec_t(usrp->get_time_now().get_real_secs() +
                                        options.start_delay_seconds);

  std::fprintf(stderr, "[TX] Timed start in %.3f seconds\n",
               options.start_delay_seconds);

  while (count > 0 && !stop_requested && !fatal) {
    size_t sent_from_buffer = 0;
    double timeout =
        metadata.has_time_spec ? options.start_delay_seconds + 1.0 : 3.0;

    while (sent_from_buffer < count && !stop_requested && !fatal) {
      size_t sent = stream->send(&buffer[sent_from_buffer * 2],
                                 count - sent_from_buffer, metadata, timeout);
      if (sent == 0) {
        std::fprintf(stderr, "[UHD] ERROR: send returned zero samples\n");
        fatal = true;
        break;
      }
      sent_from_buffer += sent;
      result->sent_samples += sent;
      metadata.start_of_burst = false;
      metadata.has_time_spec = false;
      timeout = 3.0;
    }

    drain_async(stream, 0.0, result, &fatal);
    if (!stop_requested && !fatal) {
      count = jammer_source_render_sc16(&source, buffer.data(), max_samples);
    }
  }

  metadata.start_of_burst = false;
  metadata.end_of_burst = true;
  metadata.has_time_spec = false;
  stream->send("", 0, metadata, 3.0);
  drain_async(stream, 0.1, result, &fatal);

  result->interrupted = stop_requested != 0;
  result->completed = jammer_source_done(&source) &&
                      result->sent_samples == source.config.total_samples &&
                      !fatal && result->underflows == 0 &&
                      result->sequence_errors == 0;
  jammer_source_get_metrics(&source, &result->source_metrics);

  if (options.continuous) {
    std::fprintf(stderr,
                 "[TX] Sent %llu samples before operator stop; "
                 "underflows=%llu sequence-errors=%llu time-errors=%llu\n",
                 static_cast<unsigned long long>(result->sent_samples),
                 static_cast<unsigned long long>(result->underflows),
                 static_cast<unsigned long long>(result->sequence_errors),
                 static_cast<unsigned long long>(result->time_errors));
  } else {
    std::fprintf(stderr,
                 "[TX] Sent %llu/%llu samples; underflows=%llu "
                 "sequence-errors=%llu time-errors=%llu\n",
                 static_cast<unsigned long long>(result->sent_samples),
                 static_cast<unsigned long long>(source.config.total_samples),
                 static_cast<unsigned long long>(result->underflows),
                 static_cast<unsigned long long>(result->sequence_errors),
                 static_cast<unsigned long long>(result->time_errors));
  }

  return result->completed ||
                 (result->interrupted && result->underflows == 0 &&
                  result->sequence_errors == 0 && result->time_errors == 0)
             ? 0
             : 1;
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
    std::fprintf(stderr, "[DRY-RUN] UHD was not opened and no samples were "
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
      std::fprintf(stderr, "jammertx: UHD failure: %s\n", exception.what());
      exit_code = 1;
    }
  }

  if (!write_manifest(options, source_config, result)) {
    exit_code = 1;
  }

  return exit_code;
}
