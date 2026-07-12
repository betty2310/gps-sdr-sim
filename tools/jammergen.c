#define _FILE_OFFSET_BITS 64

#include <errno.h>
#include <getopt.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "jammer_source.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define CHUNK_SAMPLES 65536U

typedef struct {
  jammer_source_type_t type;
  const char *output;
  const char *manifest;
  uint64_t sample_rate;
  double duration_s;
  double frequency_hz;
  double end_frequency_hz;
  double bandwidth_hz;
  double phase_rad;
  double amplitude;
  uint64_t seed;
  double onset_s;
  double offset_s;
  double ramp_s;
  double chirp_period_s;
  double pulse_period_s;
  double pulse_duty_cycle;
  double pulse_ramp_s;
} options_t;

static void usage(FILE *stream) {
  fprintf(stream,
          "Usage: jammergen --output FILE [options]\n"
          "Generate offline-only little-endian interleaved int16 complex jammer IQ.\n\n"
          "Options:\n"
          "  --type TYPE           cw, narrowband, wideband, chirp, or pulsed\n"
          "  --manifest FILE       Write JSON metadata\n"
          "  --sample-rate HZ      Complex sample rate (default 2600000)\n"
          "  --duration SEC        Output duration (required)\n"
          "  --frequency HZ        Fixed/center/chirp-start offset\n"
          "  --end-frequency HZ    Chirp end offset (default +500000)\n"
          "  --bandwidth HZ        Narrowband two-sided ENBW (default 100000)\n"
          "  --phase RAD           Phase at onset (default 0)\n"
          "  --amplitude VALUE     Source RMS or on-pulse amplitude/full-scale\n"
          "  --seed N              Noise RNG seed (default 20260712)\n"
          "  --chirp-period SEC    Repeating linear-sweep period (default 0.1)\n"
          "  --pulse-period SEC    Pulse repetition period (default 0.01)\n"
          "  --pulse-duty VALUE    On-time fraction in (0,1] (default 0.1)\n"
          "  --pulse-ramp SEC      Per-pulse rise/fall (default 0.00005)\n"
          "  --onset SEC           Activation time (default 30)\n"
          "  --offset SEC          Deactivation time (default 60)\n"
          "  --ramp SEC            Raised-cosine rise/fall (default 0.01)\n"
          "  --help                Show this help\n");
}

static void apply_type_defaults(options_t *opt) {
  if (isnan(opt->frequency_hz)) {
    if (opt->type == JAMMER_SOURCE_CHIRP) {
      opt->frequency_hz = -500000.0;
    } else if (opt->type == JAMMER_SOURCE_WIDEBAND) {
      opt->frequency_hz = 0.0;
    } else {
      opt->frequency_hz = 500000.0;
    }
  }
  if (isnan(opt->end_frequency_hz)) {
    opt->end_frequency_hz =
        opt->type == JAMMER_SOURCE_CHIRP ? 500000.0 : opt->frequency_hz;
  }
  if (isnan(opt->amplitude)) {
    opt->amplitude =
        opt->type == JAMMER_SOURCE_NARROWBAND ||
                opt->type == JAMMER_SOURCE_WIDEBAND
            ? 0.15
            : 0.5;
  }
}

static int parse_u64(const char *text, uint64_t *value) {
  char *end = NULL;
  unsigned long long parsed;
  errno = 0;
  parsed = strtoull(text, &end, 10);
  if (errno != 0 || end == text || *end != '\0') {
    return -1;
  }
  *value = (uint64_t)parsed;
  return 0;
}

static int parse_double(const char *text, double *value) {
  char *end = NULL;
  errno = 0;
  *value = strtod(text, &end);
  return (errno == 0 && end != text && *end == '\0' && isfinite(*value)) ? 0
                                                                          : -1;
}

static int seconds_to_samples(double seconds, uint64_t sample_rate,
                              uint64_t *samples) {
  long double exact = (long double)seconds * (long double)sample_rate;
  long double rounded = roundl(exact);
  if (seconds < 0.0 || rounded < 0.0L || rounded > (long double)UINT64_MAX ||
      fabsl(exact - rounded) > 1e-6L) {
    return -1;
  }
  *samples = (uint64_t)rounded;
  return 0;
}

static int write_manifest(const options_t *opt, uint64_t total_samples,
                          const jammer_source_config_t *config,
                          const jammer_source_metrics_t *metrics) {
  FILE *fp;
  if (opt->manifest == NULL) {
    return 0;
  }
  fp = fopen(opt->manifest, "w");
  if (fp == NULL) {
    fprintf(stderr, "jammergen: cannot open manifest '%s': %s\n", opt->manifest,
            strerror(errno));
    return -1;
  }
  fprintf(fp,
          "{\n"
          "  \"schema\": \"gps-sdr-sim.jammer-source.v2\",\n"
          "  \"tool\": \"jammergen\",\n"
          "  \"offline_only\": true,\n"
          "  \"source_type\": \"%s\",\n"
          "  \"sample_contract\": {\"sample_rate_hz\": %llu, "
          "\"format\": \"int16_le\", \"iq_order\": \"IQ\", "
          "\"complex_samples\": %llu, \"bytes\": %llu},\n"
          "  \"parameters\": {\"frequency_hz\": %.17g, "
          "\"end_frequency_hz\": %.17g, \"bandwidth_hz\": %.17g, "
          "\"phase_at_onset_rad\": %.17g, \"amplitude_full_scale\": %.17g, "
          "\"seed\": %llu, "
          "\"onset_sample\": %llu, \"offset_sample\": %llu, "
          "\"ramp_samples\": %llu, \"sweep_samples\": %llu, "
          "\"pulse_period_samples\": %llu, \"pulse_on_samples\": %llu, "
          "\"pulse_ramp_samples\": %llu},\n"
          "  \"measurements\": {\"active_plateau_rms_full_scale\": %.17g, "
          "\"peak_component_full_scale\": %.17g, "
          "\"clipped_components\": %llu, \"plateau_samples\": %llu, "
          "\"source_on_samples\": %llu}\n"
          "}\n",
          jammer_source_type_name(opt->type),
          (unsigned long long)opt->sample_rate,
          (unsigned long long)total_samples,
          (unsigned long long)(total_samples * 4ULL), opt->frequency_hz,
          opt->end_frequency_hz, opt->bandwidth_hz, opt->phase_rad,
          opt->amplitude, (unsigned long long)opt->seed,
          (unsigned long long)config->onset_sample,
          (unsigned long long)config->offset_sample,
          (unsigned long long)config->ramp_samples,
          (unsigned long long)config->sweep_samples,
          (unsigned long long)config->pulse_period_samples,
          (unsigned long long)config->pulse_on_samples,
          (unsigned long long)config->pulse_ramp_samples,
          metrics->active_plateau_rms_full_scale,
          metrics->peak_component_full_scale,
          (unsigned long long)metrics->clipped_components,
          (unsigned long long)metrics->plateau_samples,
          (unsigned long long)metrics->source_on_samples);
  if (fclose(fp) != 0) {
    fprintf(stderr, "jammergen: failed writing manifest '%s': %s\n",
            opt->manifest, strerror(errno));
    return -1;
  }
  return 0;
}

int main(int argc, char **argv) {
  options_t opt = {.type = JAMMER_SOURCE_CW,
                   .output = NULL,
                   .manifest = NULL,
                   .sample_rate = 2600000,
                   .duration_s = -1.0,
                   .frequency_hz = NAN,
                   .end_frequency_hz = NAN,
                   .bandwidth_hz = 100000.0,
                   .phase_rad = 0.0,
                   .amplitude = NAN,
                   .seed = 20260712,
                   .onset_s = 30.0,
                   .offset_s = 60.0,
                   .ramp_s = 0.01,
                   .chirp_period_s = 0.1,
                   .pulse_period_s = 0.01,
                   .pulse_duty_cycle = 0.1,
                   .pulse_ramp_s = 0.00005};
  static const struct option long_options[] = {
      {"type", required_argument, NULL, 't'},
      {"output", required_argument, NULL, 'o'},
      {"manifest", required_argument, NULL, 'm'},
      {"sample-rate", required_argument, NULL, 's'},
      {"duration", required_argument, NULL, 'd'},
      {"frequency", required_argument, NULL, 'f'},
      {"end-frequency", required_argument, NULL, 'e'},
      {"bandwidth", required_argument, NULL, 'b'},
      {"phase", required_argument, NULL, 'p'},
      {"amplitude", required_argument, NULL, 'a'},
      {"seed", required_argument, NULL, 'S'},
      {"chirp-period", required_argument, NULL, 'c'},
      {"pulse-period", required_argument, NULL, 'P'},
      {"pulse-duty", required_argument, NULL, 'u'},
      {"pulse-ramp", required_argument, NULL, 'q'},
      {"onset", required_argument, NULL, 'n'},
      {"offset", required_argument, NULL, 'x'},
      {"ramp", required_argument, NULL, 'r'},
      {"help", no_argument, NULL, 'h'},
      {NULL, 0, NULL, 0}};
  uint64_t total_samples, onset, offset, ramp, sweep, pulse_period, pulse_on,
      pulse_ramp;
  long double pulse_on_exact;
  int16_t *buffer = NULL;
  FILE *out = NULL;
  jammer_source_config_t source_config;
  jammer_source_metrics_t metrics;
  jammer_source_t source;
  char source_error[256];
  int c;

  while ((c = getopt_long(argc, argv, "", long_options, NULL)) != -1) {
    switch (c) {
    case 't':
      if (jammer_source_parse_type(optarg, &opt.type) != 0) {
        fprintf(stderr,
                "jammergen: unsupported source type '%s' (supported: cw, narrowband, wideband, chirp, pulsed)\n",
                optarg);
        return 2;
      }
      break;
    case 'o': opt.output = optarg; break;
    case 'm': opt.manifest = optarg; break;
    case 's':
      if (parse_u64(optarg, &opt.sample_rate) != 0) {
        fprintf(stderr, "jammergen: invalid sample rate '%s'\n", optarg);
        return 2;
      }
      break;
    case 'd': if (parse_double(optarg, &opt.duration_s) != 0) return 2; break;
    case 'f': if (parse_double(optarg, &opt.frequency_hz) != 0) return 2; break;
    case 'e':
      if (parse_double(optarg, &opt.end_frequency_hz) != 0) return 2;
      break;
    case 'b': if (parse_double(optarg, &opt.bandwidth_hz) != 0) return 2; break;
    case 'p': if (parse_double(optarg, &opt.phase_rad) != 0) return 2; break;
    case 'a': if (parse_double(optarg, &opt.amplitude) != 0) return 2; break;
    case 'S':
      if (parse_u64(optarg, &opt.seed) != 0) {
        fprintf(stderr, "jammergen: invalid seed '%s'\n", optarg);
        return 2;
      }
      break;
    case 'c':
      if (parse_double(optarg, &opt.chirp_period_s) != 0) return 2;
      break;
    case 'P':
      if (parse_double(optarg, &opt.pulse_period_s) != 0) return 2;
      break;
    case 'u':
      if (parse_double(optarg, &opt.pulse_duty_cycle) != 0) return 2;
      break;
    case 'q': if (parse_double(optarg, &opt.pulse_ramp_s) != 0) return 2; break;
    case 'n': if (parse_double(optarg, &opt.onset_s) != 0) return 2; break;
    case 'x': if (parse_double(optarg, &opt.offset_s) != 0) return 2; break;
    case 'r': if (parse_double(optarg, &opt.ramp_s) != 0) return 2; break;
    case 'h': usage(stdout); return 0;
    default: usage(stderr); return 2;
    }
  }

  apply_type_defaults(&opt);
  if (opt.output == NULL || opt.duration_s <= 0.0 || opt.sample_rate == 0 ||
      opt.amplitude <= 0.0 || opt.amplitude > 1.0 || opt.ramp_s < 0.0 ||
      opt.chirp_period_s <= 0.0 || opt.pulse_period_s <= 0.0 ||
      opt.pulse_duty_cycle <= 0.0 || opt.pulse_duty_cycle > 1.0 ||
      opt.pulse_ramp_s < 0.0) {
    fprintf(stderr, "jammergen: invalid or missing output/waveform argument\n");
    usage(stderr);
    return 2;
  }
  if (seconds_to_samples(opt.duration_s, opt.sample_rate, &total_samples) != 0 ||
      seconds_to_samples(opt.onset_s, opt.sample_rate, &onset) != 0 ||
      seconds_to_samples(opt.offset_s, opt.sample_rate, &offset) != 0 ||
      seconds_to_samples(opt.ramp_s, opt.sample_rate, &ramp) != 0 ||
      seconds_to_samples(opt.chirp_period_s, opt.sample_rate, &sweep) != 0 ||
      seconds_to_samples(opt.pulse_period_s, opt.sample_rate, &pulse_period) !=
          0 ||
      seconds_to_samples(opt.pulse_ramp_s, opt.sample_rate, &pulse_ramp) != 0 ||
      onset >= offset || offset > total_samples || ramp * 2 > offset - onset) {
    fprintf(stderr, "jammergen: timing must be sample-aligned and satisfy "
                    "0 <= onset < offset <= duration with two ramps fitting\n");
    return 2;
  }
  pulse_on_exact =
      (long double)pulse_period * (long double)opt.pulse_duty_cycle;
  if (pulse_on_exact < 1.0L || pulse_on_exact > (long double)UINT64_MAX) {
    fprintf(stderr, "jammergen: pulse duty produces an invalid on-time\n");
    return 2;
  }
  pulse_on = (uint64_t)roundl(pulse_on_exact);
  if (total_samples > UINT64_MAX / 4ULL) {
    fprintf(stderr, "jammergen: output size exceeds the manifest range\n");
    return 2;
  }

  memset(&source_config, 0, sizeof(source_config));
  source_config.type = opt.type;
  source_config.sample_rate_hz = (double)opt.sample_rate;
  source_config.total_samples = total_samples;
  source_config.frequency_hz = opt.frequency_hz;
  source_config.end_frequency_hz = opt.end_frequency_hz;
  source_config.bandwidth_hz = opt.bandwidth_hz;
  source_config.phase_rad = opt.phase_rad;
  source_config.amplitude = opt.amplitude;
  source_config.seed = opt.seed;
  source_config.onset_sample = onset;
  source_config.offset_sample = offset;
  source_config.ramp_samples = ramp;
  source_config.sweep_samples = sweep;
  source_config.pulse_period_samples = pulse_period;
  source_config.pulse_on_samples = pulse_on;
  source_config.pulse_ramp_samples = pulse_ramp;
  if (jammer_source_init(&source, &source_config, source_error,
                          sizeof(source_error)) != 0) {
    fprintf(stderr, "jammergen: %s\n", source_error);
    return 2;
  }

  out = fopen(opt.output, "wb");
  if (out == NULL) {
    fprintf(stderr, "jammergen: cannot open output '%s': %s\n", opt.output,
            strerror(errno));
    return 1;
  }

  buffer = malloc(CHUNK_SAMPLES * 2U * sizeof(*buffer));
  if (buffer == NULL) {
    fprintf(stderr, "jammergen: out of memory\n");
    fclose(out);
    return 1;
  }

  while (!jammer_source_done(&source)) {
    size_t count =
        jammer_source_render_sc16(&source, buffer, CHUNK_SAMPLES);
    if (fwrite(buffer, 2U * sizeof(*buffer), count, out) != count) {
      fprintf(stderr, "jammergen: failed writing '%s': %s\n", opt.output,
              strerror(errno));
      free(buffer);
      fclose(out);
      return 1;
    }
  }

  free(buffer);
  if (fclose(out) != 0) {
    fprintf(stderr, "jammergen: failed closing '%s': %s\n", opt.output,
            strerror(errno));
    return 1;
  }
  jammer_source_get_metrics(&source, &metrics);
  if (metrics.plateau_samples == 0 ||
      write_manifest(&opt, total_samples, &source_config, &metrics) != 0) {
    return 1;
  }
  return 0;
}
