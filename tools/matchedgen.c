#define _FILE_OFFSET_BITS 64

#include <errno.h>
#include <getopt.h>
#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "gps_ca.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MAX_TARGETS 32

typedef struct {
  uint64_t sample_offset;
  double code_phase_chips;
  double carrier_doppler_hz;
  double code_rate_chips_per_s;
  int clean_gain;
} trajectory_record_t;

typedef struct {
  int prn;
  trajectory_record_t *records;
  size_t record_count;
  size_t record_capacity;
  size_t cursor;
  int8_t code[GPS_CA_CHIPS];
  double code_phase;
  double osc_i;
  double osc_q;
  double step_i;
  double step_q;
  double initial_phase_rad;
} component_t;

typedef struct {
  const char *output;
  const char *manifest;
  const char *trajectory;
  uint64_t sample_rate;
  double duration_s;
  double onset_s;
  double offset_s;
  double ramp_s;
  double amplitude;
  uint64_t phase_seed;
  size_t chunk_samples;
  int target_prns[MAX_TARGETS];
  size_t target_count;
} options_t;

static void usage(FILE *stream) {
  fprintf(stream,
          "Usage: matchedgen --output FILE --trajectory FILE --target-prns LIST [options]\n"
          "Generate offline-only navigation-data-free GPS L1 C/A matched-code IQ.\n\n"
          "  --manifest FILE       Write JSON source manifest\n"
          "  --sample-rate HZ      Complex sample rate (default 2600000)\n"
          "  --duration SEC        Output duration (required)\n"
          "  --onset SEC           Activation time (default 30)\n"
          "  --offset SEC          Deactivation time (default 60)\n"
          "  --ramp SEC            Raised-cosine rise/fall (default 0.01)\n"
          "  --amplitude VALUE     Requested total plateau RMS/full-scale (default 0.25)\n"
          "  --phase-seed N        Deterministic independent carrier phases\n"
          "  --chunk-samples N     Render buffer size (default 65536)\n");
}

static int parse_u64(const char *text, uint64_t *value) {
  char *end = NULL;
  unsigned long long parsed;
  errno = 0;
  parsed = strtoull(text, &end, 10);
  if (errno != 0 || end == text || *end != '\0')
    return -1;
  *value = (uint64_t)parsed;
  return 0;
}

static int parse_double(const char *text, double *value) {
  char *end = NULL;
  errno = 0;
  *value = strtod(text, &end);
  return errno == 0 && end != text && *end == '\0' && isfinite(*value) ? 0 : -1;
}

static int seconds_to_samples(double seconds, uint64_t sample_rate,
                              uint64_t *samples) {
  long double exact = (long double)seconds * (long double)sample_rate;
  long double rounded = roundl(exact);
  if (seconds < 0.0 || rounded < 0.0L || rounded > (long double)UINT64_MAX ||
      fabsl(exact - rounded) > 1e-6L)
    return -1;
  *samples = (uint64_t)rounded;
  return 0;
}

static int parse_targets(const char *text, options_t *opt) {
  const char *cursor = text;

  while (*cursor != '\0') {
    char *end = NULL;
    long prn;
    size_t i;

    errno = 0;
    prn = strtol(cursor, &end, 10);
    if (errno != 0 || end == cursor || prn < 1 || prn > 32 ||
        opt->target_count >= MAX_TARGETS)
      return -1;
    for (i = 0; i < opt->target_count; ++i)
      if (opt->target_prns[i] == (int)prn)
        return -1;
    opt->target_prns[opt->target_count++] = (int)prn;
    if (*end == '\0')
      break;
    if (*end != ',')
      return -1;
    cursor = end + 1;
    if (*cursor == '\0')
      return -1;
  }
  return opt->target_count > 0 ? 0 : -1;
}

static component_t *find_component(component_t *components, size_t count,
                                   int prn) {
  size_t i;
  for (i = 0; i < count; ++i)
    if (components[i].prn == prn)
      return &components[i];
  return NULL;
}

static int append_record(component_t *component,
                         const trajectory_record_t *record) {
  if (component->record_count == component->record_capacity) {
    size_t capacity = component->record_capacity == 0
                          ? 64
                          : component->record_capacity * 2;
    trajectory_record_t *records =
        (trajectory_record_t *)realloc(component->records,
                                       capacity * sizeof(*records));
    if (records == NULL)
      return -1;
    component->records = records;
    component->record_capacity = capacity;
  }
  component->records[component->record_count++] = *record;
  return 0;
}

static int load_trajectory(const options_t *opt, component_t *components,
                           uint64_t total_samples, uint64_t *cadence_out) {
  FILE *fp = fopen(opt->trajectory, "r");
  char line[2048];
  double trajectory_rate = 0.0;
  uint64_t cadence = 0;
  int saw_schema = 0;
  size_t i;

  if (fp == NULL) {
    fprintf(stderr, "matchedgen: cannot open trajectory '%s': %s\n",
            opt->trajectory, strerror(errno));
    return -1;
  }
  while (fgets(line, sizeof(line), fp) != NULL) {
    trajectory_record_t record;
    int prn;
    component_t *component;

    if (strncmp(line, "# schema=gps-sdr-sim.target-trajectory.v1", 41) == 0) {
      saw_schema = 1;
      continue;
    }
    if (sscanf(line, "# sample_rate_hz=%lf", &trajectory_rate) == 1)
      continue;
    if (sscanf(line, "# epoch_cadence_samples=%" SCNu64, &cadence) == 1)
      continue;
    if (line[0] == '#' || strncmp(line, "sample_offset,", 14) == 0 ||
        line[0] == '\n' || line[0] == '\r')
      continue;
    if (sscanf(line, "%" SCNu64 ",%d,%lf,%lf,%lf,%d",
               &record.sample_offset, &prn, &record.code_phase_chips,
               &record.carrier_doppler_hz, &record.code_rate_chips_per_s,
               &record.clean_gain) != 6) {
      fprintf(stderr, "matchedgen: malformed trajectory row: %s", line);
      fclose(fp);
      return -1;
    }
    component = find_component(components, opt->target_count, prn);
    if (component == NULL)
      continue;
    if (!isfinite(record.code_phase_chips) ||
        record.code_phase_chips < 0.0 ||
        record.code_phase_chips >= GPS_CA_CHIPS ||
        !isfinite(record.carrier_doppler_hz) ||
        fabs(record.carrier_doppler_hz) >= (double)opt->sample_rate / 2.0 ||
        !isfinite(record.code_rate_chips_per_s) ||
        record.code_rate_chips_per_s <= 0.0 || record.clean_gain <= 0 ||
        append_record(component, &record) != 0) {
      fprintf(stderr, "matchedgen: invalid trajectory record for PRN %d\n", prn);
      fclose(fp);
      return -1;
    }
  }
  if (ferror(fp)) {
    fprintf(stderr, "matchedgen: failed reading trajectory '%s'\n",
            opt->trajectory);
    fclose(fp);
    return -1;
  }
  fclose(fp);

  if (!saw_schema || cadence == 0 ||
      fabs(trajectory_rate - (double)opt->sample_rate) > 0.5) {
    fprintf(stderr, "matchedgen: trajectory schema/sample contract mismatch\n");
    return -1;
  }
  for (i = 0; i < opt->target_count; ++i) {
    size_t k;
    component_t *component = &components[i];
    if (component->record_count == 0 ||
        component->records[0].sample_offset != 0) {
      fprintf(stderr, "matchedgen: missing sample-zero trajectory for PRN %d\n",
              component->prn);
      return -1;
    }
    for (k = 0; k < component->record_count; ++k) {
      if (component->records[k].sample_offset != (uint64_t)k * cadence) {
        fprintf(stderr,
                "matchedgen: discontinuous trajectory for PRN %d at record %zu\n",
                component->prn, k);
        return -1;
      }
    }
    if (component->records[component->record_count - 1].sample_offset +
            cadence <
        total_samples) {
      fprintf(stderr, "matchedgen: trajectory ends early for PRN %d\n",
              component->prn);
      return -1;
    }
  }
  *cadence_out = cadence;
  return 0;
}

static uint64_t splitmix64(uint64_t *state) {
  uint64_t z = (*state += UINT64_C(0x9e3779b97f4a7c15));
  z = (z ^ (z >> 30)) * UINT64_C(0xbf58476d1ce4e5b9);
  z = (z ^ (z >> 27)) * UINT64_C(0x94d049bb133111eb);
  return z ^ (z >> 31);
}

static void set_frequency(component_t *component, double sample_rate) {
  double radians = 2.0 * M_PI *
                   component->records[component->cursor].carrier_doppler_hz /
                   sample_rate;
  component->step_i = cos(radians);
  component->step_q = sin(radians);
}

static double envelope(uint64_t sample, uint64_t onset, uint64_t offset,
                       uint64_t ramp) {
  if (sample < onset || sample >= offset)
    return 0.0;
  if (ramp > 0 && sample < onset + ramp) {
    double x = (double)(sample - onset) / (double)ramp;
    return 0.5 - 0.5 * cos(M_PI * x);
  }
  if (ramp > 0 && sample >= offset - ramp) {
    double x = (double)(offset - sample) / (double)ramp;
    return 0.5 - 0.5 * cos(M_PI * x);
  }
  return 1.0;
}

static void json_string(FILE *fp, const char *value) {
  const unsigned char *p = (const unsigned char *)value;
  fputc('"', fp);
  for (; *p != '\0'; ++p) {
    if (*p == '"' || *p == '\\') {
      fputc('\\', fp);
      fputc((int)*p, fp);
    } else if (*p < 0x20) {
      fprintf(fp, "\\u%04x", (unsigned int)*p);
    } else {
      fputc((int)*p, fp);
    }
  }
  fputc('"', fp);
}

static int write_manifest(const options_t *opt, const component_t *components,
                          uint64_t total_samples, uint64_t onset,
                          uint64_t offset, uint64_t ramp, uint64_t cadence,
                          double measured_rms, double peak,
                          uint64_t clipped_components,
                          uint64_t plateau_samples) {
  FILE *fp;
  size_t i;

  if (opt->manifest == NULL)
    return 0;
  fp = fopen(opt->manifest, "w");
  if (fp == NULL) {
    fprintf(stderr, "matchedgen: cannot open manifest '%s': %s\n",
            opt->manifest, strerror(errno));
    return -1;
  }
  fprintf(fp,
          "{\n  \"schema\": \"gps-sdr-sim.jammer-source.v3\",\n"
          "  \"tool\": \"matchedgen\",\n  \"offline_only\": true,\n"
          "  \"source_type\": \"matched-code\",\n"
          "  \"interference_taxonomy\": \"navigation-data-free-gps-l1-ca\",\n"
          "  \"sample_contract\": {\"sample_rate_hz\": %" PRIu64
          ", \"format\": \"int16_le\", \"iq_order\": \"IQ\", "
          "\"complex_samples\": %" PRIu64 ", \"bytes\": %" PRIu64 "},\n"
          "  \"parameters\": {\"target_prns\": [",
          opt->sample_rate, total_samples, total_samples * 4);
  for (i = 0; i < opt->target_count; ++i)
    fprintf(fp, "%s%d", i == 0 ? "" : ", ", opt->target_prns[i]);
  fprintf(fp,
          "], \"data_symbol_policy\": \"constant_positive\", "
          "\"phase_seed\": %" PRIu64 ", \"amplitude_full_scale\": %.17g, "
          "\"component_weight\": %.17g, \"onset_sample\": %" PRIu64
          ", \"offset_sample\": %" PRIu64 ", \"ramp_samples\": %" PRIu64
          ", \"epoch_cadence_samples\": %" PRIu64 ", \"trajectory_path\": ",
          opt->phase_seed, opt->amplitude,
          1.0 / sqrt((double)opt->target_count), onset, offset, ramp, cadence);
  json_string(fp, opt->trajectory);
  fprintf(fp, "},\n  \"components\": [\n");
  for (i = 0; i < opt->target_count; ++i) {
    const trajectory_record_t *first = &components[i].records[0];
    fprintf(fp,
            "    {\"prn\": %d, \"initial_carrier_phase_rad\": %.17g, "
            "\"initial_code_phase_chips\": %.17g, "
            "\"initial_carrier_doppler_hz\": %.17g, "
            "\"initial_code_rate_chips_per_s\": %.17g, "
            "\"clean_gain\": %d, \"weight\": %.17g, "
            "\"nominal_component_rms_full_scale\": %.17g}%s\n",
            components[i].prn, components[i].initial_phase_rad,
            first->code_phase_chips, first->carrier_doppler_hz,
            first->code_rate_chips_per_s, first->clean_gain,
            1.0 / sqrt((double)opt->target_count),
            opt->amplitude / sqrt((double)opt->target_count),
            i + 1 == opt->target_count ? "" : ",");
  }
  fprintf(fp,
          "  ],\n  \"measurements\": {\"active_plateau_rms_full_scale\": %.17g, "
          "\"peak_component_full_scale\": %.17g, "
          "\"clipped_components\": %" PRIu64 ", "
          "\"plateau_samples\": %" PRIu64 "}\n}\n",
          measured_rms, peak, clipped_components, plateau_samples);
  if (fclose(fp) != 0) {
    fprintf(stderr, "matchedgen: failed writing manifest '%s'\n", opt->manifest);
    return -1;
  }
  return 0;
}

int main(int argc, char **argv) {
  options_t opt = {.sample_rate = 2600000,
                   .duration_s = -1.0,
                   .onset_s = 30.0,
                   .offset_s = 60.0,
                   .ramp_s = 0.01,
                   .amplitude = 0.25,
                   .phase_seed = 20260713,
                   .chunk_samples = 65536};
  static const struct option long_options[] = {
      {"output", required_argument, NULL, 'o'},
      {"manifest", required_argument, NULL, 'm'},
      {"trajectory", required_argument, NULL, 'j'},
      {"target-prns", required_argument, NULL, 'p'},
      {"sample-rate", required_argument, NULL, 's'},
      {"duration", required_argument, NULL, 'd'},
      {"onset", required_argument, NULL, 'n'},
      {"offset", required_argument, NULL, 'x'},
      {"ramp", required_argument, NULL, 'r'},
      {"amplitude", required_argument, NULL, 'a'},
      {"phase-seed", required_argument, NULL, 'S'},
      {"chunk-samples", required_argument, NULL, 'c'},
      {"help", no_argument, NULL, 'h'},
      {NULL, 0, NULL, 0}};
  component_t components[MAX_TARGETS];
  uint64_t total_samples;
  uint64_t onset;
  uint64_t offset;
  uint64_t ramp;
  uint64_t cadence;
  int16_t *buffer = NULL;
  FILE *out = NULL;
  long double plateau_power = 0.0L;
  uint64_t plateau_samples = 0;
  uint64_t clipped_components = 0;
  double peak = 0.0;
  uint64_t next_sample = 0;
  int c;
  size_t i;
  int status = 1;

  memset(components, 0, sizeof(components));
  while ((c = getopt_long(argc, argv, "", long_options, NULL)) != -1) {
    uint64_t parsed;
    switch (c) {
    case 'o': opt.output = optarg; break;
    case 'm': opt.manifest = optarg; break;
    case 'j': opt.trajectory = optarg; break;
    case 'p':
      if (parse_targets(optarg, &opt) != 0) {
        fprintf(stderr, "matchedgen: target PRNs must be ordered, unique values in 1..32\n");
        return 2;
      }
      break;
    case 's':
      if (parse_u64(optarg, &opt.sample_rate) != 0) return 2;
      break;
    case 'd': if (parse_double(optarg, &opt.duration_s) != 0) return 2; break;
    case 'n': if (parse_double(optarg, &opt.onset_s) != 0) return 2; break;
    case 'x': if (parse_double(optarg, &opt.offset_s) != 0) return 2; break;
    case 'r': if (parse_double(optarg, &opt.ramp_s) != 0) return 2; break;
    case 'a': if (parse_double(optarg, &opt.amplitude) != 0) return 2; break;
    case 'S': if (parse_u64(optarg, &opt.phase_seed) != 0) return 2; break;
    case 'c':
      if (parse_u64(optarg, &parsed) != 0 || parsed == 0 || parsed > SIZE_MAX)
        return 2;
      opt.chunk_samples = (size_t)parsed;
      break;
    case 'h': usage(stdout); return 0;
    default: usage(stderr); return 2;
    }
  }
  if (opt.output == NULL || opt.trajectory == NULL || opt.target_count == 0 ||
      opt.sample_rate == 0 || opt.duration_s <= 0.0 || opt.amplitude <= 0.0 ||
      opt.amplitude > 1.0 || opt.ramp_s < 0.0 ||
      seconds_to_samples(opt.duration_s, opt.sample_rate, &total_samples) != 0 ||
      seconds_to_samples(opt.onset_s, opt.sample_rate, &onset) != 0 ||
      seconds_to_samples(opt.offset_s, opt.sample_rate, &offset) != 0 ||
      seconds_to_samples(opt.ramp_s, opt.sample_rate, &ramp) != 0 ||
      onset >= offset || offset > total_samples || ramp * 2 >= offset - onset) {
    fprintf(stderr, "matchedgen: invalid or missing waveform argument\n");
    usage(stderr);
    return 2;
  }

  for (i = 0; i < opt.target_count; ++i) {
    uint64_t state = opt.phase_seed ^
                     ((uint64_t)opt.target_prns[i] * UINT64_C(0xd1b54a32d192ed03));
    components[i].prn = opt.target_prns[i];
    components[i].initial_phase_rad =
        2.0 * M_PI * (double)(splitmix64(&state) >> 11) / 9007199254740992.0;
    if (gps_ca_generate_bipolar(components[i].prn, components[i].code) != 0)
      goto cleanup;
  }
  if (load_trajectory(&opt, components, total_samples, &cadence) != 0)
    goto cleanup;
  for (i = 0; i < opt.target_count; ++i) {
    components[i].code_phase = components[i].records[0].code_phase_chips;
    components[i].osc_i = cos(components[i].initial_phase_rad);
    components[i].osc_q = sin(components[i].initial_phase_rad);
    set_frequency(&components[i], (double)opt.sample_rate);
  }

  out = fopen(opt.output, "wb");
  if (out == NULL) {
    fprintf(stderr, "matchedgen: cannot open output '%s': %s\n", opt.output,
            strerror(errno));
    goto cleanup;
  }
  buffer = (int16_t *)malloc(opt.chunk_samples * 2 * sizeof(*buffer));
  if (buffer == NULL) {
    fprintf(stderr, "matchedgen: out of memory\n");
    goto cleanup;
  }

  while (next_sample < total_samples) {
    size_t count = (uint64_t)opt.chunk_samples < total_samples - next_sample
                       ? opt.chunk_samples
                       : (size_t)(total_samples - next_sample);
    size_t k;
    for (k = 0; k < count; ++k) {
      uint64_t sample = next_sample + (uint64_t)k;
      double sum_i = 0.0;
      double sum_q = 0.0;
      double weight = 1.0 / sqrt((double)opt.target_count);
      double env = envelope(sample, onset, offset, ramp);
      double raw_i;
      double raw_q;
      long qi;
      long qq;

      for (i = 0; i < opt.target_count; ++i) {
        component_t *component = &components[i];
        const trajectory_record_t *record;
        int chip;
        double next_i;
        double next_q;

        while (component->cursor + 1 < component->record_count &&
               component->records[component->cursor + 1].sample_offset <= sample) {
          ++component->cursor;
          component->code_phase =
              component->records[component->cursor].code_phase_chips;
          set_frequency(component, (double)opt.sample_rate);
        }
        record = &component->records[component->cursor];
        chip = component->code[(int)floor(component->code_phase)];
        sum_i += weight * (double)chip * component->osc_i;
        sum_q += weight * (double)chip * component->osc_q;

        component->code_phase +=
            record->code_rate_chips_per_s / (double)opt.sample_rate;
        while (component->code_phase >= GPS_CA_CHIPS)
          component->code_phase -= GPS_CA_CHIPS;
        next_i = component->osc_i * component->step_i -
                 component->osc_q * component->step_q;
        next_q = component->osc_i * component->step_q +
                 component->osc_q * component->step_i;
        component->osc_i = next_i;
        component->osc_q = next_q;
        if (((sample + 1) % 1048576ULL) == 0) {
          double norm = hypot(component->osc_i, component->osc_q);
          component->osc_i /= norm;
          component->osc_q /= norm;
        }
      }

      raw_i = opt.amplitude * env * sum_i;
      raw_q = opt.amplitude * env * sum_q;
      if (fabs(raw_i) > peak) peak = fabs(raw_i);
      if (fabs(raw_q) > peak) peak = fabs(raw_q);
      if (sample >= onset + ramp && sample < offset - ramp) {
        plateau_power += (long double)raw_i * raw_i +
                         (long double)raw_q * raw_q;
        ++plateau_samples;
      }
      qi = lround(raw_i * 32767.0);
      qq = lround(raw_q * 32767.0);
      if (qi > 32767) { qi = 32767; ++clipped_components; }
      if (qi < -32768) { qi = -32768; ++clipped_components; }
      if (qq > 32767) { qq = 32767; ++clipped_components; }
      if (qq < -32768) { qq = -32768; ++clipped_components; }
      buffer[2 * k] = (int16_t)qi;
      buffer[2 * k + 1] = (int16_t)qq;
    }
    if (fwrite(buffer, 2 * sizeof(*buffer), count, out) != count) {
      fprintf(stderr, "matchedgen: failed writing '%s': %s\n", opt.output,
              strerror(errno));
      goto cleanup;
    }
    next_sample += count;
  }
  if (fclose(out) != 0) {
    out = NULL;
    fprintf(stderr, "matchedgen: failed closing output '%s'\n", opt.output);
    goto cleanup;
  }
  out = NULL;
  if (write_manifest(&opt, components, total_samples, onset, offset, ramp,
                     cadence,
                     plateau_samples > 0
                         ? sqrt((double)(plateau_power / plateau_samples))
                         : 0.0,
                     peak, clipped_components, plateau_samples) != 0)
    goto cleanup;
  status = 0;

cleanup:
  if (out != NULL)
    fclose(out);
  if (status != 0 && opt.output != NULL)
    remove(opt.output);
  free(buffer);
  for (i = 0; i < opt.target_count; ++i)
    free(components[i].records);
  return status;
}
