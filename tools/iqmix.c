#define _FILE_OFFSET_BITS 64

#include <errno.h>
#include <getopt.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define CHUNK_SAMPLES 65536U

typedef struct {
  const char *clean;
  const char *jammer;
  const char *output;
  const char *manifest;
  uint64_t samples;
  uint64_t onset;
  uint64_t offset;
  uint64_t ramp;
  uint64_t seed;
  double js_db;
  double snr_db;
  double noise_rms;
  double gain;
  int disable_jammer;
  int have_snr;
  int have_noise_rms;
  int dry_run;
} options_t;

typedef struct {
  uint64_t state;
} rng_t;

typedef struct {
  double clean_power;
  double jammer_power;
  double jammer_scale;
  double noise_rms;
  double measured_noise_rms;
  double measured_js_db;
  double measured_snr_db;
  double pre_gain_peak;
  double post_gain_power;
  uint64_t clipped_components;
} metrics_t;

static void usage(FILE *stream) {
  fprintf(stream,
          "Usage: iqmix --clean FILE --jammer FILE --samples N --manifest FILE "
          "[options]\n"
          "Mix offline-only int16 IQ using measured clean/jammer powers.\n\n"
          "Options:\n"
          "  --output FILE          Mixed int16 IQ (required unless --dry-run)\n"
          "  --onset-sample N       Jammer onset sample\n"
          "  --offset-sample N      Jammer offset sample\n"
          "  --ramp-samples N       Rise/fall sample count\n"
          "  --js-db DB             Requested jammer/clean power ratio\n"
          "  --disable-jammer       Create a clean-plus-noise control\n"
          "  --snr-db DB            Requested clean/AWGN power ratio\n"
          "  --noise-rms COUNTS     Fixed complex AWGN RMS in int16 counts\n"
          "  --seed N               AWGN seed (default 1)\n"
          "  --gain VALUE           Common output gain (default 1)\n"
          "  --dry-run              Measure exact pre-gain peak without output\n"
          "  --help                 Show this help\n");
}

static int parse_u64(const char *text, uint64_t *value) {
  char *end = NULL;
  unsigned long long parsed;
  errno = 0;
  parsed = strtoull(text, &end, 10);
  if (errno != 0 || end == text || *end != '\0') return -1;
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

static uint64_t rng_next(rng_t *rng) {
  uint64_t x = rng->state;
  x ^= x >> 12;
  x ^= x << 25;
  x ^= x >> 27;
  rng->state = x;
  return x * UINT64_C(2685821657736338717);
}

static double uniform_open(rng_t *rng) {
  return ((double)(rng_next(rng) >> 11) + 0.5) * (1.0 / 9007199254740992.0);
}

static void complex_gaussian(rng_t *rng, double complex_rms, double *i,
                             double *q) {
  double radius = sqrt(-2.0 * log(uniform_open(rng)));
  double angle = 2.0 * M_PI * uniform_open(rng);
  double sigma = complex_rms / sqrt(2.0);
  *i = sigma * radius * cos(angle);
  *q = sigma * radius * sin(angle);
}

static int check_size(const char *path, uint64_t samples) {
  struct stat st;
  uint64_t expected = samples * 4ULL;
  if (samples > UINT64_MAX / 4ULL) return -1;
  if (stat(path, &st) != 0) {
    fprintf(stderr, "iqmix: cannot stat '%s': %s\n", path, strerror(errno));
    return -1;
  }
  if ((uint64_t)st.st_size != expected) {
    fprintf(stderr,
            "iqmix: '%s' has %llu bytes; expected exactly %llu for %llu "
            "complex samples\n",
            path, (unsigned long long)st.st_size,
            (unsigned long long)expected, (unsigned long long)samples);
    return -1;
  }
  return 0;
}

static int measure_inputs(const options_t *opt, metrics_t *metrics) {
  FILE *clean = NULL, *jammer = NULL;
  int16_t *clean_buf = NULL, *jam_buf = NULL;
  uint64_t n = 0, count_power = 0;
  long double clean_power = 0.0L, jam_power = 0.0L;
  int result = -1;

  clean = fopen(opt->clean, "rb");
  jammer = fopen(opt->jammer, "rb");
  clean_buf = malloc(CHUNK_SAMPLES * 2U * sizeof(*clean_buf));
  jam_buf = malloc(CHUNK_SAMPLES * 2U * sizeof(*jam_buf));
  if (clean == NULL || jammer == NULL || clean_buf == NULL || jam_buf == NULL) {
    fprintf(stderr, "iqmix: cannot open inputs or allocate buffers: %s\n",
            strerror(errno));
    goto done;
  }
  while (n < opt->samples) {
    size_t count = (size_t)((opt->samples - n) > CHUNK_SAMPLES
                                ? CHUNK_SAMPLES
                                : (opt->samples - n));
    size_t k;
    if (fread(clean_buf, 2U * sizeof(*clean_buf), count, clean) != count ||
        fread(jam_buf, 2U * sizeof(*jam_buf), count, jammer) != count) {
      fprintf(stderr, "iqmix: short input while measuring powers\n");
      goto done;
    }
    for (k = 0; k < count; ++k) {
      uint64_t sample = n + (uint64_t)k;
      if (sample >= opt->onset + opt->ramp &&
          sample < opt->offset - opt->ramp) {
        long double ci = clean_buf[2 * k], cq = clean_buf[2 * k + 1];
        long double ji = jam_buf[2 * k], jq = jam_buf[2 * k + 1];
        clean_power += ci * ci + cq * cq;
        jam_power += ji * ji + jq * jq;
        ++count_power;
      }
    }
    n += count;
  }
  if (count_power == 0 || clean_power <= 0.0L || jam_power <= 0.0L) {
    fprintf(stderr, "iqmix: non-finite or zero clean/jammer plateau power\n");
    goto done;
  }
  metrics->clean_power = (double)(clean_power / count_power);
  metrics->jammer_power = (double)(jam_power / count_power);
  if (!isfinite(metrics->clean_power) || !isfinite(metrics->jammer_power)) {
    fprintf(stderr, "iqmix: non-finite measured input power\n");
    goto done;
  }
  result = 0;
done:
  if (clean != NULL) fclose(clean);
  if (jammer != NULL) fclose(jammer);
  free(clean_buf);
  free(jam_buf);
  return result;
}

static int mix_pass(const options_t *opt, metrics_t *metrics) {
  FILE *clean = NULL, *jammer = NULL, *out = NULL;
  int16_t *clean_buf = NULL, *jam_buf = NULL, *out_buf = NULL;
  uint64_t n = 0, noise_count = 0;
  long double noise_power = 0.0L, output_power = 0.0L;
  rng_t rng = {.state = opt->seed == 0 ? UINT64_C(0x9e3779b97f4a7c15)
                                      : opt->seed};
  int result = -1;

  clean = fopen(opt->clean, "rb");
  jammer = fopen(opt->jammer, "rb");
  if (!opt->dry_run) out = fopen(opt->output, "wb");
  clean_buf = malloc(CHUNK_SAMPLES * 2U * sizeof(*clean_buf));
  jam_buf = malloc(CHUNK_SAMPLES * 2U * sizeof(*jam_buf));
  if (!opt->dry_run) out_buf = malloc(CHUNK_SAMPLES * 2U * sizeof(*out_buf));
  if (clean == NULL || jammer == NULL || (!opt->dry_run && out == NULL) ||
      clean_buf == NULL || jam_buf == NULL || (!opt->dry_run && out_buf == NULL)) {
    fprintf(stderr, "iqmix: cannot open files or allocate buffers: %s\n",
            strerror(errno));
    goto done;
  }

  while (n < opt->samples) {
    size_t count = (size_t)((opt->samples - n) > CHUNK_SAMPLES
                                ? CHUNK_SAMPLES
                                : (opt->samples - n));
    size_t k;
    if (fread(clean_buf, 2U * sizeof(*clean_buf), count, clean) != count ||
        fread(jam_buf, 2U * sizeof(*jam_buf), count, jammer) != count) {
      fprintf(stderr, "iqmix: short input while mixing\n");
      goto done;
    }
    for (k = 0; k < count; ++k) {
      double ni, nq;
      double mixed_i, mixed_q, gained_i, gained_q;
      long qi, qq;
      complex_gaussian(&rng, metrics->noise_rms, &ni, &nq);
      noise_power += (long double)ni * ni + (long double)nq * nq;
      ++noise_count;
      mixed_i = clean_buf[2 * k] + metrics->jammer_scale * jam_buf[2 * k] + ni;
      mixed_q = clean_buf[2 * k + 1] +
                metrics->jammer_scale * jam_buf[2 * k + 1] + nq;
      if (fabs(mixed_i) > metrics->pre_gain_peak)
        metrics->pre_gain_peak = fabs(mixed_i);
      if (fabs(mixed_q) > metrics->pre_gain_peak)
        metrics->pre_gain_peak = fabs(mixed_q);
      gained_i = opt->gain * mixed_i;
      gained_q = opt->gain * mixed_q;
      if (!opt->dry_run) {
        qi = lround(gained_i);
        qq = lround(gained_q);
        if (qi > 32767) { qi = 32767; ++metrics->clipped_components; }
        if (qi < -32768) { qi = -32768; ++metrics->clipped_components; }
        if (qq > 32767) { qq = 32767; ++metrics->clipped_components; }
        if (qq < -32768) { qq = -32768; ++metrics->clipped_components; }
        out_buf[2 * k] = (int16_t)qi;
        out_buf[2 * k + 1] = (int16_t)qq;
        output_power += (long double)qi * qi + (long double)qq * qq;
      }
    }
    if (!opt->dry_run &&
        fwrite(out_buf, 2U * sizeof(*out_buf), count, out) != count) {
      fprintf(stderr, "iqmix: failed writing '%s': %s\n", opt->output,
              strerror(errno));
      goto done;
    }
    n += count;
  }
  metrics->measured_noise_rms = sqrt((double)(noise_power / noise_count));
  metrics->measured_snr_db =
      10.0 * log10(metrics->clean_power /
                   (metrics->measured_noise_rms * metrics->measured_noise_rms));
  if (!opt->disable_jammer) {
    metrics->measured_js_db =
        10.0 * log10(metrics->jammer_scale * metrics->jammer_scale *
                     metrics->jammer_power / metrics->clean_power);
  }
  if (!opt->dry_run)
    metrics->post_gain_power = (double)(output_power / opt->samples);
  result = 0;
done:
  if (clean != NULL) fclose(clean);
  if (jammer != NULL) fclose(jammer);
  if (out != NULL && fclose(out) != 0) result = -1;
  free(clean_buf);
  free(jam_buf);
  free(out_buf);
  return result;
}

static int write_manifest(const options_t *opt, const metrics_t *m) {
  char requested_js[64];
  char requested_snr[64];
  char measured_js[64];
  char headroom[64];
  FILE *fp = fopen(opt->manifest, "w");
  if (fp == NULL) {
    fprintf(stderr, "iqmix: cannot open manifest '%s': %s\n", opt->manifest,
            strerror(errno));
    return -1;
  }
  if (opt->disable_jammer) {
    strcpy(requested_js, "null");
    strcpy(measured_js, "null");
  } else {
    snprintf(requested_js, sizeof(requested_js), "%.17g", opt->js_db);
    snprintf(measured_js, sizeof(measured_js), "%.17g", m->measured_js_db);
  }
  if (m->noise_rms == 0.0) {
    strcpy(requested_snr, "null");
  } else if (opt->have_snr) {
    snprintf(requested_snr, sizeof(requested_snr), "%.17g", opt->snr_db);
  } else {
    snprintf(requested_snr, sizeof(requested_snr), "%.17g",
             10.0 * log10(m->clean_power / (m->noise_rms * m->noise_rms)));
  }
  if (m->pre_gain_peak > 0.0) {
    snprintf(headroom, sizeof(headroom), "%.17g",
             20.0 * log10(32767.0 / (opt->gain * m->pre_gain_peak)));
  } else {
    strcpy(headroom, "null");
  }
  fprintf(fp,
          "{\n"
          "  \"schema\": \"gps-sdr-sim.iqmix.v1\",\n"
          "  \"tool\": \"iqmix\",\n"
          "  \"offline_only\": true,\n"
          "  \"dry_run\": %s,\n"
          "  \"sample_contract\": {\"format\": \"int16_le\", "
          "\"iq_order\": \"IQ\", \"complex_samples\": %llu, "
          "\"bytes\": %llu},\n"
          "  \"parameters\": {\"jammer_enabled\": %s, "
          "\"requested_js_db\": %s, \"requested_snr_db\": %s, "
          "\"seed\": %llu, \"gain\": %.17g, \"onset_sample\": %llu, "
          "\"offset_sample\": %llu, \"ramp_samples\": %llu},\n"
          "  \"measurements\": {\"clean_plateau_power_counts2\": %.17g, "
          "\"jammer_plateau_power_counts2\": %.17g, "
          "\"jammer_scale\": %.17g, \"noise_rms_counts\": %.17g, "
          "\"measured_noise_rms_counts\": %.17g, "
          "\"measured_js_db\": %s, \"measured_snr_db\": %.17g, "
          "\"pre_gain_peak_component_counts\": %.17g, "
          "\"post_gain_rms_counts\": %.17g, "
          "\"headroom_db\": %s, \"clipped_components\": %llu}\n"
          "}\n",
          opt->dry_run ? "true" : "false", (unsigned long long)opt->samples,
          (unsigned long long)(opt->samples * 4ULL),
          opt->disable_jammer ? "false" : "true", requested_js, requested_snr,
          (unsigned long long)opt->seed, opt->gain,
          (unsigned long long)opt->onset, (unsigned long long)opt->offset,
          (unsigned long long)opt->ramp, m->clean_power, m->jammer_power,
          m->jammer_scale, m->noise_rms, m->measured_noise_rms, measured_js,
          m->noise_rms > 0.0 ? m->measured_snr_db : 0.0,
          m->pre_gain_peak,
          m->post_gain_power > 0.0 ? sqrt(m->post_gain_power) : 0.0,
          headroom,
          (unsigned long long)m->clipped_components);
  return fclose(fp) == 0 ? 0 : -1;
}

int main(int argc, char **argv) {
  options_t opt = {.seed = 1, .gain = 1.0, .snr_db = -10.0};
  metrics_t metrics = {0};
  static const struct option long_options[] = {
      {"clean", required_argument, NULL, 'c'},
      {"jammer", required_argument, NULL, 'j'},
      {"output", required_argument, NULL, 'o'},
      {"manifest", required_argument, NULL, 'm'},
      {"samples", required_argument, NULL, 's'},
      {"onset-sample", required_argument, NULL, 'n'},
      {"offset-sample", required_argument, NULL, 'x'},
      {"ramp-samples", required_argument, NULL, 'r'},
      {"js-db", required_argument, NULL, 'J'},
      {"disable-jammer", no_argument, NULL, 'D'},
      {"snr-db", required_argument, NULL, 'S'},
      {"noise-rms", required_argument, NULL, 'N'},
      {"seed", required_argument, NULL, 'e'},
      {"gain", required_argument, NULL, 'g'},
      {"dry-run", no_argument, NULL, 'd'},
      {"help", no_argument, NULL, 'h'},
      {NULL, 0, NULL, 0}};
  int c;

  while ((c = getopt_long(argc, argv, "", long_options, NULL)) != -1) {
    switch (c) {
    case 'c': opt.clean = optarg; break;
    case 'j': opt.jammer = optarg; break;
    case 'o': opt.output = optarg; break;
    case 'm': opt.manifest = optarg; break;
    case 's': if (parse_u64(optarg, &opt.samples) != 0) return 2; break;
    case 'n': if (parse_u64(optarg, &opt.onset) != 0) return 2; break;
    case 'x': if (parse_u64(optarg, &opt.offset) != 0) return 2; break;
    case 'r': if (parse_u64(optarg, &opt.ramp) != 0) return 2; break;
    case 'J': if (parse_double(optarg, &opt.js_db) != 0) return 2; break;
    case 'D': opt.disable_jammer = 1; break;
    case 'S':
      if (parse_double(optarg, &opt.snr_db) != 0) return 2;
      opt.have_snr = 1;
      break;
    case 'N':
      if (parse_double(optarg, &opt.noise_rms) != 0) return 2;
      opt.have_noise_rms = 1;
      break;
    case 'e': if (parse_u64(optarg, &opt.seed) != 0) return 2; break;
    case 'g': if (parse_double(optarg, &opt.gain) != 0) return 2; break;
    case 'd': opt.dry_run = 1; break;
    case 'h': usage(stdout); return 0;
    default: usage(stderr); return 2;
    }
  }

  if (opt.clean == NULL || opt.jammer == NULL || opt.manifest == NULL ||
      (!opt.dry_run && opt.output == NULL) || opt.samples == 0 ||
      opt.onset >= opt.offset || opt.offset > opt.samples ||
      opt.ramp * 2 > opt.offset - opt.onset || opt.gain <= 0.0 ||
      (opt.have_snr && opt.have_noise_rms) ||
      (opt.have_noise_rms && opt.noise_rms < 0.0)) {
    fprintf(stderr, "iqmix: invalid or missing argument\n");
    usage(stderr);
    return 2;
  }
  if (check_size(opt.clean, opt.samples) != 0 ||
      check_size(opt.jammer, opt.samples) != 0 ||
      measure_inputs(&opt, &metrics) != 0) {
    return 1;
  }
  metrics.jammer_scale = opt.disable_jammer
                             ? 0.0
                             : sqrt(pow(10.0, opt.js_db / 10.0) *
                                    metrics.clean_power / metrics.jammer_power);
  metrics.noise_rms = opt.have_noise_rms
                          ? opt.noise_rms
                          : (opt.have_snr ? sqrt(metrics.clean_power /
                                                pow(10.0, opt.snr_db / 10.0))
                                          : 0.0);
  if (!isfinite(metrics.jammer_scale) || !isfinite(metrics.noise_rms) ||
      mix_pass(&opt, &metrics) != 0 || write_manifest(&opt, &metrics) != 0) {
    return 1;
  }
  return 0;
}
