#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "tools/jammer_source.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static void test_known_quadrature_tone(void) {
  jammer_source_config_t config = {
      .sample_rate_hz = 4.0,
      .total_samples = 4,
      .frequency_hz = 1.0,
      .phase_rad = 0.0,
      .amplitude = 0.5,
      .onset_sample = 0,
      .offset_sample = 4,
      .ramp_samples = 0,
  };
  jammer_source_t source;
  int16_t samples[8];
  char error[128];

  assert(jammer_source_init(&source, &config, error, sizeof(error)) == 0);
  assert(jammer_source_render_sc16(&source, samples, 4) == 4);
  assert(jammer_source_done(&source));

  assert(samples[0] == 16384 && samples[1] == 0);
  assert(samples[2] == 0 && samples[3] == 16384);
  assert(samples[4] == -16384 && samples[5] == 0);
  assert(samples[6] == 0 && samples[7] == -16384);
}

static void test_chunk_boundary_invariance(void) {
  jammer_source_config_t config = {
      .sample_rate_hz = 100000.0,
      .total_samples = 10000,
      .frequency_hz = 12345.0,
      .phase_rad = 0.25,
      .amplitude = 0.5,
      .onset_sample = 2000,
      .offset_sample = 8000,
      .ramp_samples = 500,
  };
  static const size_t chunks[] = {1, 7, 64, 513, 2048};
  jammer_source_metrics_t metrics;
  jammer_source_t contiguous;
  jammer_source_t chunked;
  int16_t *expected =
      calloc((size_t)config.total_samples * 2, sizeof(*expected));
  int16_t *actual = calloc((size_t)config.total_samples * 2, sizeof(*actual));
  uint64_t position = 0;
  size_t chunk_index = 0;
  char error[128];

  assert(expected != NULL && actual != NULL);
  assert(jammer_source_init(&contiguous, &config, error, sizeof(error)) == 0);
  assert(jammer_source_init(&chunked, &config, error, sizeof(error)) == 0);

  assert(jammer_source_render_sc16(&contiguous, expected,
                                   (size_t)config.total_samples) ==
         config.total_samples);

  while (!jammer_source_done(&chunked)) {
    size_t requested =
        chunks[chunk_index % (sizeof(chunks) / sizeof(chunks[0]))];
    size_t rendered = jammer_source_render_sc16(
        &chunked, &actual[(size_t)position * 2], requested);
    assert(rendered > 0);
    position += rendered;
    ++chunk_index;
  }

  assert(position == config.total_samples);
  assert(memcmp(expected, actual,
                (size_t)config.total_samples * 2 * sizeof(*expected)) == 0);

  for (position = 0; position < config.onset_sample; ++position) {
    assert(actual[(size_t)position * 2] == 0);
    assert(actual[(size_t)position * 2 + 1] == 0);
  }
  for (position = config.offset_sample; position < config.total_samples;
       ++position) {
    assert(actual[(size_t)position * 2] == 0);
    assert(actual[(size_t)position * 2 + 1] == 0);
  }

  jammer_source_get_metrics(&chunked, &metrics);
  assert(fabs(metrics.active_plateau_rms_full_scale - 0.5) < 1e-9);
  assert(metrics.clipped_components == 0);
  assert(metrics.rendered_samples == config.total_samples);
  assert(metrics.plateau_samples ==
         config.offset_sample - config.onset_sample - 2 * config.ramp_samples);

  free(expected);
  free(actual);
}

static void test_seeded_wideband_noise(void) {
  jammer_source_config_t config = {
      .type = JAMMER_SOURCE_WIDEBAND,
      .sample_rate_hz = 100000.0,
      .total_samples = 10000,
      .amplitude = 0.1,
      .seed = 42,
      .onset_sample = 0,
      .offset_sample = 10000,
      .ramp_samples = 0,
  };
  jammer_source_t first;
  jammer_source_t chunked;
  jammer_source_t different;
  jammer_source_metrics_t metrics;
  int16_t *expected = calloc(20000, sizeof(*expected));
  int16_t *actual = calloc(20000, sizeof(*actual));
  int16_t *other = calloc(20000, sizeof(*other));
  uint64_t position = 0;
  char error[128];

  assert(expected != NULL && actual != NULL && other != NULL);
  assert(jammer_source_init(&first, &config, error, sizeof(error)) == 0);
  assert(jammer_source_init(&chunked, &config, error, sizeof(error)) == 0);
  assert(jammer_source_render_sc16(&first, expected, 10000) == 10000);
  while (!jammer_source_done(&chunked)) {
    size_t rendered = jammer_source_render_sc16(
        &chunked, &actual[(size_t)position * 2], 137);
    assert(rendered > 0);
    position += rendered;
  }
  assert(memcmp(expected, actual, 20000 * sizeof(*expected)) == 0);

  config.seed = 43;
  assert(jammer_source_init(&different, &config, error, sizeof(error)) == 0);
  assert(jammer_source_render_sc16(&different, other, 10000) == 10000);
  assert(memcmp(expected, other, 20000 * sizeof(*expected)) != 0);

  jammer_source_get_metrics(&chunked, &metrics);
  assert(fabs(metrics.active_plateau_rms_full_scale - 0.1) < 0.003);
  assert(metrics.clipped_components == 0);

  free(expected);
  free(actual);
  free(other);
}

static void test_narrowband_noise_correlation(void) {
  jammer_source_config_t config = {
      .type = JAMMER_SOURCE_NARROWBAND,
      .sample_rate_hz = 10000.0,
      .total_samples = 200000,
      .frequency_hz = 0.0,
      .bandwidth_hz = 500.0,
      .amplitude = 0.1,
      .seed = 99,
      .onset_sample = 0,
      .offset_sample = 200000,
      .ramp_samples = 0,
  };
  jammer_source_t source;
  jammer_source_metrics_t metrics;
  int16_t *samples = calloc(400000, sizeof(*samples));
  long double lag = 0.0;
  long double power = 0.0;
  double expected_pole =
      (config.sample_rate_hz - config.bandwidth_hz) /
      (config.sample_rate_hz + config.bandwidth_hz);
  uint64_t sample;
  char error[128];

  assert(samples != NULL);
  assert(jammer_source_init(&source, &config, error, sizeof(error)) == 0);
  assert(jammer_source_render_sc16(&source, samples, 200000) == 200000);
  for (sample = 1001; sample < config.total_samples; ++sample) {
    double previous_i = samples[(size_t)(sample - 1) * 2];
    double previous_q = samples[(size_t)(sample - 1) * 2 + 1];
    double current_i = samples[(size_t)sample * 2];
    double current_q = samples[(size_t)sample * 2 + 1];
    lag += previous_i * current_i + previous_q * current_q;
    power += previous_i * previous_i + previous_q * previous_q;
  }
  assert(fabs((double)(lag / power) - expected_pole) < 0.01);
  jammer_source_get_metrics(&source, &metrics);
  assert(fabs(metrics.active_plateau_rms_full_scale - 0.1) < 0.003);
  assert(metrics.clipped_components == 0);
  free(samples);
}

static double sample_frequency(const int16_t *first, const int16_t *second,
                               double sample_rate_hz) {
  double real = (double)first[0] * second[0] + (double)first[1] * second[1];
  double imaginary =
      (double)first[0] * second[1] - (double)first[1] * second[0];
  return atan2(imaginary, real) * sample_rate_hz / (2.0 * M_PI);
}

static void test_linear_chirp_frequency_law(void) {
  jammer_source_config_t config = {
      .type = JAMMER_SOURCE_CHIRP,
      .sample_rate_hz = 1000.0,
      .total_samples = 102,
      .frequency_hz = -100.0,
      .end_frequency_hz = 100.0,
      .phase_rad = 0.25,
      .amplitude = 0.5,
      .onset_sample = 0,
      .offset_sample = 102,
      .ramp_samples = 0,
      .sweep_samples = 101,
  };
  jammer_source_t source;
  int16_t samples[204];
  char error[128];

  assert(jammer_source_init(&source, &config, error, sizeof(error)) == 0);
  assert(jammer_source_render_sc16(&source, samples, 102) == 102);
  assert(fabs(sample_frequency(&samples[0], &samples[2], 1000.0) + 100.0) <
         0.1);
  assert(fabs(sample_frequency(&samples[198], &samples[200], 1000.0) - 98.0) <
         0.1);
}

static void test_periodic_pulsed_cw(void) {
  jammer_source_config_t config = {
      .type = JAMMER_SOURCE_PULSED,
      .sample_rate_hz = 1000.0,
      .total_samples = 30,
      .frequency_hz = 0.0,
      .phase_rad = 0.0,
      .amplitude = 0.5,
      .onset_sample = 0,
      .offset_sample = 30,
      .ramp_samples = 0,
      .pulse_period_samples = 10,
      .pulse_on_samples = 4,
      .pulse_ramp_samples = 0,
  };
  jammer_source_t source;
  jammer_source_metrics_t metrics;
  int16_t samples[60];
  uint64_t sample;
  char error[128];

  assert(jammer_source_init(&source, &config, error, sizeof(error)) == 0);
  assert(jammer_source_render_sc16(&source, samples, 30) == 30);
  for (sample = 0; sample < 30; ++sample) {
    if (sample % 10 < 4) {
      assert(samples[(size_t)sample * 2] == 16384);
      assert(samples[(size_t)sample * 2 + 1] == 0);
    } else {
      assert(samples[(size_t)sample * 2] == 0);
      assert(samples[(size_t)sample * 2 + 1] == 0);
    }
  }
  jammer_source_get_metrics(&source, &metrics);
  assert(fabs(metrics.active_plateau_rms_full_scale - 0.5 * sqrt(0.4)) <
         1e-12);
  assert(metrics.source_on_samples == 12);
}

static void test_validation(void) {
  jammer_source_config_t config = {
      .sample_rate_hz = 1000.0,
      .total_samples = 1000,
      .frequency_hz = 500.0,
      .phase_rad = 0.0,
      .amplitude = 0.5,
      .onset_sample = 100,
      .offset_sample = 900,
      .ramp_samples = 10,
  };
  jammer_source_t source;
  char error[128];

  assert(jammer_source_init(&source, &config, error, sizeof(error)) != 0);
  assert(strstr(error, "Nyquist") != NULL);

  config.frequency_hz = 100.0;
  config.ramp_samples = 400;
  assert(jammer_source_init(&source, &config, error, sizeof(error)) != 0);
  assert(strstr(error, "plateau") != NULL);
}

int main(void) {
  test_known_quadrature_tone();
  test_chunk_boundary_invariance();
  test_seeded_wideband_noise();
  test_narrowband_noise_correlation();
  test_linear_chirp_frequency_law();
  test_periodic_pulsed_cw();
  test_validation();
  return 0;
}
