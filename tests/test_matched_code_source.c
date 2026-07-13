#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "player/matched_code_alignment.h"
#include "tools/matched_code_source.h"

static matched_code_source_config_t base_config(void) {
  matched_code_source_config_t config;

  memset(&config, 0, sizeof(config));
  config.sample_rate_hz = 4.0;
  config.total_samples = 8;
  config.onset_sample = 0;
  config.offset_sample = 8;
  config.ramp_samples = 0;
  config.amplitude = 0.5;
  config.phase_seed = 42;
  config.target_prns[0] = 1;
  config.target_count = 1;
  return config;
}

static matched_code_target_state_t state_at(uint64_t offset,
                                            double code_phase,
                                            double doppler_hz) {
  matched_code_target_state_t state;

  memset(&state, 0, sizeof(state));
  state.sample_offset = offset;
  state.prn = 1;
  state.code_phase_chips = code_phase;
  state.carrier_doppler_hz = doppler_hz;
  state.code_rate_chips_per_s = 1.0;
  state.clean_gain = 80;
  return state;
}

static void test_known_samples_and_boundary_ownership(void) {
  const int16_t expected[] = {
      3537, -15997, 15997, 3537, -3537, 15997, -15997, -3537,
      -3537, 15997, 15997, 3537, 3537, -15997, -15997, -3537,
  };
  matched_code_source_config_t config = base_config();
  matched_code_source_t source;
  matched_code_target_state_t first = state_at(0, 0.0, 1.0);
  matched_code_target_state_t second = state_at(4, 2.0, -1.0);
  int16_t samples[16];
  char error[256];

  assert(matched_code_source_init(&source, &config, error, sizeof(error)) == 0);
  assert(fabs(matched_code_source_initial_phase_rad(&source, 0) -
              4.9299969426161301) < 1e-15);
  assert(matched_code_source_set_epoch(&source, &first, 1, error,
                                       sizeof(error)) == 0);
  assert(matched_code_source_render_sc16(&source, samples, 4) == 4);
  assert(matched_code_source_set_epoch(&source, &second, 1, error,
                                       sizeof(error)) == 0);
  assert(matched_code_source_render_sc16(&source, samples + 8, 4) == 4);
  assert(memcmp(samples, expected, sizeof(expected)) == 0);
  assert(matched_code_source_done(&source));
}

static void test_chunk_invariance_and_clean_prefix_phase_advancement(void) {
  matched_code_source_config_t active_config = base_config();
  matched_code_source_config_t prefix_config = base_config();
  matched_code_source_t contiguous;
  matched_code_source_t chunked;
  matched_code_source_t prefix;
  matched_code_target_state_t state = state_at(0, 0.0, 1.0);
  int16_t expected[16];
  int16_t actual[16];
  int16_t delayed[16];
  char error[256];

  assert(matched_code_source_init(&contiguous, &active_config, error,
                                  sizeof(error)) == 0);
  assert(matched_code_source_init(&chunked, &active_config, error,
                                  sizeof(error)) == 0);
  assert(matched_code_source_set_epoch(&contiguous, &state, 1, error,
                                       sizeof(error)) == 0);
  assert(matched_code_source_set_epoch(&chunked, &state, 1, error,
                                       sizeof(error)) == 0);
  assert(matched_code_source_render_sc16(&contiguous, expected, 8) == 8);
  assert(matched_code_source_render_sc16(&chunked, actual, 1) == 1);
  assert(matched_code_source_render_sc16(&chunked, actual + 2, 2) == 2);
  assert(matched_code_source_render_sc16(&chunked, actual + 6, 5) == 5);
  assert(memcmp(expected, actual, sizeof(expected)) == 0);

  prefix_config.onset_sample = 4;
  assert(matched_code_source_init(&prefix, &prefix_config, error,
                                  sizeof(error)) == 0);
  assert(matched_code_source_set_epoch(&prefix, &state, 1, error,
                                       sizeof(error)) == 0);
  assert(matched_code_source_render_sc16(&prefix, delayed, 8) == 8);
  assert(delayed[0] == 0 && delayed[1] == 0);
  assert(delayed[6] == 0 && delayed[7] == 0);
  assert(memcmp(delayed + 8, expected + 8, 8 * sizeof(*expected)) == 0);
}

static void test_validation_and_metrics(void) {
  matched_code_source_config_t config = base_config();
  matched_code_source_t source;
  matched_code_source_metrics_t metrics;
  matched_code_target_state_t state = state_at(0, 0.0, 1.0);
  double samples[16];
  char error[256];

  config.ramp_samples = 4;
  assert(matched_code_source_init(&source, &config, error, sizeof(error)) != 0);
  assert(strstr(error, "plateau") != NULL);

  config = base_config();
  assert(matched_code_source_init(&source, &config, error, sizeof(error)) == 0);
  state.sample_offset = 1;
  assert(matched_code_source_set_epoch(&source, &state, 1, error,
                                       sizeof(error)) != 0);
  assert(strstr(error, "sample offset") != NULL);
  state.sample_offset = 0;
  assert(matched_code_source_set_epoch(&source, &state, 1, error,
                                       sizeof(error)) == 0);
  assert(matched_code_source_render_f64(&source, samples, 8) == 8);
  matched_code_source_get_metrics(&source, &metrics);
  assert(metrics.rendered_samples == 8);
  assert(metrics.plateau_samples == 8);
  assert(fabs(metrics.active_plateau_rms - 0.5) < 1e-12);
  assert(metrics.clipped_components == 0);
}

static void test_clean_navigation_bits_do_not_cross_alignment_boundary(void) {
  matched_code_source_config_t config = base_config();
  matched_code_source_t first_source;
  matched_code_source_t second_source;
  matched_code_target_state_t first_state;
  matched_code_target_state_t second_state;
  channel_t channel;
  int16_t first_samples[16];
  int16_t second_samples[16];
  char error[256];

  memset(&channel, 0, sizeof(channel));
  memset(&first_state, 0, sizeof(first_state));
  memset(&second_state, 0, sizeof(second_state));
  channel.prn = 1;
  channel.code_phase = 123.25;
  channel.f_carr = 1.0;
  channel.f_code = 1.0;
  channel.dataBit = 1;
  channel.dwrd[0] = 0xAAAAAAAAUL;
  channel.sbf[0][0] = 0x55555555UL;

  matched_code_capture_channel_state(&channel, 80, 0, &first_state);
  channel.dataBit = -1;
  channel.dwrd[0] = 0x55555555UL;
  channel.sbf[0][0] = 0xAAAAAAAAUL;
  channel.iword = 7;
  channel.ibit = 13;
  matched_code_capture_channel_state(&channel, 80, 0, &second_state);

  assert(memcmp(&first_state, &second_state, sizeof(first_state)) == 0);
  assert(matched_code_source_init(&first_source, &config, error,
                                  sizeof(error)) == 0);
  assert(matched_code_source_init(&second_source, &config, error,
                                  sizeof(error)) == 0);
  assert(matched_code_source_set_epoch(&first_source, &first_state, 1, error,
                                       sizeof(error)) == 0);
  assert(matched_code_source_set_epoch(&second_source, &second_state, 1, error,
                                       sizeof(error)) == 0);
  assert(matched_code_source_render_sc16(&first_source, first_samples, 8) == 8);
  assert(matched_code_source_render_sc16(&second_source, second_samples, 8) ==
         8);
  assert(memcmp(first_samples, second_samples, sizeof(first_samples)) == 0);
}

int main(void) {
  test_known_samples_and_boundary_ownership();
  test_chunk_invariance_and_clean_prefix_phase_advancement();
  test_validation_and_metrics();
  test_clean_navigation_bits_do_not_cross_alignment_boundary();
  return 0;
}
