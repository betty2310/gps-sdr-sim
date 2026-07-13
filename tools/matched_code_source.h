#ifndef GPS_SDR_SIM_MATCHED_CODE_SOURCE_H
#define GPS_SDR_SIM_MATCHED_CODE_SOURCE_H

#include <stddef.h>
#include <stdint.h>

#include "gps_ca.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MATCHED_CODE_MAX_TARGETS 32

typedef struct {
  uint64_t sample_offset;
  int prn;
  double code_phase_chips;
  double carrier_doppler_hz;
  double code_rate_chips_per_s;
  int clean_gain;
} matched_code_target_state_t;

typedef struct {
  double sample_rate_hz;
  uint64_t total_samples;
  uint64_t onset_sample;
  uint64_t offset_sample;
  uint64_t ramp_samples;
  double amplitude;
  uint64_t phase_seed;
  int target_prns[MATCHED_CODE_MAX_TARGETS];
  size_t target_count;
} matched_code_source_config_t;

typedef struct {
  uint64_t rendered_samples;
  uint64_t plateau_samples;
  uint64_t clipped_components;
  double active_plateau_rms;
  double peak_component;
} matched_code_source_metrics_t;

typedef struct {
  int prn;
  int8_t code[GPS_CA_CHIPS];
  double code_phase;
  double code_rate_chips_per_s;
  double osc_i;
  double osc_q;
  double step_i;
  double step_q;
  double initial_phase_rad;
} matched_code_component_t;

typedef struct {
  matched_code_source_config_t config;
  matched_code_component_t components[MATCHED_CODE_MAX_TARGETS];
  uint64_t position;
  long double plateau_power;
  uint64_t plateau_samples;
  uint64_t clipped_components;
  double peak_component;
  int epoch_initialized;
} matched_code_source_t;

int matched_code_source_init(matched_code_source_t *source,
                             const matched_code_source_config_t *config,
                             char *error, size_t error_size);

int matched_code_source_set_epoch(matched_code_source_t *source,
                                  const matched_code_target_state_t *states,
                                  size_t state_count, char *error,
                                  size_t error_size);

size_t matched_code_source_render_f64(matched_code_source_t *source,
                                      double *interleaved_iq,
                                      size_t requested_samples);

size_t matched_code_source_render_sc16(matched_code_source_t *source,
                                       int16_t *interleaved_iq,
                                       size_t requested_samples);

int matched_code_source_done(const matched_code_source_t *source);

double matched_code_source_initial_phase_rad(
    const matched_code_source_t *source, size_t target_index);

void matched_code_source_get_metrics(
    const matched_code_source_t *source,
    matched_code_source_metrics_t *metrics);

#ifdef __cplusplus
}
#endif

#endif
