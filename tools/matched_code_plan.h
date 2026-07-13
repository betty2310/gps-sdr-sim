#ifndef GPS_SDR_SIM_MATCHED_CODE_PLAN_H
#define GPS_SDR_SIM_MATCHED_CODE_PLAN_H

#include <stddef.h>
#include <stdint.h>

#include "matched_code_source.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  double sample_rate_hz;
  uint64_t total_samples;
  uint64_t onset_sample;
  uint64_t offset_sample;
  uint64_t ramp_samples;
  uint64_t reference_start_sample;
  uint64_t reference_end_sample;
  int target_prns[MATCHED_CODE_MAX_TARGETS];
  size_t target_count;
  double requested_js_db;
  uint64_t phase_seed;
  double clean_reference_rms;
  double jammer_reference_rms;
  double reference_js_db;
  double jammer_scale;
  double clean_component_bound;
  double jammer_component_bound;
  double predicted_composite_bound;
  double common_gain;
  double predicted_headroom_db;
} matched_code_plan_t;

typedef struct {
  uint64_t samples;
  uint64_t plateau_samples;
  uint64_t clipped_components;
  long double clean_power;
  long double jammer_power;
  long double composite_power;
  double composite_peak;
} matched_code_mix_metrics_t;

int matched_code_parse_targets(const char *text,
                               int targets[MATCHED_CODE_MAX_TARGETS],
                               size_t *target_count, char *error,
                               size_t error_size);

int matched_code_plan_init(matched_code_plan_t *plan, double sample_rate_hz,
                           double duration_seconds, double onset_seconds,
                           double offset_seconds, double ramp_seconds,
                           const char *target_prns, double requested_js_db,
                           uint64_t phase_seed, char *error,
                           size_t error_size);

int matched_code_plan_calibrate(matched_code_plan_t *plan,
                                double clean_reference_rms,
                                double jammer_reference_rms,
                                double clean_component_bound,
                                double jammer_component_bound, char *error,
                                size_t error_size);

void matched_code_mix_metrics_init(matched_code_mix_metrics_t *metrics);

int matched_code_mix_sc16(const matched_code_plan_t *plan,
                          uint64_t first_sample_offset,
                          const double *clean_interleaved_iq,
                          const double *jammer_interleaved_iq,
                          int16_t *output_interleaved_iq, size_t sample_count,
                          matched_code_mix_metrics_t *metrics);

double matched_code_mix_achieved_js_db(
    const matched_code_mix_metrics_t *metrics);

#ifdef __cplusplus
}
#endif

#endif
