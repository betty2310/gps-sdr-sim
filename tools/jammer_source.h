#ifndef GPS_SDR_SIM_JAMMER_SOURCE_H
#define GPS_SDR_SIM_JAMMER_SOURCE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  JAMMER_SOURCE_CW = 0,
  JAMMER_SOURCE_NARROWBAND,
  JAMMER_SOURCE_WIDEBAND,
  JAMMER_SOURCE_CHIRP,
  JAMMER_SOURCE_PULSED
} jammer_source_type_t;

typedef struct {
  jammer_source_type_t type;
  double sample_rate_hz;
  uint64_t total_samples;
  double frequency_hz;
  double end_frequency_hz;
  double bandwidth_hz;
  double phase_rad;
  double amplitude;
  uint64_t seed;
  uint64_t onset_sample;
  uint64_t offset_sample;
  uint64_t ramp_samples;
  uint64_t sweep_samples;
  uint64_t pulse_period_samples;
  uint64_t pulse_on_samples;
  uint64_t pulse_ramp_samples;
} jammer_source_config_t;

typedef struct {
  double active_plateau_rms_full_scale;
  double peak_component_full_scale;
  uint64_t clipped_components;
  uint64_t rendered_samples;
  uint64_t plateau_samples;
  uint64_t source_on_samples;
} jammer_source_metrics_t;

typedef struct {
  jammer_source_config_t config;
  uint64_t next_sample;
  uint64_t plateau_samples;
  uint64_t clipped_components;
  uint64_t source_on_samples;
  uint64_t rng_state;
  uint64_t sweep_position;
  long double plateau_power;
  double peak_component;
  double osc_i;
  double osc_q;
  double step_i;
  double step_q;
  double chirp_step_step_i;
  double chirp_step_step_q;
  double narrowband_pole;
  double narrowband_input_scale;
  double narrowband_state_i;
  double narrowband_state_q;
} jammer_source_t;

const char *jammer_source_type_name(jammer_source_type_t type);

int jammer_source_parse_type(const char *name, jammer_source_type_t *type);

int jammer_source_init(jammer_source_t *source,
                       const jammer_source_config_t *config, char *error,
                       size_t error_size);

size_t jammer_source_render_sc16(jammer_source_t *source, int16_t *buffer,
                                 size_t max_samples);

int jammer_source_done(const jammer_source_t *source);

void jammer_source_get_metrics(const jammer_source_t *source,
                               jammer_source_metrics_t *metrics);

#ifdef __cplusplus
}
#endif

#endif
