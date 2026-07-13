#include "matched_code_source.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static void set_error(char *error, size_t error_size, const char *format, ...) {
  va_list args;

  if (error == NULL || error_size == 0)
    return;
  va_start(args, format);
  vsnprintf(error, error_size, format, args);
  va_end(args);
}

static uint64_t splitmix64(uint64_t *state) {
  uint64_t z = (*state += UINT64_C(0x9e3779b97f4a7c15));
  z = (z ^ (z >> 30)) * UINT64_C(0xbf58476d1ce4e5b9);
  z = (z ^ (z >> 27)) * UINT64_C(0x94d049bb133111eb);
  return z ^ (z >> 31);
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

static void set_frequency(matched_code_component_t *component,
                          double carrier_doppler_hz,
                          double sample_rate_hz) {
  double radians = 2.0 * M_PI * carrier_doppler_hz / sample_rate_hz;
  component->step_i = cos(radians);
  component->step_q = sin(radians);
}

int matched_code_source_init(matched_code_source_t *source,
                             const matched_code_source_config_t *config,
                             char *error, size_t error_size) {
  size_t i;

  if (source == NULL || config == NULL) {
    set_error(error, error_size, "source and config are required");
    return -1;
  }
  if (!isfinite(config->sample_rate_hz) || config->sample_rate_hz <= 0.0 ||
      config->total_samples == 0 || config->target_count == 0 ||
      config->target_count > MATCHED_CODE_MAX_TARGETS ||
      !isfinite(config->amplitude) || config->amplitude <= 0.0 ||
      config->onset_sample >= config->offset_sample ||
      config->offset_sample > config->total_samples ||
      config->ramp_samples * 2 >=
          config->offset_sample - config->onset_sample) {
    set_error(error, error_size,
              "invalid sample contract or active plateau");
    return -1;
  }

  memset(source, 0, sizeof(*source));
  source->config = *config;
  for (i = 0; i < config->target_count; ++i) {
    size_t previous;
    int prn = config->target_prns[i];
    uint64_t phase_state;

    if (prn < 1 || prn > 32) {
      set_error(error, error_size, "target PRN %d is outside 1..32", prn);
      return -1;
    }
    for (previous = 0; previous < i; ++previous) {
      if (config->target_prns[previous] == prn) {
        set_error(error, error_size, "target PRN %d is duplicated", prn);
        return -1;
      }
    }

    source->components[i].prn = prn;
    if (gps_ca_generate_bipolar(prn, source->components[i].code) != 0) {
      set_error(error, error_size, "failed to generate C/A code for PRN %d",
                prn);
      return -1;
    }
    phase_state = config->phase_seed ^
                  ((uint64_t)prn * UINT64_C(0xd1b54a32d192ed03));
    source->components[i].initial_phase_rad =
        2.0 * M_PI * (double)(splitmix64(&phase_state) >> 11) /
        9007199254740992.0;
    source->components[i].osc_i =
        cos(source->components[i].initial_phase_rad);
    source->components[i].osc_q =
        sin(source->components[i].initial_phase_rad);
  }
  return 0;
}

int matched_code_source_set_epoch(matched_code_source_t *source,
                                  const matched_code_target_state_t *states,
                                  size_t state_count, char *error,
                                  size_t error_size) {
  size_t i;

  if (source == NULL || states == NULL ||
      state_count != source->config.target_count) {
    set_error(error, error_size,
              "one target state per configured PRN is required");
    return -1;
  }
  for (i = 0; i < state_count; ++i) {
    const matched_code_target_state_t *state = &states[i];
    matched_code_component_t *component = &source->components[i];

    if (state->sample_offset != source->position) {
      set_error(error, error_size,
                "target PRN %d sample offset does not own the next sample",
                state->prn);
      return -1;
    }
    if (state->prn != component->prn) {
      set_error(error, error_size,
                "target state order does not match configured PRNs");
      return -1;
    }
    if (!isfinite(state->code_phase_chips) ||
        state->code_phase_chips < 0.0 ||
        state->code_phase_chips >= GPS_CA_CHIPS ||
        !isfinite(state->carrier_doppler_hz) ||
        fabs(state->carrier_doppler_hz) >= source->config.sample_rate_hz / 2.0 ||
        !isfinite(state->code_rate_chips_per_s) ||
        state->code_rate_chips_per_s <= 0.0 || state->clean_gain <= 0) {
      set_error(error, error_size, "invalid target state for PRN %d",
                state->prn);
      return -1;
    }
  }

  for (i = 0; i < state_count; ++i) {
    matched_code_component_t *component = &source->components[i];
    component->code_phase = states[i].code_phase_chips;
    component->code_rate_chips_per_s = states[i].code_rate_chips_per_s;
    set_frequency(component, states[i].carrier_doppler_hz,
                  source->config.sample_rate_hz);
  }
  source->epoch_initialized = 1;
  return 0;
}

static size_t render(matched_code_source_t *source, double *f64_output,
                     int16_t *sc16_output, size_t requested_samples) {
  uint64_t remaining;
  size_t count;
  size_t k;
  double weight;

  if (source == NULL || !source->epoch_initialized ||
      (f64_output == NULL && sc16_output == NULL) || requested_samples == 0 ||
      source->position >= source->config.total_samples)
    return 0;

  remaining = source->config.total_samples - source->position;
  count = requested_samples < remaining ? requested_samples : (size_t)remaining;
  weight = 1.0 / sqrt((double)source->config.target_count);

  for (k = 0; k < count; ++k) {
    uint64_t sample = source->position;
    double sum_i = 0.0;
    double sum_q = 0.0;
    double env = envelope(sample, source->config.onset_sample,
                          source->config.offset_sample,
                          source->config.ramp_samples);
    double raw_i;
    double raw_q;
    size_t i;

    for (i = 0; i < source->config.target_count; ++i) {
      matched_code_component_t *component = &source->components[i];
      int chip = component->code[(int)floor(component->code_phase)];
      double next_i;
      double next_q;

      sum_i += weight * (double)chip * component->osc_i;
      sum_q += weight * (double)chip * component->osc_q;
      component->code_phase +=
          component->code_rate_chips_per_s / source->config.sample_rate_hz;
      while (component->code_phase >= GPS_CA_CHIPS)
        component->code_phase -= GPS_CA_CHIPS;

      next_i = component->osc_i * component->step_i -
               component->osc_q * component->step_q;
      next_q = component->osc_i * component->step_q +
               component->osc_q * component->step_i;
      component->osc_i = next_i;
      component->osc_q = next_q;
      if (((sample + 1) % UINT64_C(1048576)) == 0) {
        double norm = hypot(component->osc_i, component->osc_q);
        component->osc_i /= norm;
        component->osc_q /= norm;
      }
    }

    raw_i = source->config.amplitude * env * sum_i;
    raw_q = source->config.amplitude * env * sum_q;
    if (fabs(raw_i) > source->peak_component)
      source->peak_component = fabs(raw_i);
    if (fabs(raw_q) > source->peak_component)
      source->peak_component = fabs(raw_q);
    if (sample >= source->config.onset_sample + source->config.ramp_samples &&
        sample < source->config.offset_sample - source->config.ramp_samples) {
      source->plateau_power +=
          (long double)raw_i * raw_i + (long double)raw_q * raw_q;
      ++source->plateau_samples;
    }

    if (f64_output != NULL) {
      f64_output[2 * k] = raw_i;
      f64_output[2 * k + 1] = raw_q;
    } else {
      long qi = lround(raw_i * 32767.0);
      long qq = lround(raw_q * 32767.0);
      if (qi > 32767) {
        qi = 32767;
        ++source->clipped_components;
      }
      if (qi < -32768) {
        qi = -32768;
        ++source->clipped_components;
      }
      if (qq > 32767) {
        qq = 32767;
        ++source->clipped_components;
      }
      if (qq < -32768) {
        qq = -32768;
        ++source->clipped_components;
      }
      sc16_output[2 * k] = (int16_t)qi;
      sc16_output[2 * k + 1] = (int16_t)qq;
    }
    ++source->position;
  }
  return count;
}

size_t matched_code_source_render_f64(matched_code_source_t *source,
                                      double *interleaved_iq,
                                      size_t requested_samples) {
  return render(source, interleaved_iq, NULL, requested_samples);
}

size_t matched_code_source_render_sc16(matched_code_source_t *source,
                                       int16_t *interleaved_iq,
                                       size_t requested_samples) {
  return render(source, NULL, interleaved_iq, requested_samples);
}

int matched_code_source_done(const matched_code_source_t *source) {
  return source != NULL && source->position == source->config.total_samples;
}

double matched_code_source_initial_phase_rad(
    const matched_code_source_t *source, size_t target_index) {
  if (source == NULL || target_index >= source->config.target_count)
    return NAN;
  return source->components[target_index].initial_phase_rad;
}

void matched_code_source_get_metrics(
    const matched_code_source_t *source,
    matched_code_source_metrics_t *metrics) {
  if (source == NULL || metrics == NULL)
    return;
  metrics->rendered_samples = source->position;
  metrics->plateau_samples = source->plateau_samples;
  metrics->clipped_components = source->clipped_components;
  metrics->active_plateau_rms =
      source->plateau_samples > 0
          ? sqrt((double)(source->plateau_power / source->plateau_samples))
          : 0.0;
  metrics->peak_component = source->peak_component;
}
