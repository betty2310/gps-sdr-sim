#include "jammer_source.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static void set_error(char *error, size_t error_size, const char *message) {
  if (error != NULL && error_size > 0) {
    snprintf(error, error_size, "%s", message);
  }
}

const char *jammer_source_type_name(jammer_source_type_t type) {
  switch (type) {
  case JAMMER_SOURCE_CW:
    return "cw";
  case JAMMER_SOURCE_NARROWBAND:
    return "narrowband";
  case JAMMER_SOURCE_WIDEBAND:
    return "wideband";
  case JAMMER_SOURCE_CHIRP:
    return "chirp";
  case JAMMER_SOURCE_PULSED:
    return "pulsed";
  default:
    return "unknown";
  }
}

int jammer_source_parse_type(const char *name, jammer_source_type_t *type) {
  if (name == NULL || type == NULL) {
    return -1;
  }
  if (strcmp(name, "cw") == 0) {
    *type = JAMMER_SOURCE_CW;
  } else if (strcmp(name, "narrowband") == 0) {
    *type = JAMMER_SOURCE_NARROWBAND;
  } else if (strcmp(name, "wideband") == 0) {
    *type = JAMMER_SOURCE_WIDEBAND;
  } else if (strcmp(name, "chirp") == 0) {
    *type = JAMMER_SOURCE_CHIRP;
  } else if (strcmp(name, "pulsed") == 0) {
    *type = JAMMER_SOURCE_PULSED;
  } else {
    return -1;
  }
  return 0;
}

static double global_envelope(uint64_t sample,
                              const jammer_source_config_t *config) {
  uint64_t onset = config->onset_sample;
  uint64_t offset = config->offset_sample;
  uint64_t ramp = config->ramp_samples;

  if (sample < onset || sample >= offset) {
    return 0.0;
  }
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

static double pulse_envelope(uint64_t sample,
                             const jammer_source_config_t *config) {
  uint64_t position;
  uint64_t ramp;

  if (config->type != JAMMER_SOURCE_PULSED) {
    return 1.0;
  }

  position = (sample - config->onset_sample) % config->pulse_period_samples;
  if (position >= config->pulse_on_samples) {
    return 0.0;
  }
  ramp = config->pulse_ramp_samples;
  if (ramp > 0 && position < ramp) {
    double x = (double)position / (double)ramp;
    return 0.5 - 0.5 * cos(M_PI * x);
  }
  if (ramp > 0 && position >= config->pulse_on_samples - ramp) {
    double x = (double)(config->pulse_on_samples - position) / (double)ramp;
    return 0.5 - 0.5 * cos(M_PI * x);
  }
  return 1.0;
}

static uint64_t rng_next(uint64_t *state) {
  uint64_t value = *state;
  value ^= value >> 12;
  value ^= value << 25;
  value ^= value >> 27;
  *state = value;
  return value * UINT64_C(2685821657736338717);
}

static double rng_uniform_open(uint64_t *state) {
  return ((double)(rng_next(state) >> 11) + 1.0) /
         9007199254740993.0;
}

static void proper_complex_gaussian(jammer_source_t *source, double *out_i,
                                    double *out_q) {
  double radius = sqrt(-log(rng_uniform_open(&source->rng_state)));
  double angle = 2.0 * M_PI * rng_uniform_open(&source->rng_state);
  *out_i = radius * cos(angle);
  *out_q = radius * sin(angle);
}

static void normalize_pair(double *i, double *q) {
  double norm = hypot(*i, *q);
  if (norm > 0.0) {
    *i /= norm;
    *q /= norm;
  }
}

static void advance_oscillator(jammer_source_t *source, uint64_t active_index) {
  double next_i =
      source->osc_i * source->step_i - source->osc_q * source->step_q;
  double next_q =
      source->osc_i * source->step_q + source->osc_q * source->step_i;
  source->osc_i = next_i;
  source->osc_q = next_q;

  if (source->config.type == JAMMER_SOURCE_CHIRP) {
    ++source->sweep_position;
    if (source->sweep_position >= source->config.sweep_samples) {
      source->sweep_position = 0;
      source->step_i = cos(2.0 * M_PI * source->config.frequency_hz /
                           source->config.sample_rate_hz);
      source->step_q = sin(2.0 * M_PI * source->config.frequency_hz /
                           source->config.sample_rate_hz);
    } else {
      next_i = source->step_i * source->chirp_step_step_i -
               source->step_q * source->chirp_step_step_q;
      next_q = source->step_i * source->chirp_step_step_q +
               source->step_q * source->chirp_step_step_i;
      source->step_i = next_i;
      source->step_q = next_q;
    }
  }

  if (((active_index + 1) % 1048576ULL) == 0) {
    normalize_pair(&source->osc_i, &source->osc_q);
    if (source->config.type == JAMMER_SOURCE_CHIRP) {
      double frequency =
          source->config.frequency_hz +
          (source->config.end_frequency_hz - source->config.frequency_hz) *
              (double)source->sweep_position /
              (double)(source->config.sweep_samples - 1);
      source->step_i =
          cos(2.0 * M_PI * frequency / source->config.sample_rate_hz);
      source->step_q =
          sin(2.0 * M_PI * frequency / source->config.sample_rate_hz);
    }
  }
}

int jammer_source_init(jammer_source_t *source,
                       const jammer_source_config_t *config, char *error,
                       size_t error_size) {
  uint64_t active_samples;

  if (source == NULL || config == NULL) {
    set_error(error, error_size, "source and config are required");
    return -1;
  }
  if (!isfinite(config->sample_rate_hz) || config->sample_rate_hz <= 0.0) {
    set_error(error, error_size, "sample rate must be finite and positive");
    return -1;
  }
  if (config->total_samples == 0) {
    set_error(error, error_size, "total sample count must be positive");
    return -1;
  }
  if (config->type < JAMMER_SOURCE_CW ||
      config->type > JAMMER_SOURCE_PULSED) {
    set_error(error, error_size, "unsupported jammer source type");
    return -1;
  }
  if (!isfinite(config->phase_rad)) {
    set_error(error, error_size, "phase must be finite");
    return -1;
  }
  if (!isfinite(config->amplitude) || config->amplitude <= 0.0 ||
      config->amplitude > 1.0) {
    set_error(error, error_size, "amplitude must be in the interval (0, 1]");
    return -1;
  }
  if (config->onset_sample >= config->offset_sample ||
      config->offset_sample > config->total_samples) {
    set_error(error, error_size,
              "timing must satisfy onset < offset <= total samples");
    return -1;
  }

  active_samples = config->offset_sample - config->onset_sample;
  if (config->ramp_samples > active_samples / 2 ||
      (config->ramp_samples > 0 &&
       config->ramp_samples * 2 == active_samples)) {
    set_error(error, error_size,
              "rise and fall ramps must leave a non-empty plateau");
    return -1;
  }

  if (config->type == JAMMER_SOURCE_CW ||
      config->type == JAMMER_SOURCE_NARROWBAND ||
      config->type == JAMMER_SOURCE_PULSED) {
    if (!isfinite(config->frequency_hz) ||
        fabs(config->frequency_hz) >= config->sample_rate_hz / 2.0) {
      set_error(error, error_size,
                "source frequency must be finite and strictly inside Nyquist");
      return -1;
    }
  }
  if (config->type == JAMMER_SOURCE_NARROWBAND) {
    if (!isfinite(config->bandwidth_hz) || config->bandwidth_hz <= 0.0 ||
        config->bandwidth_hz >= config->sample_rate_hz ||
        fabs(config->frequency_hz) + config->bandwidth_hz / 2.0 >=
            config->sample_rate_hz / 2.0) {
      set_error(error, error_size,
                "narrowband ENBW must be positive and its nominal band must fit inside Nyquist");
      return -1;
    }
  }
  if (config->type == JAMMER_SOURCE_CHIRP) {
    if (!isfinite(config->frequency_hz) ||
        !isfinite(config->end_frequency_hz) ||
        fabs(config->frequency_hz) >= config->sample_rate_hz / 2.0 ||
        fabs(config->end_frequency_hz) >= config->sample_rate_hz / 2.0 ||
        config->frequency_hz == config->end_frequency_hz ||
        config->sweep_samples < 2) {
      set_error(error, error_size,
                "chirp endpoints must differ, fit inside Nyquist, and use at least two sweep samples");
      return -1;
    }
  }
  if (config->type == JAMMER_SOURCE_PULSED) {
    if (config->pulse_period_samples == 0 || config->pulse_on_samples == 0 ||
        config->pulse_on_samples > config->pulse_period_samples ||
        config->pulse_ramp_samples > config->pulse_on_samples / 2 ||
        (config->pulse_ramp_samples > 0 &&
         config->pulse_ramp_samples * 2 == config->pulse_on_samples)) {
      set_error(error, error_size,
                "pulse timing must leave a non-empty on-pulse plateau within each period");
      return -1;
    }
  }

  memset(source, 0, sizeof(*source));
  source->config = *config;
  source->rng_state = config->seed != 0
                          ? config->seed
                          : UINT64_C(0x9e3779b97f4a7c15);
  source->osc_i = cos(config->phase_rad);
  source->osc_q = sin(config->phase_rad);
  source->step_i =
      cos(2.0 * M_PI * config->frequency_hz / config->sample_rate_hz);
  source->step_q =
      sin(2.0 * M_PI * config->frequency_hz / config->sample_rate_hz);
  if (config->type == JAMMER_SOURCE_CHIRP) {
    double frequency_step =
        (config->end_frequency_hz - config->frequency_hz) /
        (double)(config->sweep_samples - 1);
    source->chirp_step_step_i =
        cos(2.0 * M_PI * frequency_step / config->sample_rate_hz);
    source->chirp_step_step_q =
        sin(2.0 * M_PI * frequency_step / config->sample_rate_hz);
  }
  if (config->type == JAMMER_SOURCE_NARROWBAND) {
    source->narrowband_pole =
        (config->sample_rate_hz - config->bandwidth_hz) /
        (config->sample_rate_hz + config->bandwidth_hz);
    source->narrowband_input_scale =
        sqrt(1.0 - source->narrowband_pole * source->narrowband_pole);
  }
  set_error(error, error_size, "");
  return 0;
}

size_t jammer_source_render_sc16(jammer_source_t *source, int16_t *buffer,
                                 size_t max_samples) {
  uint64_t remaining;
  size_t count;
  size_t k;

  if (source == NULL || buffer == NULL || max_samples == 0 ||
      source->next_sample >= source->config.total_samples) {
    return 0;
  }

  remaining = source->config.total_samples - source->next_sample;
  count = remaining > (uint64_t)max_samples ? max_samples : (size_t)remaining;

  for (k = 0; k < count; ++k) {
    uint64_t sample = source->next_sample + (uint64_t)k;
    double env = global_envelope(sample, &source->config);
    double source_env = 1.0;
    double raw_i = 0.0;
    double raw_q = 0.0;
    long qi;
    long qq;

    if (sample >= source->config.onset_sample &&
        sample < source->config.offset_sample) {
      uint64_t active_index = sample - source->config.onset_sample;
      if (source->config.type == JAMMER_SOURCE_NARROWBAND ||
          source->config.type == JAMMER_SOURCE_WIDEBAND) {
        double noise_i;
        double noise_q;
        proper_complex_gaussian(source, &noise_i, &noise_q);
        if (source->config.type == JAMMER_SOURCE_NARROWBAND) {
          double filtered_i =
              source->narrowband_pole * source->narrowband_state_i +
              source->narrowband_input_scale * noise_i;
          double filtered_q =
              source->narrowband_pole * source->narrowband_state_q +
              source->narrowband_input_scale * noise_q;
          source->narrowband_state_i = filtered_i;
          source->narrowband_state_q = filtered_q;
          raw_i = source->config.amplitude * env *
                  (filtered_i * source->osc_i - filtered_q * source->osc_q);
          raw_q = source->config.amplitude * env *
                  (filtered_i * source->osc_q + filtered_q * source->osc_i);
          advance_oscillator(source, active_index);
        } else {
          raw_i = source->config.amplitude * env * noise_i;
          raw_q = source->config.amplitude * env * noise_q;
        }
      } else {
        source_env = pulse_envelope(sample, &source->config);
        raw_i =
            source->config.amplitude * env * source_env * source->osc_i;
        raw_q =
            source->config.amplitude * env * source_env * source->osc_q;
        advance_oscillator(source, active_index);
      }

      if (source_env > 0.0) {
        ++source->source_on_samples;
      }

      if (sample >= source->config.onset_sample + source->config.ramp_samples &&
          sample < source->config.offset_sample - source->config.ramp_samples) {
        source->plateau_power +=
            (long double)raw_i * raw_i + (long double)raw_q * raw_q;
        ++source->plateau_samples;
      }
    }

    if (fabs(raw_i) > source->peak_component) {
      source->peak_component = fabs(raw_i);
    }
    if (fabs(raw_q) > source->peak_component) {
      source->peak_component = fabs(raw_q);
    }

    qi = lround(raw_i * 32767.0);
    qq = lround(raw_q * 32767.0);
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

    buffer[2 * k] = (int16_t)qi;
    buffer[2 * k + 1] = (int16_t)qq;
  }

  source->next_sample += (uint64_t)count;
  return count;
}

int jammer_source_done(const jammer_source_t *source) {
  return source != NULL && source->next_sample >= source->config.total_samples;
}

void jammer_source_get_metrics(const jammer_source_t *source,
                               jammer_source_metrics_t *metrics) {
  if (source == NULL || metrics == NULL) {
    return;
  }

  memset(metrics, 0, sizeof(*metrics));
  metrics->active_plateau_rms_full_scale =
      source->plateau_samples > 0
          ? sqrt((double)(source->plateau_power / source->plateau_samples))
          : 0.0;
  metrics->peak_component_full_scale = source->peak_component;
  metrics->clipped_components = source->clipped_components;
  metrics->rendered_samples = source->next_sample;
  metrics->plateau_samples = source->plateau_samples;
  metrics->source_on_samples = source->source_on_samples;
}
