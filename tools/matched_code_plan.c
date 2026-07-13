#include "matched_code_plan.h"

#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void set_error(char *error, size_t error_size, const char *format, ...) {
  va_list args;

  if (error == NULL || error_size == 0)
    return;
  va_start(args, format);
  vsnprintf(error, error_size, format, args);
  va_end(args);
}

static int seconds_to_samples(double seconds, double sample_rate_hz,
                              uint64_t *samples) {
  long double exact;
  long double rounded;

  if (!isfinite(seconds) || seconds < 0.0 || !isfinite(sample_rate_hz) ||
      sample_rate_hz <= 0.0)
    return -1;
  exact = (long double)seconds * (long double)sample_rate_hz;
  rounded = roundl(exact);
  if (rounded < 0.0L || rounded > (long double)UINT64_MAX)
    return -1;
  *samples = (uint64_t)rounded;
  return 0;
}

int matched_code_parse_targets(const char *text,
                               int targets[MATCHED_CODE_MAX_TARGETS],
                               size_t *target_count, char *error,
                               size_t error_size) {
  const char *cursor;
  size_t count = 0;

  if (text == NULL || text[0] == '\0' || targets == NULL ||
      target_count == NULL) {
    set_error(error, error_size, "target PRN list is required");
    return -1;
  }
  cursor = text;
  while (*cursor != '\0') {
    char *end = NULL;
    long prn;
    size_t i;

    errno = 0;
    prn = strtol(cursor, &end, 10);
    if (errno != 0 || end == cursor || prn < 1 || prn > 32 ||
        count == MATCHED_CODE_MAX_TARGETS) {
      set_error(error, error_size,
                "target PRNs must be ordered values in 1..32");
      return -1;
    }
    for (i = 0; i < count; ++i) {
      if (targets[i] == prn) {
        set_error(error, error_size, "duplicate target PRN %ld", prn);
        return -1;
      }
    }
    targets[count++] = (int)prn;
    if (*end == '\0')
      break;
    if (*end != ',' || end[1] == '\0') {
      set_error(error, error_size, "invalid target PRN list");
      return -1;
    }
    cursor = end + 1;
  }
  *target_count = count;
  return 0;
}

int matched_code_plan_init(matched_code_plan_t *plan, double sample_rate_hz,
                           double duration_seconds, double onset_seconds,
                           double offset_seconds, double ramp_seconds,
                           const char *target_prns, double requested_js_db,
                           uint64_t phase_seed, char *error,
                           size_t error_size) {
  uint64_t one_second_samples;
  uint64_t plateau_end;

  if (plan == NULL || !isfinite(requested_js_db)) {
    set_error(error, error_size, "plan and finite digital J/S are required");
    return -1;
  }
  memset(plan, 0, sizeof(*plan));
  plan->sample_rate_hz = sample_rate_hz;
  plan->requested_js_db = requested_js_db;
  plan->phase_seed = phase_seed;
  if (matched_code_parse_targets(target_prns, plan->target_prns,
                                 &plan->target_count, error,
                                 error_size) != 0 ||
      seconds_to_samples(duration_seconds, sample_rate_hz,
                         &plan->total_samples) != 0 ||
      seconds_to_samples(onset_seconds, sample_rate_hz,
                         &plan->onset_sample) != 0 ||
      seconds_to_samples(offset_seconds, sample_rate_hz,
                         &plan->offset_sample) != 0 ||
      seconds_to_samples(ramp_seconds, sample_rate_hz,
                         &plan->ramp_samples) != 0 ||
      seconds_to_samples(1.0, sample_rate_hz, &one_second_samples) != 0) {
    if (error != NULL && error[0] == '\0')
      set_error(error, error_size,
                "duration or envelope cannot be represented as samples");
    return -1;
  }
  if (plan->total_samples == 0 || plan->onset_sample >= plan->offset_sample ||
      plan->offset_sample > plan->total_samples ||
      plan->ramp_samples * 2 >=
          plan->offset_sample - plan->onset_sample) {
    set_error(error, error_size,
              "timing must contain a non-empty active plateau");
    return -1;
  }

  plan->reference_start_sample = plan->onset_sample + plan->ramp_samples;
  plateau_end = plan->offset_sample - plan->ramp_samples;
  plan->reference_end_sample =
      plan->reference_start_sample + one_second_samples < plateau_end
          ? plan->reference_start_sample + one_second_samples
          : plateau_end;
  return 0;
}

int matched_code_plan_calibrate(matched_code_plan_t *plan,
                                double clean_reference_rms,
                                double jammer_reference_rms,
                                double clean_component_bound,
                                double jammer_component_bound, char *error,
                                size_t error_size) {
  const double sc16_limit = 32767.0;
  const double target_peak = sc16_limit * pow(10.0, -1.0 / 20.0);
  double requested_voltage_ratio;
  double accepted_peak;

  if (plan == NULL || !isfinite(clean_reference_rms) ||
      clean_reference_rms <= 0.0 || !isfinite(jammer_reference_rms) ||
      jammer_reference_rms <= 0.0 || !isfinite(clean_component_bound) ||
      clean_component_bound <= 0.0 || !isfinite(jammer_component_bound) ||
      jammer_component_bound <= 0.0) {
    set_error(error, error_size,
              "positive clean/jammer reference powers and bounds are required");
    return -1;
  }

  requested_voltage_ratio = pow(10.0, plan->requested_js_db / 20.0);
  plan->clean_reference_rms = clean_reference_rms;
  plan->jammer_reference_rms = jammer_reference_rms;
  plan->jammer_scale =
      clean_reference_rms * requested_voltage_ratio / jammer_reference_rms;
  plan->reference_js_db =
      20.0 * log10(plan->jammer_scale * jammer_reference_rms /
                   clean_reference_rms);
  plan->clean_component_bound = clean_component_bound;
  plan->jammer_component_bound = jammer_component_bound;
  plan->predicted_composite_bound =
      clean_component_bound + plan->jammer_scale * jammer_component_bound;
  plan->common_gain = plan->predicted_composite_bound > target_peak
                          ? target_peak / plan->predicted_composite_bound
                          : 1.0;
  accepted_peak = plan->predicted_composite_bound * plan->common_gain;
  plan->predicted_headroom_db = 20.0 * log10(sc16_limit / accepted_peak);
  return 0;
}

void matched_code_mix_metrics_init(matched_code_mix_metrics_t *metrics) {
  if (metrics != NULL)
    memset(metrics, 0, sizeof(*metrics));
}

int matched_code_mix_sc16(const matched_code_plan_t *plan,
                          uint64_t first_sample_offset,
                          const double *clean_interleaved_iq,
                          const double *jammer_interleaved_iq,
                          int16_t *output_interleaved_iq, size_t sample_count,
                          matched_code_mix_metrics_t *metrics) {
  size_t i;

  if (plan == NULL || clean_interleaved_iq == NULL ||
      jammer_interleaved_iq == NULL || output_interleaved_iq == NULL ||
      metrics == NULL || !isfinite(plan->jammer_scale) ||
      !isfinite(plan->common_gain) || plan->common_gain <= 0.0)
    return -1;

  for (i = 0; i < sample_count; ++i) {
    uint64_t sample = first_sample_offset + (uint64_t)i;
    double clean_i = clean_interleaved_iq[2 * i];
    double clean_q = clean_interleaved_iq[2 * i + 1];
    double jammer_i = plan->jammer_scale * jammer_interleaved_iq[2 * i];
    double jammer_q = plan->jammer_scale * jammer_interleaved_iq[2 * i + 1];
    double mixed_i = plan->common_gain * (clean_i + jammer_i);
    double mixed_q = plan->common_gain * (clean_q + jammer_q);
    long quantized_i = lround(mixed_i);
    long quantized_q = lround(mixed_q);

    if (fabs(mixed_i) > metrics->composite_peak)
      metrics->composite_peak = fabs(mixed_i);
    if (fabs(mixed_q) > metrics->composite_peak)
      metrics->composite_peak = fabs(mixed_q);
    if (sample >= plan->onset_sample + plan->ramp_samples &&
        sample < plan->offset_sample - plan->ramp_samples) {
      metrics->clean_power +=
          (long double)clean_i * clean_i + (long double)clean_q * clean_q;
      metrics->jammer_power +=
          (long double)jammer_i * jammer_i + (long double)jammer_q * jammer_q;
      metrics->composite_power +=
          (long double)mixed_i * mixed_i + (long double)mixed_q * mixed_q;
      ++metrics->plateau_samples;
    }
    if (quantized_i > 32767) {
      quantized_i = 32767;
      ++metrics->clipped_components;
    } else if (quantized_i < -32768) {
      quantized_i = -32768;
      ++metrics->clipped_components;
    }
    if (quantized_q > 32767) {
      quantized_q = 32767;
      ++metrics->clipped_components;
    } else if (quantized_q < -32768) {
      quantized_q = -32768;
      ++metrics->clipped_components;
    }
    output_interleaved_iq[2 * i] = (int16_t)quantized_i;
    output_interleaved_iq[2 * i + 1] = (int16_t)quantized_q;
    ++metrics->samples;
  }
  return 0;
}

double matched_code_mix_achieved_js_db(
    const matched_code_mix_metrics_t *metrics) {
  if (metrics == NULL || metrics->clean_power <= 0.0L ||
      metrics->jammer_power <= 0.0L)
    return NAN;
  return 10.0 * log10((double)(metrics->jammer_power /
                              metrics->clean_power));
}
