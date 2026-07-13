#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "tools/matched_code_plan.h"

static void test_target_and_timing_contract(void) {
  matched_code_plan_t plan;
  int targets[MATCHED_CODE_MAX_TARGETS];
  size_t target_count = 0;
  char error[256];

  assert(matched_code_parse_targets("7,1,32", targets, &target_count, error,
                                    sizeof(error)) == 0);
  assert(target_count == 3);
  assert(targets[0] == 7 && targets[1] == 1 && targets[2] == 32);
  assert(matched_code_parse_targets("1,1", targets, &target_count, error,
                                    sizeof(error)) != 0);
  assert(strstr(error, "duplicate") != NULL);

  assert(matched_code_plan_init(&plan, 2500000.0, 1.25, 0.25, 1.0, 0.01,
                                "7,1", 20.0, 42, error,
                                sizeof(error)) == 0);
  assert(plan.total_samples == 3125000);
  assert(plan.onset_sample == 625000);
  assert(plan.offset_sample == 2500000);
  assert(plan.ramp_samples == 25000);
  assert(plan.reference_start_sample == 650000);
  assert(plan.reference_end_sample == 2500000 - 25000);
  assert(plan.target_count == 2 && plan.target_prns[0] == 7 &&
         plan.target_prns[1] == 1);

  assert(matched_code_plan_init(&plan, 2500000.0, 1.0, 0.4, 0.6, 0.1,
                                "1", 0.0, 1, error,
                                sizeof(error)) != 0);
  assert(strstr(error, "plateau") != NULL);
}

static void test_fixed_js_and_common_headroom_gain(void) {
  matched_code_plan_t plan;
  char error[256];

  assert(matched_code_plan_init(&plan, 1000.0, 2.0, 0.0, 2.0, 0.1, "1,7",
                                20.0, 42, error, sizeof(error)) == 0);
  assert(matched_code_plan_calibrate(&plan, 100.0, 0.5, 500.0, sqrt(2.0),
                                     error, sizeof(error)) == 0);
  assert(fabs(plan.jammer_scale - 2000.0) < 1e-12);
  assert(fabs(plan.reference_js_db - 20.0) < 1e-12);
  assert(plan.common_gain == 1.0);

  plan.requested_js_db = 60.0;
  assert(matched_code_plan_calibrate(&plan, 100.0, 0.5, 500.0, sqrt(2.0),
                                     error, sizeof(error)) == 0);
  assert(plan.common_gain < 1.0);
  assert(plan.predicted_headroom_db >= 1.0 - 1e-12);
}

static void test_wide_mix_quantizes_once_and_measures_components(void) {
  matched_code_plan_t plan;
  matched_code_mix_metrics_t metrics;
  const double clean[] = {100.0, -100.0, 200.0, 50.0};
  const double jammer[] = {0.25, 0.5, -0.5, 0.25};
  int16_t output[4];

  memset(&plan, 0, sizeof(plan));
  plan.onset_sample = 0;
  plan.offset_sample = 2;
  plan.ramp_samples = 0;
  plan.jammer_scale = 2.0;
  plan.common_gain = 0.5;
  matched_code_mix_metrics_init(&metrics);
  assert(matched_code_mix_sc16(&plan, 0, clean, jammer, output, 2,
                               &metrics) == 0);
  assert(output[0] == 50 && output[1] == -50);
  assert(output[2] == 100 && output[3] == 25);
  assert(metrics.samples == 2);
  assert(metrics.plateau_samples == 2);
  assert(metrics.clipped_components == 0);
  assert(metrics.clean_power > 0.0L && metrics.jammer_power > 0.0L &&
         metrics.composite_power > 0.0L);
  assert(metrics.composite_peak == 99.5);
}

int main(void) {
  test_target_and_timing_contract();
  test_fixed_js_and_common_headroom_gain();
  test_wide_mix_quantizes_once_and_measures_components();
  return 0;
}
