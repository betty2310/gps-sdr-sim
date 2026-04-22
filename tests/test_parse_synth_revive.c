#include "gpssim.h"

#include <stdio.h>
#include <string.h>

static int expect_true(int cond, const char *msg) {
  if (!cond) {
    fprintf(stderr, "FAIL: %s\n", msg);
    return 1;
  }
  return 0;
}

int main(void) {
  synth_config_t cfg;
  int failures = 0;

  initSynthConfig(&cfg);
  failures += expect_true(parseSynthConfig(&cfg, "1:revive") == TRUE,
                          "single revive parses");
  failures +=
      expect_true(cfg.mode[0] == SYNTH_REVIVE, "PRN 1 mode is revive");

  initSynthConfig(&cfg);
  failures += expect_true(parseSynthConfig(&cfg, "1:revive,2:revive,7:revive") ==
                              TRUE,
                          "multi revive parses");
  failures += expect_true(cfg.mode[0] == SYNTH_REVIVE &&
                              cfg.mode[1] == SYNTH_REVIVE &&
                              cfg.mode[6] == SYNTH_REVIVE,
                          "multi revive modes set");

  initSynthConfig(&cfg);
  failures += expect_true(parseSynthConfig(&cfg, "1:revive,2:20/60") == FALSE,
                          "revive mixed with az/el fails");

  initSynthConfig(&cfg);
  failures += expect_true(parseSynthConfig(&cfg, "1:revive,2:clone=5") == FALSE,
                          "revive mixed with clone fails");

  return failures == 0 ? 0 : 1;
}
