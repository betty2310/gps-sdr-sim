#ifndef GPS_SDR_SIM_MATCHED_CODE_ALIGNMENT_H
#define GPS_SDR_SIM_MATCHED_CODE_ALIGNMENT_H

#include <stdint.h>

#include "gpssim.h"
#include "tools/matched_code_source.h"

/*
 * Copy only the synthetic alignment state used by the matched-code source.
 * Navigation words, dataBit, codeCA, and clean carrier phase intentionally do
 * not cross this boundary into the jammer renderer.
 */
static inline void matched_code_capture_channel_state(
    const channel_t *channel, int clean_gain, uint64_t sample_offset,
    matched_code_target_state_t *state) {
  state->sample_offset = sample_offset;
  state->prn = channel->prn;
  state->code_phase_chips = channel->code_phase;
  state->carrier_doppler_hz = channel->f_carr;
  state->code_rate_chips_per_s = channel->f_code;
  state->clean_gain = clean_gain;
}

#endif
