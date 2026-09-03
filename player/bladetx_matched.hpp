/*
 * bladeRF adapter for the matched-code jammer-only pipeline.
 *
 * This file is included by bladetx.cpp after the shared GPS simulation
 * helpers are defined. It deliberately keeps the waveform contract aligned
 * with x300tx while isolating libbladeRF-specific timed transmission.
 */

static bool captureMatchedTargetStates(
    const matched_code_plan_t &plan, const channel_t chan[MAX_CHAN],
    const int gain[MAX_CHAN], uint64_t sample_offset,
    matched_code_target_state_t states[MATCHED_CODE_MAX_TARGETS],
    std::string *error) {
  for (size_t index = 0; index < plan.target_count; ++index) {
    int prn = plan.target_prns[index];
    int channel = allocatedSat[prn - 1];
    if (channel < 0 || channel >= MAX_CHAN || chan[channel].prn != prn ||
        gain[channel] <= 0) {
      std::ostringstream message;
      message << "target PRN " << prn
              << " is not allocated with positive clean gain at sample "
              << sample_offset;
      *error = message.str();
      return false;
    }
    matched_code_capture_channel_state(&chan[channel], gain[channel],
                                       sample_offset, &states[index]);
  }
  return true;
}

static bool validateMatchedTargetUsability(
    const matched_code_plan_t &plan, const ephem_t active_ephemeris[MAX_SAT],
    const synth_config_t *synth_config, gpstime_t boundary_time,
    double elevation_mask, uint64_t sample_offset, std::string *error) {
  for (size_t index = 0; index < plan.target_count; ++index) {
    int prn = plan.target_prns[index];
    int satellite = prn - 1;
    bool synthetic = synth_config != nullptr && synth_config->enabled &&
                     synth_config->mode[satellite] != SYNTH_NONE;
    double azimuth_elevation[2];
    int usable =
        synthetic
            ? (active_ephemeris[satellite].vflg == 1 ? TRUE : FALSE)
            : checkSatVisibility(active_ephemeris[satellite], boundary_time,
                                 xyz[0], elevation_mask, azimuth_elevation);
    if (usable != TRUE) {
      std::ostringstream message;
      message << "target PRN " << prn
              << " is not geometrically usable at sample " << sample_offset;
      *error = message.str();
      return false;
    }
  }
  return true;
}

static void writeTrajectoryHeader(std::ofstream &trajectory,
                                  const matched_code_plan_t &plan,
                                  gpstime_t start_time) {
  trajectory << std::setprecision(17);
  trajectory << "# schema=gps-sdr-sim.target-trajectory.v1\n";
  trajectory << "# sample_rate_hz=" << plan.sample_rate_hz << "\n";
  trajectory << "# epoch_cadence_samples="
             << static_cast<uint64_t>(
                    std::llround(plan.sample_rate_hz * EPOCH_TARGET_SEC))
             << "\n";
  trajectory << "# gps_week=" << start_time.week << "\n";
  trajectory << "# gps_tow=" << start_time.sec << "\n";
  trajectory << "# boundary=first_sample\n";
  trajectory << "sample_offset,prn,code_phase_chips,carrier_doppler_hz,"
                "code_rate_chips_per_s,clean_gain\n";
}

static void writeTrajectoryStates(std::ofstream &trajectory,
                                  const matched_code_target_state_t *states,
                                  size_t state_count) {
  for (size_t index = 0; index < state_count; ++index) {
    const matched_code_target_state_t &state = states[index];
    trajectory << state.sample_offset << ',' << state.prn << ','
               << state.code_phase_chips << ',' << state.carrier_doppler_hz
               << ',' << state.code_rate_chips_per_s << ',' << state.clean_gain
               << '\n';
  }
}

static std::string jsonEscape(const std::string &value) {
  std::ostringstream escaped;
  for (unsigned char character : value) {
    switch (character) {
    case '"': escaped << "\\\""; break;
    case '\\': escaped << "\\\\"; break;
    case '\b': escaped << "\\b"; break;
    case '\f': escaped << "\\f"; break;
    case '\n': escaped << "\\n"; break;
    case '\r': escaped << "\\r"; break;
    case '\t': escaped << "\\t"; break;
    default:
      if (character < 0x20) {
        escaped << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                << static_cast<int>(character) << std::dec;
      } else {
        escaped << static_cast<char>(character);
      }
    }
  }
  return escaped.str();
}

static uint64_t updateFnv1a64Sc16(uint64_t hash, const int16_t *samples,
                                  size_t sample_count) {
  for (size_t index = 0; index < sample_count * 2; ++index) {
    uint16_t value = static_cast<uint16_t>(samples[index]);
    hash ^= static_cast<uint8_t>(value & 0xffU);
    hash *= FNV1A64_PRIME;
    hash ^= static_cast<uint8_t>((value >> 8) & 0xffU);
    hash *= FNV1A64_PRIME;
  }
  return hash;
}

static std::string fnv1a64Hex(uint64_t value) {
  std::ostringstream output;
  output << std::hex << std::setfill('0') << std::setw(16) << value;
  return output.str();
}

static bool initializeMatchedJammerPlan(matched_code_plan_t *plan,
                                        double sample_rate_hz,
                                        const std::string &target_prns,
                                        double amplitude, uint64_t phase_seed,
                                        char *error, size_t error_size) {
  if (plan == nullptr || !std::isfinite(sample_rate_hz) ||
      sample_rate_hz <= 0.0 || !std::isfinite(amplitude) || amplitude <= 0.0 ||
      amplitude > 1.0) {
    snprintf(error, error_size,
             "sample rate and matched-code amplitude in (0, 1] are required");
    return false;
  }
  memset(plan, 0, sizeof(*plan));
  plan->sample_rate_hz = sample_rate_hz;
  plan->phase_seed = phase_seed;
  plan->jammer_scale = amplitude;
  if (matched_code_parse_targets(target_prns.c_str(), plan->target_prns,
                                 &plan->target_count, error,
                                 error_size) != 0)
    return false;
  long double exact = static_cast<long double>(sample_rate_hz) *
                      static_cast<long double>(MATCHED_DRY_RUN_VALIDATION_SEC);
  long double rounded = roundl(exact);
  if (rounded <= 0.0L || rounded > static_cast<long double>(UINT64_MAX) ||
      fabsl(exact - rounded) > 1.0e-6L) {
    snprintf(error, error_size,
             "the 100 ms validation window is not an integral sample count");
    return false;
  }
  plan->total_samples = static_cast<uint64_t>(rounded);
  plan->onset_sample = 0;
  plan->offset_sample = plan->total_samples;
  plan->ramp_samples = 0;
  plan->reference_start_sample = 0;
  plan->reference_end_sample = plan->total_samples;
  plan->jammer_component_bound =
      amplitude * sqrt(static_cast<double>(plan->target_count));
  plan->predicted_composite_bound = plan->jammer_component_bound;
  plan->predicted_headroom_db =
      20.0 * log10(1.0 / plan->predicted_composite_bound);
  if (plan->predicted_headroom_db + 1.0e-12 < MATCHED_MIN_HEADROOM_DB) {
    snprintf(error, error_size,
             "matched-code amplitude leaves %.3f dB predicted headroom; at "
             "least %.1f dB is required for %zu target(s)",
             plan->predicted_headroom_db, MATCHED_MIN_HEADROOM_DB,
             plan->target_count);
    return false;
  }
  return true;
}

static matched_code_source_config_t
matchedSourceConfig(const matched_code_plan_t &plan, uint64_t sample_limit) {
  matched_code_source_config_t config{};
  config.sample_rate_hz = plan.sample_rate_hz;
  config.total_samples = sample_limit;
  config.onset_sample = 0;
  config.offset_sample = sample_limit;
  config.ramp_samples = 0;
  config.amplitude = plan.jammer_scale;
  config.phase_seed = plan.phase_seed;
  config.target_count = plan.target_count;
  for (size_t index = 0; index < plan.target_count; ++index)
    config.target_prns[index] = plan.target_prns[index];
  return config;
}

static bool writeMatchedManifestAtomic(
    const MatchedCodeOptions &options, const matched_code_plan_t &plan,
    const MatchedCodeRunResult &result, const char *navfile,
    const std::string &ephemeris_sha256, const std::string &scenario_sha256,
    gpstime_t sample_zero, const double reference_xyz[3],
    double requested_rate_hz, const char *device_address, int requested_txvga1,
    int requested_txvga2, int prebuffer_count, long long tx_delay_cal_ns,
    double gps_time_ppm, bool trimble_mode) {
  std::string temporary_path = options.manifest_path + ".tmp";
  std::ofstream manifest(temporary_path, std::ios::out | std::ios::trunc);
  if (!manifest)
    return false;

  matched_code_source_config_t source_config =
      matchedSourceConfig(plan, plan.total_samples);
  matched_code_source_t phase_source;
  char source_error[128];
  bool phases_available =
      matched_code_source_init(&phase_source, &source_config, source_error,
                               sizeof(source_error)) == 0;

  manifest << std::setprecision(17);
  manifest << "{\n";
  manifest << "  \"schema\": \"gps-sdr-sim.bladetx-matched-code.v2\",\n";
  manifest << "  \"tool\": \"bladetx\",\n";
  manifest << "  \"status\": \"" << jsonEscape(result.status) << "\",\n";
  manifest << "  \"failure_reason\": ";
  if (result.failure_reason.empty())
    manifest << "null";
  else
    manifest << "\"" << jsonEscape(result.failure_reason) << "\"";
  manifest << ",\n  \"exit_status\": " << result.exit_status << ",\n";
  manifest << "  \"acceptance_scope\": \"transmitter-only; RF waveform "
              "fidelity, propagation, receiver-input power, RF code "
              "alignment, and receiver behavior were not measured\",\n";
  manifest << "  \"rf_output\": {\"contains\": "
              "\"matched_code_interference_only\", "
              "\"clean_gps_transmitted\": false},\n";
  manifest << "  \"safety\": {\"controlled_rf_only\": true, "
              "\"controlled_rf_confirmed\": "
           << (options.controlled_rf_confirmed ? "true" : "false")
           << ", \"calibration_id\": ";
  if (options.calibration_id.empty())
    manifest << "null";
  else
    manifest << "\"" << jsonEscape(options.calibration_id) << "\"";
  manifest << "},\n";
  manifest << "  \"scenario\": {\"schema\": "
              "\"gps-sdr-sim.bladetx-frozen-scenario.v1\", "
              "\"sha256\": \""
           << jsonEscape(scenario_sha256) << "\", \"ephemeris_path\": \""
           << jsonEscape(navfile != nullptr ? navfile : "")
           << "\", \"ephemeris_sha256\": \"" << jsonEscape(ephemeris_sha256)
           << "\", \"reference_ecef_m\": [" << reference_xyz[0] << ", "
           << reference_xyz[1] << ", " << reference_xyz[2]
           << "], \"sample_zero_gps_week\": " << sample_zero.week
           << ", \"sample_zero_gps_tow\": " << sample_zero.sec
           << ", \"internal_alignment_source\": "
              "\"synthetic_clean_state_discarded_not_transmitted\", "
              "\"requested_target_prns\": [";
  for (size_t index = 0; index < plan.target_count; ++index)
    manifest << (index == 0 ? "" : ", ") << plan.target_prns[index];
  manifest << "], \"selected_target_prns\": [";
  for (size_t index = 0; index < plan.target_count; ++index)
    manifest << (index == 0 ? "" : ", ") << plan.target_prns[index];
  manifest << "], \"startup_target_allocation_passed\": "
           << (result.target_allocation_passed ? "true" : "false") << "},\n";
  manifest << "  \"trajectory\": {\"schema\": "
              "\"gps-sdr-sim.target-trajectory.v1\", \"path\": \""
           << jsonEscape(options.trajectory_path) << "\", \"sha256\": \""
           << jsonEscape(result.trajectory_sha256) << "\"},\n";
  manifest << "  \"waveform\": {\"taxonomy\": "
              "\"navigation-data-free-gps-l1-ca-matched-code\", "
              "\"data_symbol_policy\": \"constant_positive\", "
              "\"carrier_phase_policy\": \"independent_deterministic\", "
              "\"phase_seed\": "
           << plan.phase_seed << ", \"equal_component_weight\": "
           << 1.0 / sqrt((double)plan.target_count)
           << ", \"target_count_normalization\": \"sqrt_n\", "
              "\"initial_carrier_phases_rad\": [";
  for (size_t index = 0; index < plan.target_count; ++index) {
    double phase = phases_available
                       ? matched_code_source_initial_phase_rad(&phase_source,
                                                               index)
                       : 0.0;
    manifest << (index == 0 ? "" : ", ") << phase;
  }
  manifest << "], \"output_amplitude_full_scale\": " << options.amplitude
           << ", \"predicted_peak_full_scale\": "
           << plan.predicted_composite_bound
           << ", \"predicted_headroom_db\": " << plan.predicted_headroom_db
           << ", \"jammer_iq_fnv1a64\": ";
  if (result.quantized_samples == 0)
    manifest << "null";
  else
    manifest << "\"" << fnv1a64Hex(result.jammer_iq_fnv1a64) << "\"";
  manifest << "},\n";
  manifest << "  \"sample_contract\": {\"requested_rate_hz\": "
           << requested_rate_hz << ", \"actual_rate_hz\": "
           << result.actual_rate_hz
           << ", \"format\": \"sc16_le\", \"device_format\": "
              "\"SC16_Q11_META\", \"device_conversion\": "
              "\"q15_divide_16_round_saturate_q11\", "
              "\"device_scale\": 0.0625, \"iq_order\": \"IQ\", "
              "\"continuous\": true, \"planned_samples\": null, "
              "\"dry_run_validation_samples\": "
           << plan.total_samples << ", \"internal_alignment_samples\": "
           << result.internal_alignment_samples
           << ", \"rendered_jammer_samples\": "
           << result.rendered_jammer_samples
           << ", \"quantized_jammer_samples\": " << result.quantized_samples
           << ", \"sent_jammer_samples\": " << result.sent_samples << "},\n";
  manifest << "  \"activation\": {\"start_sample\": 0, "
              "\"stop_condition\": \"SIGINT_or_SIGTERM\"},\n";
  manifest << "  \"timing\": {\"start_mode\": \""
           << jsonEscape(result.start_mode.empty()
                             ? (trimble_mode ? "trimble_time_tag"
                                             : "explicit_gps_time")
                             : result.start_mode)
           << "\", \"gps_week\": " << sample_zero.week
           << ", \"gps_tow\": " << sample_zero.sec
           << ", \"tx_delay_calibration_ns\": " << tx_delay_cal_ns
           << ", \"gps_time_ppm\": " << gps_time_ppm
           << ", \"prebuffer_epochs\": " << prebuffer_count
           << ", \"prebuffer_samples\": "
           << (uint64_t)prebuffer_count *
                  (uint64_t)llround(plan.sample_rate_hz * EPOCH_TARGET_SEC)
           << ", \"device_start_timestamp_samples\": "
           << result.device_start_timestamp_samples
           << ", \"start_margin_met\": "
           << (result.start_margin_met ? "true" : "false") << "},\n";
  manifest << "  \"hardware\": {\"device_type\": \""
           << jsonEscape(result.device_type) << "\", \"product\": \""
           << jsonEscape(result.device_product) << "\", \"serial\": \""
           << jsonEscape(result.device_serial) << "\", \"address\": \""
           << jsonEscape(device_address)
           << "\", \"tx_channel_count\": 1, \"selected_channel\": 0, "
              "\"requested_center_frequency_hz\": "
           << TX_FREQUENCY << ", \"actual_center_frequency_hz\": "
           << result.actual_frequency_hz << ", \"requested_txvga1_db\": "
           << requested_txvga1 << ", \"actual_txvga1_db\": "
           << result.actual_txvga1_db << ", \"requested_txvga2_db\": "
           << requested_txvga2 << ", \"actual_txvga2_db\": "
           << result.actual_txvga2_db << "},\n";
  manifest << "  \"measurements\": {\"jammer_rms_full_scale\": "
           << result.source_metrics.active_plateau_rms
           << ", \"jammer_peak_full_scale\": "
           << result.source_metrics.peak_component
           << ", \"clipped_components\": "
           << result.source_metrics.clipped_components
           << ", \"underflows\": " << result.underflows
           << ", \"sequence_errors\": " << result.sequence_errors
           << ", \"time_errors\": " << result.time_errors
           << ", \"operator_stopped\": "
           << (result.interrupted ? "true" : "false") << "}\n";
  manifest << "}\n";
  manifest.close();
  if (!manifest ||
      std::rename(temporary_path.c_str(), options.manifest_path.c_str()) != 0) {
    std::remove(temporary_path.c_str());
    return false;
  }
  return true;
}

struct MatchedSimulationState {
  channel_t channels[MAX_CHAN];
  int gains[MAX_CHAN];
  ephem_t active_ephemeris[MAX_SAT];
  synth_ephem_store_t synthetic_ephemeris;
  int ephemeris_index = 0;
  epoch_plan_t epoch_plan{};
  uint64_t sample_offset = 0;
  gpstime_t receiver_time{};
};

static void initializeMatchedSimulationState(
    MatchedSimulationState *state, const channel_t channels[MAX_CHAN],
    const int gains[MAX_CHAN], const ephem_t active_ephemeris[MAX_SAT],
    const synth_ephem_store_t *synthetic_ephemeris, int ephemeris_index,
    const epoch_plan_t *epoch_plan, gpstime_t start_time) {
  memcpy(state->channels, channels, sizeof(state->channels));
  memcpy(state->gains, gains, sizeof(state->gains));
  memcpy(state->active_ephemeris, active_ephemeris,
         sizeof(state->active_ephemeris));
  state->synthetic_ephemeris = *synthetic_ephemeris;
  state->ephemeris_index = ephemeris_index;
  state->epoch_plan = *epoch_plan;
  state->sample_offset = 0;
  state->receiver_time = start_time;
}

static int nextMatchedFrameSampleCount(MatchedSimulationState *state,
                                       uint64_t total_samples) {
  uint64_t remaining = total_samples - state->sample_offset;
  int epoch_samples = nextEpochSampleCount(&state->epoch_plan);
  return remaining < (uint64_t)epoch_samples ? (int)remaining : epoch_samples;
}

static bool prepareMatchedFrame(
    MatchedSimulationState *state, const matched_code_plan_t &plan,
    uint64_t sample_limit, gpstime_t sample_zero, ionoutc_t *ionoutc,
    double gps_time_ppm, double delt, int path_loss_enable, int fixed_gain,
    double ant_pat[37], const synth_config_t *synth_config,
    double elevation_mask,
    matched_code_target_state_t states[MATCHED_CODE_MAX_TARGETS],
    int *sample_count, std::string *error) {
  gpstime_t block_start =
      getGpsTimeAtSampleOffset(sample_zero, (long long)state->sample_offset,
                               plan.sample_rate_hz, gps_time_ppm);
  *sample_count = nextMatchedFrameSampleCount(state, sample_limit);
  if (!validateMatchedTargetUsability(plan, state->active_ephemeris,
                                      synth_config, block_start, elevation_mask,
                                      state->sample_offset, error))
    return false;
  gpstime_t block_end = getGpsTimeAtSampleOffset(
      sample_zero, (long long)(state->sample_offset + *sample_count),
      plan.sample_rate_hz, gps_time_ppm);
  state->receiver_time = block_end;
  prepareEpoch(state->channels, state->gains, state->active_ephemeris, ionoutc,
               block_end, subGpsTime(block_end, block_start), delt,
               path_loss_enable, fixed_gain, ant_pat, FALSE, nullptr);
  return captureMatchedTargetStates(plan, state->channels, state->gains,
                                    state->sample_offset, states, error);
}

static void refreshMatchedStateIfNeeded(
    MatchedSimulationState *state, ephem_t eph[][MAX_SAT],
    const ephem_t synth_source[][MAX_SAT], int neph, ionoutc_t *ionoutc,
    const synth_config_t *synth_config, const attack_config_t *attack_config,
    double elevation_mask, const int *required_prns, double sample_rate_hz) {
  uint64_t refresh_samples =
      (uint64_t)llround(SYNTH_EPHEM_REFRESH_SEC * sample_rate_hz);
  if (refresh_samples == 0 || state->sample_offset == 0 ||
      state->sample_offset % refresh_samples != 0)
    return;
  int rtcm_alive = FALSE;
  refreshNavState(state->channels, eph, synth_source, neph,
                  &state->ephemeris_index, state->active_ephemeris,
                  &state->synthetic_ephemeris, synth_config, ionoutc,
                  state->receiver_time, elevation_mask, FALSE, &rtcm_alive,
                  nullptr, attack_config, required_prns);
}

static bool renderMatchedFrame(
    MatchedSimulationState *state, matched_code_source_t *source,
    const matched_code_plan_t &plan, uint64_t sample_limit,
    gpstime_t sample_zero, ionoutc_t *ionoutc, double gps_time_ppm,
    double delt, int path_loss_enable, int fixed_gain, double ant_pat[37],
    const synth_config_t *synth_config, double elevation_mask,
    std::vector<double> *alignment_discard,
    std::vector<int16_t> *jammer_output, std::ofstream *trajectory,
    MatchedCodeRunResult *result, int *sample_count, std::string *error) {
  matched_code_target_state_t states[MATCHED_CODE_MAX_TARGETS];
  char source_error[256];
  if (!prepareMatchedFrame(state, plan, sample_limit, sample_zero, ionoutc,
                           gps_time_ppm, delt, path_loss_enable, fixed_gain,
                           ant_pat, synth_config, elevation_mask, states,
                           sample_count, error))
    return false;
  if (trajectory != nullptr)
    writeTrajectoryStates(*trajectory, states, plan.target_count);
  if (matched_code_source_set_epoch(source, states, plan.target_count,
                                    source_error, sizeof(source_error)) != 0) {
    *error = source_error;
    return false;
  }
  alignment_discard->resize((size_t)*sample_count * 2);
  jammer_output->resize((size_t)*sample_count * 2);
  renderCleanEpochWide(alignment_discard->data(), *sample_count, state->channels,
                       state->gains, delt);
  if (matched_code_source_render_sc16(source, jammer_output->data(),
                                      (size_t)*sample_count) !=
      (size_t)*sample_count) {
    *error = "shared matched-code renderer stopped before the epoch ended";
    return false;
  }
  result->jammer_iq_fnv1a64 =
      updateFnv1a64Sc16(result->jammer_iq_fnv1a64, jammer_output->data(),
                        (size_t)*sample_count);
  result->internal_alignment_samples += (uint64_t)*sample_count;
  result->rendered_jammer_samples += (uint64_t)*sample_count;
  result->quantized_samples += (uint64_t)*sample_count;
  matched_code_source_get_metrics(source, &result->source_metrics);
  state->sample_offset += (uint64_t)*sample_count;
  return true;
}

static bool runMatchedPreflight(
    matched_code_plan_t *plan, MatchedCodeRunResult *result,
    const MatchedCodeOptions &options,
    const channel_t initial_channels[MAX_CHAN],
    const int initial_gains[MAX_CHAN],
    const ephem_t initial_active_ephemeris[MAX_SAT],
    const synth_ephem_store_t *initial_synthetic_ephemeris,
    int initial_ephemeris_index, const epoch_plan_t *initial_epoch_plan,
    ephem_t eph[][MAX_SAT], const ephem_t synth_source[][MAX_SAT], int neph,
    gpstime_t sample_zero, ionoutc_t *ionoutc,
    const synth_config_t *synth_config, const attack_config_t *attack_config,
    double elevation_mask, const int *required_prns, double gps_time_ppm,
    double delt, int path_loss_enable, int fixed_gain, double ant_pat[37],
    std::string *error) {
  MatchedSimulationState state;
  matched_code_target_state_t states[MATCHED_CODE_MAX_TARGETS];
  int saved_allocated[MAX_SAT];
  std::vector<double> alignment_discard;
  std::string temporary = options.trajectory_path + ".tmp";
  std::ofstream trajectory(temporary, std::ios::out | std::ios::trunc);
  if (!trajectory) {
    *error = "cannot create target trajectory artifact";
    return false;
  }
  writeTrajectoryHeader(trajectory, *plan, sample_zero);
  memcpy(saved_allocated, allocatedSat, sizeof(saved_allocated));
  initializeMatchedSimulationState(
      &state, initial_channels, initial_gains, initial_active_ephemeris,
      initial_synthetic_ephemeris, initial_ephemeris_index, initial_epoch_plan,
      sample_zero);
  while (state.sample_offset < plan->total_samples) {
    int sample_count;
    if (!prepareMatchedFrame(&state, *plan, plan->total_samples, sample_zero,
                             ionoutc, gps_time_ppm, delt, path_loss_enable,
                             fixed_gain, ant_pat, synth_config, elevation_mask,
                             states, &sample_count, error)) {
      memcpy(allocatedSat, saved_allocated, sizeof(saved_allocated));
      return false;
    }
    writeTrajectoryStates(trajectory, states, plan->target_count);
    alignment_discard.resize((size_t)sample_count * 2);
    renderCleanEpochWide(alignment_discard.data(), sample_count, state.channels,
                         state.gains, delt);
    state.sample_offset += (uint64_t)sample_count;
    refreshMatchedStateIfNeeded(&state, eph, synth_source, neph, ionoutc,
                                synth_config, attack_config, elevation_mask,
                                required_prns, plan->sample_rate_hz);
  }
  memcpy(allocatedSat, saved_allocated, sizeof(saved_allocated));
  trajectory.close();
  if (!trajectory ||
      std::rename(temporary.c_str(), options.trajectory_path.c_str()) != 0) {
    std::remove(temporary.c_str());
    *error = "cannot finalize target trajectory artifact";
    return false;
  }
  char hash[SHA256_HEX_SIZE];
  if (sha256_file_hex(options.trajectory_path.c_str(), hash) != 0) {
    *error = "cannot checksum target trajectory artifact";
    return false;
  }
  result->trajectory_sha256 = hash;
  result->target_allocation_passed = true;
  return true;
}

static bool runMatchedDryRender(
    const matched_code_plan_t &plan, MatchedCodeRunResult *result,
    const channel_t initial_channels[MAX_CHAN],
    const int initial_gains[MAX_CHAN],
    const ephem_t initial_active_ephemeris[MAX_SAT],
    const synth_ephem_store_t *initial_synthetic_ephemeris,
    int initial_ephemeris_index, const epoch_plan_t *initial_epoch_plan,
    ephem_t eph[][MAX_SAT], const ephem_t synth_source[][MAX_SAT], int neph,
    gpstime_t sample_zero, ionoutc_t *ionoutc,
    const synth_config_t *synth_config, const attack_config_t *attack_config,
    double elevation_mask, const int *required_prns, double gps_time_ppm,
    double delt, int path_loss_enable, int fixed_gain, double ant_pat[37],
    std::string *error) {
  MatchedSimulationState state;
  int saved_allocated[MAX_SAT];
  matched_code_source_config_t config =
      matchedSourceConfig(plan, plan.total_samples);
  matched_code_source_t source;
  char source_error[256];
  std::vector<double> alignment_discard;
  std::vector<int16_t> jammer_output;
  if (matched_code_source_init(&source, &config, source_error,
                               sizeof(source_error)) != 0) {
    *error = source_error;
    return false;
  }
  result->internal_alignment_samples = 0;
  result->rendered_jammer_samples = 0;
  result->quantized_samples = 0;
  result->jammer_iq_fnv1a64 = FNV1A64_OFFSET_BASIS;
  result->source_metrics = {};
  memcpy(saved_allocated, allocatedSat, sizeof(saved_allocated));
  initializeMatchedSimulationState(
      &state, initial_channels, initial_gains, initial_active_ephemeris,
      initial_synthetic_ephemeris, initial_ephemeris_index, initial_epoch_plan,
      sample_zero);
  while (state.sample_offset < plan.total_samples) {
    int sample_count;
    if (!renderMatchedFrame(
            &state, &source, plan, plan.total_samples, sample_zero, ionoutc,
            gps_time_ppm, delt, path_loss_enable, fixed_gain, ant_pat,
            synth_config, elevation_mask, &alignment_discard, &jammer_output,
            nullptr, result, &sample_count, error)) {
      memcpy(allocatedSat, saved_allocated, sizeof(saved_allocated));
      return false;
    }
    refreshMatchedStateIfNeeded(&state, eph, synth_source, neph, ionoutc,
                                synth_config, attack_config, elevation_mask,
                                required_prns, plan.sample_rate_hz);
    if (result->source_metrics.clipped_components > 0) {
      *error = "unexpected SC16 clipping during dry-run render";
      memcpy(allocatedSat, saved_allocated, sizeof(saved_allocated));
      return false;
    }
  }
  memcpy(allocatedSat, saved_allocated, sizeof(saved_allocated));
  matched_code_source_get_metrics(&source, &result->source_metrics);
  return matched_code_source_done(&source) &&
         result->internal_alignment_samples == plan.total_samples &&
         result->rendered_jammer_samples == plan.total_samples &&
         result->quantized_samples == plan.total_samples;
}

struct BladeMatchedFrame {
  std::vector<int16_t> device_samples;
  size_t sample_count = 0;
};

static void convertMatchedSc16ToBladeQ11(const std::vector<int16_t> &sc16,
                                         std::vector<int16_t> *q11) {
  q11->resize(sc16.size());
  for (size_t index = 0; index < sc16.size(); ++index) {
    long value = lrint((double)sc16[index] / 16.0);
    if (value > 2047)
      value = 2047;
    else if (value < -2048)
      value = -2048;
    (*q11)[index] = (int16_t)value;
  }
}

static bool runMatchedTransmitter(
    const matched_code_plan_t &plan, const MatchedCodeOptions &options,
    MatchedCodeRunResult *result, struct bladerf *dev,
    double requested_start_delay_seconds, double trimble_tag_monotonic,
    double trimble_tag_lead_seconds, double trimble_start_offset_seconds,
    int prebuffer_count, const channel_t initial_channels[MAX_CHAN],
    const int initial_gains[MAX_CHAN],
    const ephem_t initial_active_ephemeris[MAX_SAT],
    const synth_ephem_store_t *initial_synthetic_ephemeris,
    int initial_ephemeris_index, const epoch_plan_t *initial_epoch_plan,
    ephem_t eph[][MAX_SAT], const ephem_t synth_source[][MAX_SAT], int neph,
    gpstime_t sample_zero, ionoutc_t *ionoutc,
    const synth_config_t *synth_config, const attack_config_t *attack_config,
    double elevation_mask, const int *required_prns, double gps_time_ppm,
    double delt, int path_loss_enable, int fixed_gain, double ant_pat[37],
    std::string *error) {
  const uint64_t continuous_limit = std::numeric_limits<uint64_t>::max();
  const double blade_queue_lead_seconds = 0.3;
  MatchedSimulationState state;
  int saved_allocated[MAX_SAT];
  matched_code_source_config_t config =
      matchedSourceConfig(plan, continuous_limit);
  matched_code_source_t source;
  char source_error[256];
  std::vector<double> alignment_discard;
  std::vector<int16_t> jammer_output;
  std::deque<BladeMatchedFrame> queue;
  std::string temporary = options.trajectory_path + ".tmp";
  std::ofstream trajectory(temporary, std::ios::out | std::ios::trunc);
  bool fatal = false;

  if (dev == nullptr) {
    *error = "bladeRF device is not open";
    return false;
  }
  if (!trajectory) {
    *error = "cannot create live target trajectory artifact";
    return false;
  }
  writeTrajectoryHeader(trajectory, plan, sample_zero);
  if (matched_code_source_init(&source, &config, source_error,
                               sizeof(source_error)) != 0) {
    *error = source_error;
    return false;
  }

  result->internal_alignment_samples = 0;
  result->rendered_jammer_samples = 0;
  result->quantized_samples = 0;
  result->sent_samples = 0;
  result->jammer_iq_fnv1a64 = FNV1A64_OFFSET_BASIS;
  result->source_metrics = {};
  memcpy(saved_allocated, allocatedSat, sizeof(saved_allocated));
  initializeMatchedSimulationState(
      &state, initial_channels, initial_gains, initial_active_ephemeris,
      initial_synthetic_ephemeris, initial_ephemeris_index, initial_epoch_plan,
      sample_zero);
  installSignalHandlers();

  for (int buffered = 0; buffered < prebuffer_count && !stop_requested;
       ++buffered) {
    BladeMatchedFrame frame;
    int sample_count;
    if (!renderMatchedFrame(
            &state, &source, plan, continuous_limit, sample_zero, ionoutc,
            gps_time_ppm, delt, path_loss_enable, fixed_gain, ant_pat,
            synth_config, elevation_mask, &alignment_discard, &jammer_output,
            &trajectory, result, &sample_count, error)) {
      memcpy(allocatedSat, saved_allocated, sizeof(saved_allocated));
      return false;
    }
    frame.sample_count = (size_t)sample_count;
    convertMatchedSc16ToBladeQ11(jammer_output, &frame.device_samples);
    queue.push_back(std::move(frame));
    refreshMatchedStateIfNeeded(&state, eph, synth_source, neph, ionoutc,
                                synth_config, attack_config, elevation_mask,
                                required_prns, plan.sample_rate_hz);
  }
  if (queue.empty()) {
    *error = stop_requested ? "interrupted during prebuffer"
                            : "prebuffer produced no samples";
    result->interrupted = stop_requested != 0;
    memcpy(allocatedSat, saved_allocated, sizeof(saved_allocated));
    return false;
  }
  if (result->source_metrics.clipped_components > 0) {
    *error = "unexpected SC16 clipping during prebuffer";
    memcpy(allocatedSat, saved_allocated, sizeof(saved_allocated));
    return false;
  }

  double total_remaining = requested_start_delay_seconds;
  if (trimble_tag_monotonic >= 0.0) {
    double planned_delay =
        trimble_tag_lead_seconds + trimble_start_offset_seconds;
    double elapsed = getMonotonicSeconds() - trimble_tag_monotonic;
    total_remaining = planned_delay - elapsed;
    fprintf(stderr,
            "[TRIMBLE] Matched-code preflight+prebuffer elapsed %.3f ms; "
            "remaining start margin %.3f ms\n",
            elapsed * 1000.0, total_remaining * 1000.0);
  }
  if (total_remaining < TX_START_LEAD_MIN_SEC) {
    *error = "calibrated bladeRF timed start became stale during preflight";
    memcpy(allocatedSat, saved_allocated, sizeof(saved_allocated));
    return false;
  }

  double wait_seconds = total_remaining - blade_queue_lead_seconds;
  if (wait_seconds > 0.05) {
    fprintf(stderr,
            "[TX] Waiting %.3f s before arming bladeRF matched-code TX ...\n",
            wait_seconds);
    struct timespec wait_time;
    wait_time.tv_sec = (time_t)floor(wait_seconds);
    wait_time.tv_nsec =
        (long)((wait_seconds - floor(wait_seconds)) * 1.0e9);
    nanosleep(&wait_time, nullptr);
  }

  double device_lead = blade_queue_lead_seconds;
  if (trimble_tag_monotonic >= 0.0) {
    double planned_delay =
        trimble_tag_lead_seconds + trimble_start_offset_seconds;
    device_lead = planned_delay -
                  (getMonotonicSeconds() - trimble_tag_monotonic);
  }
  if (device_lead < TX_START_LEAD_MIN_SEC) {
    *error = "bladeRF timed start became stale before device timestamp latch";
    memcpy(allocatedSat, saved_allocated, sizeof(saved_allocated));
    return false;
  }

  bladerf_timestamp device_now = 0;
  int tx_status = bladerf_get_timestamp(dev, BLADERF_TX, &device_now);
  if (tx_status != 0) {
    *error = std::string("cannot read bladeRF TX timestamp: ") +
             bladerf_strerror(tx_status);
    memcpy(allocatedSat, saved_allocated, sizeof(saved_allocated));
    return false;
  }
  bladerf_timestamp lead_samples =
      (bladerf_timestamp)llround(device_lead * plan.sample_rate_hz);
  bladerf_timestamp start_timestamp = device_now + lead_samples;
  result->device_start_timestamp_samples = (double)start_timestamp;
  result->start_margin_met = device_lead >= TX_START_LEAD_MIN_SEC;
  fprintf(stderr,
          "[TX] Matched-code jammer-only start in %.3f ms at bladeRF "
          "timestamp %llu with %zu prebuffered frame(s)\n",
          device_lead * 1000.0, (unsigned long long)start_timestamp,
          queue.size());

  bool first_frame = true;
  while (!queue.empty() && !fatal && !stop_requested) {
    BladeMatchedFrame &frame = queue.front();
    struct bladerf_metadata metadata;
    memset(&metadata, 0, sizeof(metadata));
    if (first_frame) {
      metadata.flags = BLADERF_META_FLAG_TX_BURST_START;
      metadata.timestamp = start_timestamp;
    }
    tx_status = bladerf_sync_tx(dev, frame.device_samples.data(),
                                (unsigned int)frame.sample_count, &metadata,
                                TIMEOUT_MS);
    if (tx_status != 0) {
      if (tx_status == BLADERF_ERR_TIMEOUT)
        ++result->underflows;
      else
        ++result->sequence_errors;
      *error = std::string("bladeRF matched-code TX failed: ") +
               bladerf_strerror(tx_status);
      fatal = true;
      break;
    }
    if ((metadata.status & BLADERF_META_STATUS_UNDERRUN) != 0) {
      ++result->underflows;
      *error = "bladeRF reported a matched-code TX underrun";
      fatal = true;
      break;
    }
    result->sent_samples += (uint64_t)frame.sample_count;
    first_frame = false;
    queue.pop_front();

    if (!fatal && !stop_requested) {
      BladeMatchedFrame next;
      int sample_count;
      if (!renderMatchedFrame(
              &state, &source, plan, continuous_limit, sample_zero, ionoutc,
              gps_time_ppm, delt, path_loss_enable, fixed_gain, ant_pat,
              synth_config, elevation_mask, &alignment_discard, &jammer_output,
              &trajectory, result, &sample_count, error)) {
        fatal = true;
        break;
      }
      next.sample_count = (size_t)sample_count;
      convertMatchedSc16ToBladeQ11(jammer_output, &next.device_samples);
      queue.push_back(std::move(next));
      refreshMatchedStateIfNeeded(&state, eph, synth_source, neph, ionoutc,
                                  synth_config, attack_config, elevation_mask,
                                  required_prns, plan.sample_rate_hz);
      if (result->source_metrics.clipped_components > 0) {
        *error = "unexpected SC16 clipping during live render";
        fatal = true;
      }
    }
  }

  if (!first_frame) {
    struct bladerf_metadata end_metadata;
    memset(&end_metadata, 0, sizeof(end_metadata));
    end_metadata.flags = BLADERF_META_FLAG_TX_BURST_END;
    int16_t zero[2] = {0, 0};
    int end_status = bladerf_sync_tx(dev, zero, 1, &end_metadata, TIMEOUT_MS);
    if (end_status != 0 && !fatal) {
      *error = std::string("bladeRF end-of-burst failed: ") +
               bladerf_strerror(end_status);
      fatal = true;
    }
  }

  result->interrupted = stop_requested != 0;
  matched_code_source_get_metrics(&source, &result->source_metrics);
  memcpy(allocatedSat, saved_allocated, sizeof(saved_allocated));
  trajectory.close();
  if (!trajectory ||
      std::rename(temporary.c_str(), options.trajectory_path.c_str()) != 0) {
    *error = "cannot finalize live target trajectory artifact";
    fatal = true;
  } else {
    char hash[SHA256_HEX_SIZE];
    if (sha256_file_hex(options.trajectory_path.c_str(), hash) != 0) {
      *error = "cannot checksum live target trajectory artifact";
      fatal = true;
    } else {
      result->trajectory_sha256 = hash;
    }
  }
  if (error->empty()) {
    if (result->underflows > 0)
      *error = "bladeRF reported a TX underrun";
    else if (result->sequence_errors > 0)
      *error = "bladeRF reported a TX sequence error";
    else if (result->source_metrics.clipped_components > 0)
      *error = "the matched-code jammer clipped during live rendering";
    else if (!result->interrupted)
      *error = "continuous matched-code source ended before operator stop";
    else if (result->sent_samples == 0)
      *error = "operator stop occurred before any jammer samples were sent";
  }
  return !fatal && result->interrupted && result->sent_samples > 0 &&
         result->underflows == 0 && result->sequence_errors == 0 &&
         result->time_errors == 0 &&
         result->source_metrics.clipped_components == 0;
}
