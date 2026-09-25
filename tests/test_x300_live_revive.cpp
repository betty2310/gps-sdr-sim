// Exercise the production start planner, reference-health policy, renderer and
// manifest writer together. The renamed CLI main is never called; all radio
// operations use FakeRadio, so this executable cannot discover or transmit RF.
#define main x300tx_program_main
#include "player/x300tx.cpp"
#undef main
#include "fake_x300_radio.hpp"
#include "fake_f9p_pps.hpp"
#include <cassert>
#include <iostream>

static LiveRun timeReference(FakeRadio &radio, gpstime_t wanted_zero,
                             double model_offset) {
  LiveRun live;
  live.endpoint = "test-reference";
  live.time_only = true;
  live.prns = {1};
  live.delivery = .04;
  live.tx_path = .00002;
  live.sky_path = .00003;
  live.model_time_offset = model_offset;
  live.model_time_offset_set = true;
  live.initial.connected = true;
  live.initial.identity = "MOD=ZED-F9P";
  const double start = ceil(radio.now().get_real_secs() + .25);
  gpstime_t at_receipt = wanted_zero;
  at_receipt.sec -= start - radio.now().get_real_secs() + live.delivery +
                    live.tx_path - live.sky_path + live.model_time_offset;
  for (int n = -2; n <= 0; ++n) {
    ubx::TimeSolution time;
    time.gps = at_receipt;
    time.gps.sec += n;
    time.gps = x300::sampleTime(time.gps, 0, 1);
    time.received = radio.monotonic() + n;
    time.valid = true;
    time.accuracy_ns = 10;
    live.initial.times.push_back(time);
  }
  live.initial.time = live.initial.times.back();
  live.initial.last_received = live.initial.time.received;
  return live;
}

static void testClockQueries() {
  FakeRadio radio;
  const auto sync = x300::synchronize(radio);
  (void)sync;
  radio.query_delays = {.060, .008, .004, .001, .003, .002, .006, .005};
  auto observation = x300::observeClock(radio);
  assert(radio.query_delays.empty());
  assert(fabs(observation.width() - .001) < 1e-9);
  assert(fabs(observation.midpoint() - observation.hardware - radio.origin) < 1e-9);
  radio.query_delays.assign(8, .051);
  bool failed = false;
  try { x300::observeClock(radio); }
  catch (const std::exception &) { failed = true; }
  assert(failed);
}

static void testReviveTransmission(bool continuous, bool late, bool disconnect,
                                  bool pps = false) {
  FakeRadio radio;
  const auto sync = x300::synchronize(radio);
  radio.rate = 1000000;
  radio.partial_limit = 364;
  radio.fifo_capacity = 65156;
  auto live = timeReference(radio, {2347, 439229.98},
                            continuous ? -.0369307 : .0369307);
  FakeF9pPps source;
  if (pps) {
    live.gps_pps = true;
    live.pps_options.host_utc_bound = .2;
    live.delivery = live.sky_path = live.model_time_offset = 0;
    live.model_time_offset_set = false;
    live.tx_path = .98; // Exercise an explicit model path correction at NAV rollover.
    radio.utc_offset = source.unixUtc(0);
    radio.on_advance = [&](double at) {
      source.advance(at); live.initial = source.decoder.state;
    };
    radio.advance(0);
  }
  const gpstime_t zero = prepareLiveStart(live, &radio, .25);
  assert(live.plan.valid && live.plan.hardware_start > radio.now().get_real_secs());
  assert(live.plan.pps.verified == pps);
  assert(zero.week == 2347 && fabs(zero.sec - 439229.98) < 1e-9);
  assert(fabs(subGpsTime(zero, live.plan.gps_zero_before_model_offset) -
              live.model_time_offset) < 1e-9);
  assert(!live.initial.position.valid && live.initial.observations.empty());
  for (const auto &nav : live.initial.navigation) assert(!nav.eph.vflg);
  // The ordinary navigation source still requires PVT/RAWX/SFRBX.
  live.time_only = false;
  bool refused = false;
  try { live.checkHealth(radio.monotonic(), zero); }
  catch (const std::exception &) { refused = true; }
  assert(refused);
  live.time_only = true;

  ephem_t ephemerides[EPHEM_ARRAY_SIZE][MAX_SAT]{};
  ephem_t active[MAX_SAT]{};
  ionoutc_t ionoutc{};
  ionoutc.enable = TRUE;
  const int sets = readRinexNavAll(ephemerides, &ionoutc,
                                  "tests/fixtures/brdc0030.25n");
  assert(sets == 1);
  synth_config_t synth;
  synth_ephem_store_t store;
  attack_config_t attack;
  initSynthConfig(&synth);
  initSynthEphemStore(&store);
  initAttackConfig(&attack);
  assert(parsePartialPrns(&attack, "1"));
  assert(parseSynthConfig(&synth, "1:revive"));
  assert(rendersOnlyRevivedPrns(attack, synth));
  // This source has no current set; the generated revive overlay must still
  // support finite/continuous timed transmission and navigation refresh.
  assert(!shouldAdvanceEphSet(ephemerides[0][0].toc, zero));
  double llh[3] = {0, 0, 0};
  llh2xyz(llh, xyz[0]);
  assert(refreshSyntheticEphemerisSet(&store, ephemerides, sets,
      ephemerides[0], &ionoutc, &synth, xyz[0], quantizeSynthReferenceTime(zero)));
  overlaySyntheticEphemerisSet(active, ephemerides[0], &synth, &store);
  channel_t channels[MAX_CHAN]{};
  std::fill(allocatedSat, allocatedSat + MAX_SAT, -1);
  allocateChannel(channels, active, ionoutc, zero, xyz[0], 0, &attack, &synth, nullptr);
  assert(allocatedSat[0] >= 0);
  for (int sv = 1; sv < MAX_SAT; ++sv) assert(allocatedSat[sv] == -1);
  assert(subGpsTime(active[0].toe, ephemerides[0][0].toe) > 3000);

  int gain[MAX_CHAN]{};
  double antenna[37];
  std::fill(antenna, antenna + 37, 1.0);
  unsigned noise[MAX_SAT]{};
  uint64_t generated = 0, clipped = 0;
  int64_t nav_frame = x300::navFrame(zero);
  int ephemeris_index = 0;
  const uint64_t limit = continuous ? UINT64_MAX : 125000;
  auto render = [&](x300::Frame &frame) {
    if (generated == limit) return false;
    if (late && generated == 0) radio.advance(1.1);
    const auto begin = x300::sampleTime(zero, generated, radio.rate);
    const auto count = x300::capAtNavBoundary(begin, radio.rate,
        std::min<uint64_t>(100000, limit - generated));
    const auto end = x300::sampleTime(zero, generated + count, radio.rate);
    frame.iq.resize(count * 2);
    generateEpoch(frame.iq.data(), count, channels, gain, active, &ionoutc,
        end, TRUE, subGpsTime(end, begin), 1 / radio.rate, FALSE, 32,
        antenna, FALSE, &attack, noise, 1, &clipped);
    generated += count;
    if (x300::navFrame(end) != nav_frame) {
      refreshNavState(channels, ephemerides, ephemerides, sets, &ephemeris_index,
          active, &store, &synth, &ionoutc, end, 0, false, &attack, nullptr);
      nav_frame = x300::navFrame(end);
    }
    return true;
  };
  if (disconnect) live.initial.connected = false;
  x300::TxStats stats;
  const bool ok = x300::transmit(radio, sync, radio.rate, .25, 364, 2, render,
      [&] { return continuous && radio.received.size() / 2 >= 350000; }, stats,
      [] {}, live.plan.hardware_start,
      [&] { live.checkPps(radio); live.checkHealth(radio.monotonic(),
              x300::sampleTime(zero, stats.accepted, radio.rate)); });
  if (late || disconnect) {
    assert(!ok && !stats.send_attempted && radio.send_calls == 0);
    assert(stats.failure.find(late ? "deadline missed" : "not connected") !=
           std::string::npos);
    return;
  }
  assert(ok && stats.burst_ack && clipped == 0);
  assert(stats.prestart_backpressure_waits > 0);
  assert(stats.accepted == radio.received.size() / 2);
  assert(continuous ? stats.interrupted : stats.accepted == limit);
  assert(radio.metadata.front().has_time_spec);
  assert(radio.metadata.front().time_spec.get_real_secs() == live.plan.hardware_start);
  assert(nav_frame > x300::navFrame(zero));
  assert(std::any_of(radio.received.begin(), radio.received.end(),
                    [](int16_t sample) { return sample != 0; }));
  const auto manifest = std::filesystem::temp_directory_path() /
      ("x300-revive-test-" + std::to_string(getpid()) + ".json");
  assert(writeRunManifest(manifest.string(), continuous ? "stopped" : "completed",
      zero, radio.rate, "external", "external", sync, stats, clipped,
      "tests/fixtures/brdc0030.25n", "test", xyz[0], &live));
  std::ifstream input(manifest);
  const std::string json((std::istreambuf_iterator<char>(input)), {});
  assert(json.find(pps ? "\"epoch_association\": \"ublox_tim_tp_hardware_pps\""
                      : "\"epoch_association\": \"ublox_receiver_time_estimate\"") != std::string::npos);
  if (pps) assert(json.find("\"pps_epoch_association_verified\": true") != std::string::npos);
  assert(json.find("\"navigation_source\": \"frozen_rinex\"") != std::string::npos);
  assert(json.find("\"live_start_plan\": {\"valid\":true") != std::string::npos);
  assert(json.find("\"gps_alignment_verified\": false") != std::string::npos);
  assert(json.find("\"selected_prns\": [1]") != std::string::npos);
  assert(json.find(pps ? "\"model_time_offset_source\": \"none\""
                      : "\"model_time_offset_source\": \"operator_supplied\"") != std::string::npos);
  assert(json.find("\"model_offset_changes_hardware_deadline\":false") != std::string::npos);
  assert(json.find("\"prestart_backpressure_waits\": " +
                   std::to_string(stats.prestart_backpressure_waits)) != std::string::npos);
  std::filesystem::remove(manifest);
}

int main() {
  testClockQueries();
  testReviveTransmission(false, false, false);
  testReviveTransmission(true, false, false);
  testReviveTransmission(false, true, false);
  testReviveTransmission(false, false, true);
  testReviveTransmission(false, false, false, true);
  testReviveTransmission(true, false, false, true);
  testReviveTransmission(false, true, false, true);
  testReviveTransmission(false, false, true, true);
  std::cout << "F9P time-only revive finite/continuous TX, deadline and manifest tests passed\n";
}
