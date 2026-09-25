#include "player/ubx_receiver.hpp"
#include "player/x300_live_time.hpp"
#include <cassert>
#include <cstring>
#include <iostream>
#include <limits>

using Bytes = std::vector<uint8_t>;
void put(Bytes &p, size_t at, uint32_t value, size_t n = 4) {
  for (size_t i = 0; i < n; ++i)
    p[at + i] = (value >> (i * 8)) & 255;
}
Bytes frame(uint8_t cls, uint8_t id, const Bytes &p) {
  Bytes b{0xb5, 0x62, cls, id, uint8_t(p.size()), uint8_t(p.size() >> 8)};
  b.insert(b.end(), p.begin(), p.end());
  uint8_t a = 0, c = 0;
  for (size_t j = 2; j < b.size(); ++j) {
    a += b[j];
    c += a;
  }
  b.push_back(a);
  b.push_back(c);
  return b;
}
Bytes timeFrame(int week, uint32_t ms, int32_t fraction = 0,
                uint8_t valid = 3) {
  Bytes p(16);
  put(p, 0, ms);
  put(p, 4, fraction);
  put(p, 8, week, 2);
  p[10] = 18;
  p[11] = valid;
  put(p, 12, 10);
  return frame(1, 0x20, p);
}
void feed(ubx::Decoder &d, const Bytes &b, double at) {
  d.feed(b.data(), b.size(), at);
}
template <typename F> void throws(F f, const std::string &match) {
  try {
    f();
    assert(false);
  } catch (const std::exception &e) {
    assert(std::string(e.what()).find(match) != std::string::npos);
  }
}
void identity(ubx::Decoder &d) {
  Bytes id(70);
  std::memcpy(id.data() + 40, "MOD=ZED-F9P", 11);
  feed(d, frame(10, 4, id), 1);
  d.state.connected = true;
}
void testFramingAndTime() {
  ubx::Decoder d;
  auto t = timeFrame(2437, 1, -400000, 7);
  for (auto b : t)
    d.feed(&b, 1, 100);
  assert(d.state.frames == 1 && d.state.time.valid);
  assert(std::fabs(d.state.time.gps.sec - .0006) < 1e-12);
  assert(d.state.time.leap_valid && d.state.time.accuracy_ns == 10);
  auto bad = t;
  bad.back() ^= 1;
  Bytes mixed{0x24, 0, 0xb5, 0x62, 0, 0, 0xff, 0xff};
  mixed.insert(mixed.end(), bad.begin(), bad.end());
  auto next = timeFrame(2437, 1001, -400000);
  mixed.insert(mixed.end(), next.begin(), next.end());
  feed(d, mixed, 101);
  assert(d.state.checksum_errors == 1 && d.state.malformed == 1);
  assert(d.state.frames == 2 && d.state.times.size() == 2 && d.buffered() == 0);
  feed(d, next, 101.01); // duplicate/backlogged epoch invalidates warmup
  assert(!d.state.time.valid && d.state.time_rejections == 1 &&
         d.state.times.empty());
  feed(d, timeFrame(2437, 2000), 102);
  feed(d, timeFrame(2437, 3000), 102.01);
  assert(!d.state.time.valid); // fast replay is not live time
  feed(d, timeFrame(2437, 4000, 600000), 104);
  assert(!d.state.time.valid);
  feed(d, timeFrame(-1, 0), 105);
  assert(!d.state.time.valid);
  feed(d, timeFrame(2437, 0, 0, 0), 106);
  assert(!d.state.time.valid);
  ubx::Decoder rollover;
  feed(rollover, timeFrame(2437, 604799000), 1);
  feed(rollover, timeFrame(2438, 0, -200000), 2);
  assert(rollover.state.time.valid && rollover.state.time.gps.week == 2437);
  assert(std::fabs(rollover.state.time.gps.sec - 604799.9998) < 1e-9);
  feed(rollover, timeFrame(2438, 1000, -200000), 3);
  assert(rollover.state.time.valid && rollover.state.time.gps.week == 2438);
  Bytes garbage(100000, 0xff);
  feed(rollover, garbage, 4);
  assert(rollover.buffered() < 2);
  auto truncated = timeFrame(2438, 2000);
  rollover.feed(truncated.data(), 8, 5);
  assert(rollover.state.time.gps.sec < 2);
}

// GPS PRN 3 SFRBX words recorded 2026-09-21; independent expected fields
// come from RTKLIB convbin (provenance: fixtures/f9p-lnav-prn3.json).
const std::array<std::array<uint32_t, 10>, 3> words{
    {{{583008274, 2313666836, 407896097, 2147483689, 2147483689, 2147483689,
       2147483871, 176650387, 4182512, 140502756}},
     {{583008274, 2313675388, 176228474, 2325451969, 2624768038, 2160328944,
       578153763, 32548983, 58134306, 2272821391}},
     {{583008274, 2313642856, 1073366327, 222298635, 1072925185, 2759414003,
       169626973, 2159307686, 3219821638, 2323719047}}}};
Bytes sfrbx(unsigned sf, unsigned signal = 0, unsigned version = 2) {
  Bytes p(48);
  p[1] = 3;
  p[2] = signal;
  p[4] = 10;
  p[6] = version;
  for (unsigned i = 0; i < 10; ++i)
    put(p, 8 + i * 4, words[sf - 1][i]);
  return frame(2, 0x13, p);
}
void testNavigation() {
  ubx::Decoder d;
  feed(d, timeFrame(2437, 121724000), 100);
  feed(d, sfrbx(1, 3), 100); // GPS L2C must not enter LNAV
  assert(!d.state.navigation[2].coverage);
  feed(d, sfrbx(1, 0, 1), 100);
  assert(d.state.malformed == 1);
  feed(d, sfrbx(1), 100);
  feed(d, sfrbx(2), 101);
  assert(!d.state.navigation[2].complete);
  feed(d, sfrbx(3), 102);
  auto &nav = d.state.navigation[2];
  assert(nav.complete && nav.eph.iode == 42 && nav.eph.iodc == 42);
  assert(nav.eph.toe.week == 2437 && nav.eph.toe.sec == 122400);
  const auto &e = nav.eph;
  const std::vector<double> actual{e.af0,
                                   e.af1,
                                   e.af2,
                                   double(e.iode),
                                   e.crs,
                                   e.deltan,
                                   e.m0,
                                   e.cuc,
                                   e.ecc,
                                   e.cus,
                                   e.sqrta,
                                   e.toe.sec,
                                   e.cic,
                                   e.omg0,
                                   e.cis,
                                   e.inc0,
                                   e.crc,
                                   e.aop,
                                   e.omgdot,
                                   e.idot,
                                   double(e.codeL2),
                                   double(e.toe.week),
                                   e.tgd,
                                   double(e.iodc)};
  const std::vector<double> expected{.000255572609603,
                                     -2.10320649785e-11,
                                     0,
                                     42,
                                     33.03125,
                                     3.87944730884e-9,
                                     2.04803629737,
                                     1.460313797e-6,
                                     .00691103050485,
                                     3.69921326637e-6,
                                     5153.73253632,
                                     122400,
                                     -4.28408384323e-8,
                                     .495955175187,
                                     -9.31322574615e-8,
                                     .995735265262,
                                     323.53125,
                                     1.30108598224,
                                     -7.83425489959e-9,
                                     1.03932900651e-10,
                                     0,
                                     2437,
                                     1.39698386192e-9,
                                     42};
  for (size_t n = 0; n < actual.size(); ++n)
    assert(std::fabs(actual[n] - expected[n]) <=
           std::fabs(expected[n]) * 2e-12 + 1e-17);
  assert(ubx::usableNavigation(nav, {2437, 121800}, 103));
  assert(!ubx::usableNavigation(nav, {2437, 130000}, 103));
  assert(!ubx::usableNavigation(nav, {2437, 121800}, 193));
  auto unhealthy = nav;
  unhealthy.eph.svhlth = 1;
  assert(!ubx::usableNavigation(unhealthy, {2437, 121800}, 103));
  auto wrong_issue = words[2];
  wrong_issue[9] ^= 1u << 22;
  ubx::ingestLnav(nav, wrong_issue, {2437, 121724}, 103);
  assert(!nav.complete);
  ubx::ingestLnav(nav, words[2], {2437, 121724}, 104);
  assert(nav.complete);
  // A different 1024-week era comes only from the valid receiver, not host UTC.
  ubx::Navigation future;
  for (const auto &w : words)
    ubx::ingestLnav(future, w, {3461, 121724}, 100);
  assert(future.complete && future.eph.toe.week == 3461);
  ubx::Navigation old_week;
  for (const auto &w : words)
    ubx::ingestLnav(old_week, w, {2438, 121724}, 100);
  assert(!old_week.complete); // Never rebase a previous week's ephemeris.
}
void testRawx() {
  ubx::Decoder d;
  feed(d, timeFrame(2437, 100000), 100);
  Bytes p(48);
  double tow = 100, range = 21000000, phase = 12345;
  float doppler = -500;
  std::memcpy(p.data(), &tow, 8);
  put(p, 8, 2437, 2);
  p[11] = 1;
  p[13] = 1;
  std::memcpy(p.data() + 16, &range, 8);
  std::memcpy(p.data() + 24, &phase, 8);
  std::memcpy(p.data() + 32, &doppler, 4);
  p[37] = 3;
  p[46] = 15;
  feed(d, frame(2, 0x15, p), 100);
  assert(d.state.gps_l1_observed[2] && d.state.observations.size() == 1);
  assert(d.state.observations[0].carrier_cycles == phase);
  double invalid = std::numeric_limits<double>::quiet_NaN();
  std::memcpy(p.data() + 16, &invalid, 8);
  feed(d, frame(2, 0x15, p), 101);
  assert(d.state.malformed == 1);
  std::memcpy(p.data() + 16, &range, 8);
  p[12] = 2;
  feed(d, frame(2, 0x15, p), 102);
  assert(d.state.clock_resets == 1 && !d.state.time.valid);
}
void testMapping() {
  ubx::Decoder d;
  identity(d);
  for (int n = 0; n < 3; ++n)
    feed(d, timeFrame(2437, 604797000 + n * 1000, -123456), 100 + n);
  auto plan = x300::planLiveStart(d.state, {102.09, 1.1, 102.11}, 2, .25,
                                  .00001, .00002);
  assert(plan.hardware_start == 4);
  assert(plan.gps_zero.week == 2438);
  // latest epoch + (Hstart-H(receipt)) + delivery + TX - sky
  double expected = 604798.999876544 + (4 - 1.0) + .25 - .00001 - 604800;
  assert(std::fabs(plan.gps_zero.sec - expected) < 1e-9);
  auto biased = x300::planLiveStart(d.state, {102.09, 1.1, 102.11}, 2, .75);
  assert(std::fabs(subGpsTime(biased.gps_zero, plan.gps_zero) - .50001) < 1e-9);
  assert(x300::livePlanJson(plan).find("\"absolute_gps_uncertainty_s\":null") !=
         std::string::npos);
  for (double offset : {-.0369307, 0., .0369307}) {
    auto corrected = x300::planLiveStart(d.state, {102.09, 1.1, 102.11}, 2,
                                         .25, .00001, .00002, offset);
    assert(corrected.hardware_start == plan.hardware_start);
    assert(std::fabs(subGpsTime(corrected.gps_zero, plan.gps_zero)-offset) < 1e-9);
    assert(subGpsTime(corrected.gps_zero_before_model_offset, plan.gps_zero) == 0);
    // A common epoch correction does not become a sample-rate correction.
    auto later = x300::sampleTime(corrected.gps_zero, 2500000, 2500000);
    assert(std::fabs(subGpsTime(later, corrected.gps_zero)-1) < 1e-9);
    assert(corrected.anchor.received == plan.anchor.received);
  }
  auto rollover = x300::offsetModelEpoch({2437, 604799.99}, .0369307);
  assert(rollover.week == 2438 && std::fabs(rollover.sec-.0269307) < 1e-9);
  rollover = x300::offsetModelEpoch({2438, .01}, -.0369307);
  assert(rollover.week == 2437 && std::fabs(rollover.sec-604799.9730693) < 1e-9);
  for (double invalid : {-1.01, 1.01, std::numeric_limits<double>::quiet_NaN(),
                          std::numeric_limits<double>::infinity()})
    throws([&] { x300::planLiveStart(d.state, {102.09, 1.1, 102.11}, 2,
                                    0, 0, 0, invalid); }, "model time offset");
  auto replay = d.state;
  replay.replay = true;
  throws([&] { x300::planLiveStart(replay, {102.09, 1.1, 102.11}, 2); },
         "recorded UBX");
  throws([&] { x300::planLiveStart(d.state, {105, 4, 105.01}, 2); }, "stale");
  throws([&] { x300::planLiveStart(d.state, {102, 1, 102.1}, 2); },
         "invalid live");
  d.state.times.clear();
  throws([&] { x300::planLiveStart(d.state, {102, 1, 102.01}, 2); },
         "consecutive");
}
void testGpsNow() {
  ubx::Decoder d;
  identity(d);
  for (int n = 0; n < 3; ++n)
    feed(d, timeFrame(2437, 604797000 + n * 1000, -123456), 100 + n);
  // No PVT, RAWX or SFRBX is needed for this receive-only diagnostic.
  assert(!d.state.position.valid && d.state.observations.empty());
  auto estimate = ubx::estimateGpsNow(d.state, 103.25);
  assert(estimate.gps.week == 2438);
  assert(std::fabs(estimate.gps.sec - .249876544) < 1e-9);
  assert(estimate.host_monotonic == 103.25 && estimate.solution_age == 1.25);
  auto delayed = ubx::estimateGpsNow(d.state, 103.25, .25);
  assert(std::fabs(subGpsTime(delayed.gps, estimate.gps) - .25) < 1e-9);
  assert(d.state.time.gps.week == 2437); // The receiver solution stays intact.
  x300::LiveStartPlan health_anchor;
  health_anchor.anchor = d.state.time;
  health_anchor.time_rejections = d.state.time_rejections;
  health_anchor.clock_resets = d.state.clock_resets;
  x300::checkLiveHealth(d.state, health_anchor, 103.25, estimate.gps, {1}, false);
  throws([&] {
    x300::checkLiveHealth(d.state, health_anchor, 103.25, estimate.gps, {1});
  }, "NAV-PVT"); // Ordinary live navigation keeps its stronger requirements.
  auto reset = d.state;
  ++reset.clock_resets;
  throws([&] {
    x300::checkLiveHealth(reset, health_anchor, 103.25, estimate.gps, {1}, false);
  }, "discontinuity/reset");
  auto replay = d.state;
  replay.replay = true;
  throws([&] { ubx::estimateGpsNow(replay, 103.25); }, "recorded UBX");
  auto disconnected = d.state;
  disconnected.connected = false;
  throws([&] { ubx::estimateGpsNow(disconnected, 103.25); }, "not connected");
  throws([&] {
    x300::checkLiveHealth(disconnected, health_anchor, 103.25, estimate.gps, {1}, false);
  }, "not connected");
  throws([&] { ubx::estimateGpsNow(d.state, 105); }, "stale");
  throws([&] { ubx::estimateGpsNow(d.state, 101); }, "stale");
  throws([&] { ubx::estimateGpsNow(d.state, std::numeric_limits<double>::quiet_NaN()); },
         "invalid GPS-now");
  throws([&] { ubx::estimateGpsNow(d.state, 103, -1); }, "invalid GPS-now");
  throws([&] { ubx::estimateGpsNow(d.state, 103, 10.1); }, "invalid GPS-now");
  feed(d, timeFrame(2438, 0, 0, 0), 103);
  throws([&] { ubx::estimateGpsNow(d.state, 103.25); }, "consecutive valid");
}

void testMinimumDelayEstimate() {
  ubx::Decoder d;
  identity(d);
  const double delays[] = {.04, .14, .07, .20, .09};
  for (int n = 0; n < 5; ++n)
    feed(d, timeFrame(2437, (1000 + n) * 1000, 0), 100 + n + delays[n]);
  const auto estimate = ubx::estimateGpsNow(d.state, 104.5);
  // Queuing jitter is removed; the unobservable 40 ms floor is not invented.
  assert(std::fabs(estimate.gps.sec - 1004.46) < 1e-9);
  assert(std::fabs(estimate.arrival_correction - .05) < 1e-9);
  assert(std::fabs(estimate.arrival_variation - .16) < 1e-9);
  assert(estimate.observations == 5);
  const auto corrected = ubx::estimateGpsNow(d.state, 104.5, .04);
  assert(std::fabs(corrected.gps.sec - 1004.5) < 1e-9);
  const auto plan = x300::planLiveStart(d.state, {104.49, 4.5, 104.51}, 2, .04);
  assert(plan.hardware_start == 7 && std::fabs(plan.gps_zero.sec - 1007) < 1e-9);
  // A constant shift of all arrivals is indistinguishable from delivery bias.
  auto shifted = d.state;
  shifted.time.received += 1;
  for (auto &t : shifted.times) t.received += 1;
  assert(std::fabs(ubx::estimateGpsNow(shifted, 105.5).gps.sec -
                   estimate.gps.sec) < 1e-9);
  assert(ubx::timeEstimateJson(estimate).find(
      "\"constant_delivery_bias_known\":false") != std::string::npos);
  // Expired low-delay samples must not influence a later window.
  for (int n = 5; n < 11; ++n)
    feed(d, timeFrame(2437, (1000 + n) * 1000, 0), 100 + n + .10);
  const auto later = ubx::estimateGpsNow(d.state, 110.5);
  assert(std::fabs(later.gps.sec - 1010.4) < 1e-9);
}

void testReceiverDiagnostics() {
  ubx::Decoder d;
  auto empty = ubx::snapshotJson(d.state, 0);
  assert(empty.find("\"rf_monitor\":null") != std::string::npos);
  assert(empty.find("\"time_pulse\":null") != std::string::npos);
  assert(empty.find("\"time_mark\":null") != std::string::npos);
  Bytes uid{1, 0, 0, 0, 0, 1, 0xa2, 0xff, 0x10};
  feed(d, frame(0x27, 3, uid), 1);
  assert(d.state.unique_id == "0001a2ff10");
  uid[0] = 2; // Do not mistake a different ID format for the HPG 1.32 format.
  feed(d, frame(0x27, 3, uid), 2);
  assert(d.state.malformed == 1 && d.state.unique_id == "0001a2ff10");

  Bytes rf(52);
  rf[1] = 2;
  rf[5] = 3;
  rf[6] = 1; // Unknown antenna supervision is not "OK".
  rf[7] = 2; // Unknown power is not "ON".
  put(rf, 16, 300, 2);
  put(rf, 18, 8191, 2);
  rf[20] = 200;
  rf[28] = 1;
  feed(d, frame(10, 0x38, rf), 3);
  assert(d.state.rf_present && d.state.rf_blocks.size() == 2);
  const auto &r = d.state.rf_blocks[0];
  assert(r.jamming_state == 3 && r.antenna_status == 1 && r.antenna_power == 2);
  assert(r.noise_per_ms == 300 && r.agc_count == 8191 &&
         r.cw_suppression == 200);
  rf[1] = 3; // Count/length mismatch cannot replace the previous report.
  feed(d, frame(10, 0x38, rf), 4);
  assert(d.state.malformed == 2 && d.state.rf_received == 3);

  Bytes pulse(16);
  put(pulse, 0, 604799999);
  put(pulse, 4, 0x80000000); // TIM-TP: half a millisecond, not nanoseconds.
  put(pulse, 8, uint32_t(-123));
  put(pulse, 12, 2437, 2);
  pulse[14] = 0x11; // UTC time base and invalid qErr must remain visible.
  pulse[15] = 0x30;
  feed(d, frame(0x0d, 1, pulse), 5);
  assert(d.state.time_pulse.present && d.state.time_pulse.flags == 0x11);
  assert(d.state.time_pulse.tow_sub_ms == 0x80000000);
  assert(d.state.time_pulse.quantization_error_ps == -123);
  assert(!d.state.time.valid && d.state.times.empty());

  Bytes mark(28);
  mark[1] = 0xc8; // New rising edge, valid time, GNSS time base.
  put(mark, 2, 65535, 2);
  put(mark, 4, 2438, 2);
  put(mark, 6, 2437, 2);
  put(mark, 8, 0);
  put(mark, 12, 500000); // TIM-TM2 uses ns, unlike TIM-TP's 2^-32 ms units.
  put(mark, 16, 604799999);
  put(mark, 20, 999999);
  put(mark, 24, 30);
  feed(d, frame(0x0d, 3, mark), 6);
  assert(d.state.time_mark.present && d.state.time_mark.rising_week == 2438);
  assert(d.state.time_mark.falling_week == 2437 &&
         d.state.time_mark.count == 65535);
  assert(d.state.time_mark.rising_sub_ns == 500000);
  auto json = ubx::snapshotJson(d.state, 7);
  assert(json.find("\"hardware_edge_association_verified\":false") !=
         std::string::npos);
  assert(json.find("\"tow_sub_ms_units_2_neg_32\":2147483648") !=
         std::string::npos);
  assert(json.find("\"rising_tow_sub_ns\":500000") != std::string::npos);
  // Receiving valid pulse/EXTINT labels alone must not supply live GPS time.
  assert(!d.state.time.valid && d.state.times.empty());
  put(mark, 12, 1000000);
  feed(d, frame(0x0d, 3, mark), 8);
  assert(d.state.malformed == 3 && d.state.time_mark.received == 6);
  put(pulse, 0, 604800000);
  feed(d, frame(0x0d, 1, pulse), 9);
  assert(d.state.malformed == 4 && d.state.time_pulse.received == 5);
  Bytes pvt(92);
  pvt[20] = 3;
  pvt[21] = 1;
  feed(d, frame(1, 7, pvt), 10);
  pvt[21] = 0;
  feed(d, frame(1, 7, pvt), 11);
  assert(d.state.pvt_epochs == 2 && d.state.valid_3d_epochs == 1);
}
int main() {
  testFramingAndTime();
  testNavigation();
  testRawx();
  testMapping();
  testGpsNow();
  testMinimumDelayEstimate();
  testReceiverDiagnostics();
  std::cout
      << "UBX parser, LNAV, validity, reset and estimated epoch tests passed\n";
}
