#include "fake_x300_radio.hpp"
#include <cassert>
#include <iostream>

template <typename F> void mustThrow(F call, const std::string &message) {
  try {
    call();
    assert(false && "expected failure");
  } catch (const std::exception &e) {
    assert(std::string(e.what()).find(message) != std::string::npos);
  }
}

void testTimelineAndNavigationBoundaries() {
  auto end = x300::sampleTime({2400, 604799.98}, 250000, 2500000);
  assert(end.week == 2401 && fabs(end.sec - 0.08) < 1e-9);
  end = x300::sampleTime({2400, 604799.98}, 2500000ULL * 86400 * 15, 2500000);
  assert(end.week == 2403 && fabs(end.sec - 86399.98) < 1e-8);
  const auto fractional = x300::sampleTime({2400, 29.98}, 250001, 2500001);
  assert(fabs(fractional.sec - (29.98 + 250001.0 / 2500001)) < 1e-12);
  assert(x300::navFrame({2400, 604799.98}) + 1 == x300::navFrame({2401, 0}));
  for (gpstime_t zero : {gpstime_t{2400, 29.98}, gpstime_t{2400, 604799.98}}) {
    uint64_t count = x300::capAtNavBoundary(zero, 2500000, 250000);
    assert(count >= 50000 && count <= 50001);
    auto boundary = x300::sampleTime(zero, count, 2500000);
    assert(x300::navFrame(boundary) == x300::navFrame(zero) + 1);
    assert(x300::capAtNavBoundary(boundary, 2500000, 250000) == 250000);
  }
  mustThrow([] { x300::sampleTime({2400, 0}, 1, 0); }, "sample timeline");
}

void testPpsContract() {
  FakeRadio radio;
  auto sync = x300::synchronize(radio);
  assert(sync.verified && sync.last_pps_seconds == 1 && radio.latch_calls == 1);
  radio.advance(0.25);
  x300::checkHealth(radio, sync);
  radio.advance(2);
  x300::checkHealth(radio,
                    sync); // slow prebuffer is allowed while PPS advances
  radio.origin -= 2;       // device time jumps, even by whole seconds
  mustThrow([&] { x300::checkHealth(radio, sync); }, "jumped");

  FakeRadio absent;
  absent.pps_present = false;
  mustThrow([&] { x300::synchronize(absent, 0.1); }, "no PPS edge");
  assert(absent.latch_calls == 0 && absent.send_calls == 0);
  FakeRadio unlocked;
  unlocked.locked = false;
  mustThrow([&] { x300::synchronize(unlocked); }, "unlocked");
  FakeRadio wrong_latch;
  wrong_latch.bad_latch = true;
  mustThrow([&] { x300::synchronize(wrong_latch); }, "latch");
  FakeRadio missing_second;
  missing_second.lose_pps_after_latch = true;
  mustThrow([&] { x300::synchronize(missing_second, 0.1); }, "no PPS edge");
  // Enough time to see the first edge, then fail if no second edge follows.
  FakeRadio disappearing;
  disappearing.lose_pps_after_latch = true;
  mustThrow([&] { x300::synchronize(disappearing, 1.1); }, "disappeared");
  FakeRadio lost;
  auto last = x300::synchronize(lost);
  lost.frozen_pps = lost.lastPps().get_real_secs();
  lost.pps_present = false;
  lost.advance(1.6);
  mustThrow([&] { x300::checkHealth(lost, last); }, "PPS");
  assert(lost.send_calls == 0);
}

x300::Render numberedSource(int frames, int samples = 10) {
  return [=, index = 0](x300::Frame &frame) mutable {
    if (index == frames)
      return false;
    frame.iq.resize(samples * 2);
    for (int j = 0; j < samples * 2; ++j)
      frame.iq[j] = static_cast<int16_t>(index * samples * 2 + j);
    ++index;
    return true;
  };
}

void testFiniteBurstAndShortSends() {
  for (size_t capacity : {1, 3, 50}) {
    FakeRadio radio;
    auto sync = x300::synchronize(radio);
    x300::TxStats stats;
    auto numbered = numberedSource(6);
    unsigned renders = 0;
    auto render = [&](x300::Frame &frame) {
      // Startup computation can take longer than the requested TX lead.
      if (renders++ < std::min<size_t>(capacity, 6))
        radio.advance(0.2);
      return numbered(frame);
    };
    assert(x300::transmit(
        radio, sync, 100, 0.25, 7, capacity, render, [] { return false; },
        stats,
        [&] {
          assert(stats.generated == std::min<size_t>(capacity, 6) * 10);
          assert(stats.start_seconds - radio.now().get_real_secs() >= 0.25);
          assert(radio.received.empty());
        }));
    assert(stats.generated == 60 && stats.accepted == 60);
    assert(stats.burst_ack && stats.start_margin_met && stats.failure.empty());
    assert(radio.latch_calls == 1 && radio.ended);
    for (size_t i = 0; i < radio.received.size(); ++i)
      assert(radio.received[i] == static_cast<int16_t>(i));
    assert(radio.metadata.front().has_time_spec &&
           radio.metadata.front().start_of_burst);
    for (size_t i = 1; i < radio.metadata.size(); ++i)
      assert(!radio.metadata[i].has_time_spec &&
             !radio.metadata[i].start_of_burst);
  }
}

void testTransportFailures() {
  for (unsigned event :
       {Event::EVENT_CODE_UNDERFLOW, Event::EVENT_CODE_UNDERFLOW_IN_PACKET,
        Event::EVENT_CODE_SEQ_ERROR, Event::EVENT_CODE_SEQ_ERROR_IN_BURST,
        Event::EVENT_CODE_TIME_ERROR}) {
    FakeRadio radio;
    auto sync = x300::synchronize(radio);
    radio.inject_event = event;
    x300::TxStats stats;
    assert(!x300::transmit(
        radio, sync, 100, 0.25, 7, 2, numberedSource(10), [] { return false; },
        stats));
    assert(stats.underflows + stats.sequence_errors + stats.time_errors == 1);
    assert(stats.accepted < 100 &&
           stats.accepted_at_first_error == stats.accepted);
    // A fault becomes available after the first packet. Do not submit the
    // rest of this render frame before checking the async queue.
    assert(stats.accepted == radio.partial_limit);
    assert(stats.first_event_has_time && stats.burst_ack &&
           radio.latch_calls == 1);
    assert(stats.failure.find("continuity") != std::string::npos);
  }
  FakeRadio zero;
  auto sync = x300::synchronize(zero);
  zero.zero_send = true;
  x300::TxStats stats;
  assert(!x300::transmit(
      zero, sync, 100, 0.25, 7, 2, numberedSource(5), [] { return false; },
      stats));
  assert(stats.accepted == 0 &&
         zero.send_calls == 2); // one failed send, one EOB
  assert(stats.failure.find("progress") != std::string::npos);

  FakeRadio stale;
  sync = x300::synchronize(stale);
  stats = {};
  assert(!x300::transmit(
      stale, sync, 100, 0.25, 7, 2, numberedSource(5), [] { return false; },
      stats, [&] { stale.advance(2); }));
  assert(stale.send_calls == 0 && stats.accepted == 0 &&
         !stats.start_margin_met);
  assert(stats.failure.find("stale") != std::string::npos);

  FakeRadio no_ack;
  sync = x300::synchronize(no_ack);
  no_ack.acknowledge = false;
  stats = {};
  assert(!x300::transmit(
      no_ack, sync, 100, 0.25, 7, 2, numberedSource(1), [] { return false; },
      stats));
  assert(stats.accepted == 10 && !stats.burst_ack);
  assert(stats.failure.find("acknowledgement") != std::string::npos);

  // Even a completely queued short burst must monitor its reference until ACK.
  FakeRadio lost_lock;
  sync = x300::synchronize(lost_lock);
  lost_lock.lose_lock_on_send = true;
  stats = {};
  assert(!x300::transmit(
      lost_lock, sync, 100, 0.25, 7, 2, numberedSource(1), [] { return false; },
      stats));
  assert(stats.accepted == 10 &&
         stats.failure.find("unlocked") != std::string::npos);

  FakeRadio render_fault;
  sync = x300::synchronize(render_fault);
  stats = {};
  auto fail = [count = 0](x300::Frame &frame) mutable {
    if (++count == 4)
      throw std::runtime_error("injected renderer failure");
    frame.iq.resize(20);
    return true;
  };
  assert(!x300::transmit(
      render_fault, sync, 100, 0.25, 7, 2, fail, [] { return false; }, stats));
  assert(stats.failure == "injected renderer failure");

  FakeRadio stop;
  sync = x300::synchronize(stop);
  stats = {};
  assert(x300::transmit(
      stop, sync, 100, 0.25, 7, 2, numberedSource(100),
      [&] { return stop.received.size() >= 6; }, stats));
  assert(stats.interrupted && stats.accepted == 3 && stats.burst_ack);
}

void testTimedStartBackpressure() {
  for (bool continuous : {false, true}) {
    // A navigation boundary can make the first render frame smaller than
    // the FIFO, so subsequent frames must also tolerate pre-start blocking.
    for (int frame_samples : {250000, 10000}) {
      FakeRadio radio;
      auto sync = x300::synchronize(radio);
      radio.advance(1.188227605 - radio.now().get_real_secs());
      radio.rate = 2500000;
      radio.partial_limit = 364;
      radio.fifo_capacity = 65156;
      x300::TxStats stats;
      const int frames = continuous ? 1000 : 2000000 / frame_samples;
      const bool ok = x300::transmit(
          radio, sync, radio.rate, 2, 364, 50,
          numberedSource(frames, frame_samples),
          [&] { return continuous && radio.received.size() / 2 >= 500000; },
          stats, [] {}, 4.0);
      if (!ok)
        std::cerr << "timed FIFO regression: " << stats.failure
                  << "; accepted=" << stats.accepted
                  << "; first timeout H=" << radio.first_timeout_hardware
                  << '\n';
      assert(ok && stats.burst_ack && stats.underflows == 0);
      assert(stats.prestart_backpressure_waits > 0);
      assert(stats.zero_sample_sends == stats.prestart_backpressure_waits);
      assert(radio.first_timeout_hardware < stats.start_seconds);
      if (frame_samples == 250000)
        assert(radio.accepted_at_first_timeout == 65156);
      assert(continuous ? stats.interrupted : stats.accepted == 2000000);
      assert(stats.start_seconds == 4.0);
      assert(stats.accepted == radio.received.size() / 2);
      for (size_t i = 0; i < radio.received.size(); ++i)
        assert(radio.received[i] == static_cast<int16_t>(i));
      assert(radio.metadata.front().has_time_spec &&
             radio.metadata.front().time_spec.get_real_secs() == 4.0);
      for (size_t i = 1; i < radio.metadata.size(); ++i)
        assert(!radio.metadata[i].has_time_spec &&
               !radio.metadata[i].start_of_burst);
    }
  }
  // A finite burst can exactly fill the FIFO before its future start.
  // EOB must also wait for room, otherwise the run ends without an ACK.
  FakeRadio radio;
  auto sync = x300::synchronize(radio);
  radio.rate = 2500000;
  radio.partial_limit = 364;
  radio.fifo_capacity = 65156;
  x300::TxStats stats;
  assert(x300::transmit(radio, sync, radio.rate, 2, 364, 1,
      numberedSource(1, 65156), [] { return false; }, stats, [] {}, 4.0));
  assert(stats.accepted == 65156 && stats.burst_ack && stats.underflows == 0);
}

void testBackpressureFaultsAndStop() {
  for (const std::string fault : {"stall", "underflow", "receiver", "async", "stop"}) {
    FakeRadio radio;
    auto sync = x300::synchronize(radio);
    radio.advance(1.188227605 - radio.now().get_real_secs());
    radio.rate = 2500000;
    radio.partial_limit = 364;
    radio.fifo_capacity = 65156;
    radio.stalled_fifo = fault == "stall" || fault == "underflow";
    // A real post-start stall must fail even without an async error report.
    radio.fifo_underflow_reported = fault == "stall";
    if (fault == "async")
      radio.inject_event = Event::EVENT_CODE_SEQ_ERROR;
    x300::TxStats stats;
    const bool ok = x300::transmit(
        radio, sync, radio.rate, 2, 364, 2, numberedSource(10, 250000),
        [&] { return fault == "stop" && radio.send_timeouts >= 2; }, stats,
        [] {}, 4.0, [&] {
          if (fault == "receiver" && radio.send_timeouts > 0)
            throw std::runtime_error("receiver lost while waiting for TX start");
        });
    assert(stats.accepted == (fault == "async" ? 364u : 65156u));
    assert(radio.now().get_real_secs() < stats.start_seconds + 2);
    if (fault == "stop") {
      assert(ok && stats.interrupted && stats.burst_ack && stats.underflows == 0);
      assert(stats.zero_sample_sends == 2);
      assert(stats.prestart_backpressure_waits == 1);
    } else {
      assert(!ok);
      if (fault == "stall") {
        assert(stats.failure.find("progress at hardware") != std::string::npos);
        assert(stats.zero_sample_sends == stats.prestart_backpressure_waits + 1);
      } else if (fault == "underflow") {
        assert(stats.underflows == 1);
        assert(stats.failure.find("continuity") != std::string::npos);
      } else if (fault == "receiver") {
        assert(stats.failure == "receiver lost while waiting for TX start");
        assert(stats.prestart_backpressure_waits == 0);
      } else {
        assert(stats.sequence_errors == 1 && stats.prestart_backpressure_waits == 0);
        assert(stats.failure.find("continuity") != std::string::npos);
      }
    }
  }
}

void testUntimedSequenceErrorAndStopDuringSend() {
  FakeRadio error;
  auto sync = x300::synchronize(error);
  error.rate = 2500000;
  error.partial_limit = 364;
  error.inject_event = Event::EVENT_CODE_SEQ_ERROR;
  error.event_has_time = false; // X300 transport errors may lack a timestamp.
  x300::TxStats stats;
  assert(!x300::transmit(error, sync, error.rate, 2, 364, 2,
      numberedSource(2, 250000), [] { return false; }, stats));
  assert(stats.accepted == 364 && stats.accepted_at_first_error == 364);
  assert(stats.sequence_errors == 1 && stats.underflows == 0);
  assert(!stats.first_event_has_time && stats.burst_ack);
  assert(stats.first_event_observed_host_monotonic_seconds.has_value());
  assert(*stats.first_event_observed_host_monotonic_seconds >= sync.last_checked_at);
  assert(stats.data_send_calls == 1 && stats.partial_sample_sends == 0);
  assert(stats.max_samples_per_send == 364);

  // SIGINT can interrupt a blocked post-start send and make it return zero.
  // That is an operator stop, unless UHD also reports a continuity error.
  for (bool with_error : {false, true}) {
    FakeRadio radio;
    sync = x300::synchronize(radio);
    radio.rate = 2500000;
    radio.partial_limit = 364;
    radio.fifo_capacity = 65156;
    radio.stalled_fifo = true;
    radio.fifo_underflow_reported = true;
    stats = {};
    auto stop = [&] {
      if (radio.send_timeouts && radio.wall - radio.origin >= radio.start) {
        radio.stalled_fifo = false; // unblock shutdown once stop is observed
        if (with_error) radio.inject_event = Event::EVENT_CODE_SEQ_ERROR;
        return true;
      }
      return false;
    };
    const bool ok = x300::transmit(radio, sync, radio.rate, 2, 364, 2,
        numberedSource(10, 250000), stop, stats, [] {}, 4.0);
    assert(ok == !with_error);
    assert(stats.interrupted && stats.accepted == 65156 && stats.burst_ack);
    assert(stats.sequence_errors == (with_error ? 1u : 0u));
    if (with_error)
      assert(stats.failure.find("continuity") != std::string::npos);
    else
      assert(stats.failure.empty());
  }
}

void testFixedDeadline() {
  for (double render_cost : {0.1, 3.0}) {
    FakeRadio radio;
    auto sync=x300::synchronize(radio);
    const double deadline=ceil(radio.now().get_real_secs()+1);
    x300::TxStats stats;
    auto source=numberedSource(1);
    bool first=true;
    auto render=[&](x300::Frame &frame) {
      if(first) { radio.advance(render_cost); first=false; }
      return source(frame);
    };
    bool ok=x300::transmit(radio,sync,100,0.25,7,1,render,
        [] { return false; },stats,[] {},deadline);
    assert(stats.start_seconds==deadline);
    assert(fabs(stats.prebuffer_seconds-render_cost)<1e-9);
    if(render_cost<1) {
      assert(ok && stats.accepted==10 && stats.send_attempted);
      assert(radio.metadata.front().time_spec.get_real_secs()==deadline);
    } else {
      assert(!ok && !stats.send_attempted && radio.send_calls==0);
      assert(stats.failure.find("deadline missed")!=std::string::npos);
    }
  }
  FakeRadio radio;
  auto sync=x300::synchronize(radio);
  x300::TxStats stats;
  int checks=0;
  assert(!x300::transmit(radio,sync,100,.25,7,2,numberedSource(5),
      [] { return false; },stats,[] {},3.0,[&] {
        if(++checks==2) throw std::runtime_error("receiver disconnected");
      }));
  assert(radio.send_calls==0 && stats.failure=="receiver disconnected");
}

int main() {
  testTimelineAndNavigationBoundaries();
  testPpsContract();
  testFiniteBurstAndShortSends();
  testTransportFailures();
  testTimedStartBackpressure();
  testBackpressureFaultsAndStop();
  testUntimedSequenceErrorAndStopDuringSend();
  testFixedDeadline();
  std::cout
      << "X300 timeline, PPS, bounded queue and transport fault tests passed\n";
}
