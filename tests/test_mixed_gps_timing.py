"""Independent ephemeris oracle and injected clock/epoch-offset recovery."""
import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import analyze_mixed_gps_timing as timing


class MixedTimingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fixture = Path(__file__).parent / 'fixtures/f9p-lnav-prn3.json'
        cls.fixture = json.loads(fixture.read_text())
        cls.ep = {}
        for words in cls.fixture['words'].values():
            bits = ''.join(f'{(w >> 6) & 0xffffff:024b}' for w in words)
            cls.ep.update(timing.decode(bits, 2437))

    def test_ephemeris_against_rtklib_fixture(self):
        names = dict(af0='f0', af1='f1', af2='f2', deltan='dn', sqrta='sqrt_a',
                     omg0='omega0', inc0='i0', aop='arg_perigee', omgdot='omega_dot',
                     idot='i_dot', codeL2='code', L2P='flag')
        for key, expected in self.fixture['expected'].items():
            name = names.get(key, key)
            if name in self.ep:
                self.assertAlmostEqual(self.ep[name], expected,
                                       delta=max(1e-14, abs(expected)*2e-11))

    def test_recovers_delay_independent_of_receiver_clock(self):
        receiver = timing.receiver_ecef(21.0047844, 105.8460541, 22)
        epoch = 121730.25
        measured = []
        for receiver_bias in (-.0065, .0023):
            offsets = []
            for model_lag in (0., .0369307, -.0125):
                # Independent forward model: find transmission time from
                # reception time and geometric flight time, then form RAWX.
                rx_model = epoch-model_lag
                tx = rx_model-.07
                for _ in range(8):
                    rho, clk, _ = timing.range_and_clock(self.ep, tx, receiver)
                    tx = rx_model-rho/timing.C
                rho, clk, _ = timing.range_and_clock(self.ep, tx, receiver)
                pseudorange = rho+timing.C*(receiver_bias+model_lag-clk)
                # This test isolates code-time recovery. Doppler is not used.
                result, _ = timing.reconstruct_observation(
                    self.ep, epoch+receiver_bias, pseudorange, 0, receiver)
                offsets.append(result['apparent_clock_m']/timing.C)
            self.assertAlmostEqual(offsets[1]-offsets[0], .0369307, delta=1e-9)
            self.assertAlmostEqual(offsets[2]-offsets[0], -.0125, delta=1e-9)
            measured.append(offsets[1]-offsets[0])
        self.assertAlmostEqual(*measured, delta=1e-9)


if __name__ == '__main__':
    unittest.main()
