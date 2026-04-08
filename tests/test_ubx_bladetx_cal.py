import importlib.util
import sys
import unittest
from pathlib import Path


def load_module():
    path = Path(__file__).resolve().parents[1] / "tools" / "ubx_bladetx_cal.py"
    spec = importlib.util.spec_from_file_location("ubx_bladetx_cal", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


mod = load_module()


class CalibrationMathTest(unittest.TestCase):
    def test_positive_delta_advances_injected_epoch_when_injected_is_late(self):
        delta = mod.recommendation_from_real_minus_injected_m(-29.9792458)
        self.assertEqual(delta, 100)

    def test_negative_delta_delays_injected_epoch_when_injected_is_early(self):
        delta = mod.recommendation_from_real_minus_injected_m(59.9584916)
        self.assertEqual(delta, -200)

    def test_clamp_detection_matches_ublox_floor(self):
        self.assertTrue(mod.is_prres_clamped(-3276.8))
        self.assertTrue(mod.is_prres_clamped(3276.75))
        self.assertFalse(mod.is_prres_clamped(-120.0))


class ConfidenceTest(unittest.TestCase):
    def test_confidence_is_medium_for_one_used_injected_prn_with_enough_epochs(self):
        residuals = [
            mod.ResidualEpoch(
                itow_sec=100.0 + i,
                utc_label="2026-04-07 11:11:00",
                synth_prns=(8,),
                real_prns=(15, 23, 24, 29),
                synth_pr_res_m=8.0,
                real_pr_res_m=0.0,
                real_minus_injected_m=-8.0,
                recommended_delta_cal_ns=27,
            )
            for i in range(12)
        ]
        self.assertEqual(mod.classify_confidence(residuals, {8}), "medium")


if __name__ == "__main__":
    unittest.main()
