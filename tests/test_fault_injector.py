import os
import sys
import unittest

# Allow running tests from repository root without installing as a package.
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from fault_inject import FaultConfig, FaultInjector, FaultType, validate_road


class TestFaultInjector(unittest.TestCase):
    def setUp(self):
        self.road = [[float(i), 0.0, 0.0, 3.0] for i in range(30)]

    def test_f1_waypoint_displacement_changes_some_points(self):
        cfg = FaultConfig(
            fault_type=FaultType.WAYPOINT_DISPLACEMENT,
            severity=0.5,
            seed=7,
            affected_fraction=0.2,
        )
        injector = FaultInjector(cfg)
        out = injector.inject(self.road)

        changed = 0
        for a, b in zip(self.road, out):
            if abs(a[0] - b[0]) > 1e-9 or abs(a[1] - b[1]) > 1e-9:
                changed += 1

        self.assertGreaterEqual(changed, 1)
        self.assertEqual(len(out), len(self.road))

    def test_f2_curvature_injection_preserves_length_and_changes_geometry(self):
        cfg = FaultConfig(
            fault_type=FaultType.CURVATURE_INJECTION,
            severity=1.2,
            seed=7,
            window_size=12,
            inject_at="peak",
        )
        injector = FaultInjector(cfg)
        out = injector.inject(self.road)

        self.assertEqual(len(out), len(self.road))
        self.assertTrue(any(abs(p[1]) > 1e-9 for p in out))

    def test_inject_with_width_passthrough(self):
        cfg = FaultConfig(
            fault_type=FaultType.NOISE_INJECTION,
            severity=0.0,
            seed=7,
        )
        injector = FaultInjector(cfg)
        out, width = injector.inject_with_width(self.road, road_width=8.0)

        self.assertEqual(out, self.road)
        self.assertAlmostEqual(width, 8.0, places=6)

    def test_f4_dropout_contiguous_with_multiple_blocks_reduces_points(self):
        cfg = FaultConfig(
            fault_type=FaultType.WAYPOINT_DROPOUT,
            severity=0.2,
            seed=7,
            contiguous=True,
            dropout_blocks=2,
        )
        injector = FaultInjector(cfg)
        out = injector.inject(self.road)

        self.assertLess(len(out), len(self.road))
        self.assertEqual(out[0], self.road[0])
        self.assertEqual(out[-1], self.road[-1])

    def test_f5_noise_injection_keeps_length_and_is_valid(self):
        cfg = FaultConfig(
            fault_type=FaultType.NOISE_INJECTION,
            severity=0.1,
            seed=7,
        )
        injector = FaultInjector(cfg)
        out = injector.inject(self.road)
        valid, _ = validate_road(out, original_len=len(self.road))

        self.assertEqual(len(out), len(self.road))
        self.assertTrue(valid)


if __name__ == "__main__":
    unittest.main()
