"""Behavior checks for SI-unit, geometry-derived passive compensation."""

import math
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from research.gravity_compensation.geometry import FourierCam, GeometryError, RollerCompensator


class GravityGeometryTests(unittest.TestCase):
    def non_circular_module(self, **changes):
        parameters = dict(
            cam=FourierCam(0.025, cos_m=(-0.004, 0.0004, 0.0001, 0.00005),
                           sin_m=(0.0002, -0.0001, 0.00005, -0.00002)),
            roller_radius_m=0.004, follower_offset_m=0.002,
            spring_stiffness_n_per_m=2000.0,
            preload_m=0.015, max_compression_m=0.040,
        )
        return RollerCompensator(**(parameters | changes))

    def test_circular_profile_has_constant_radius_and_zero_derivatives(self):
        cam = FourierCam(mean_radius_m=0.025)
        for angle in (0.0, math.pi / 4, math.pi, 2 * math.pi):
            self.assertEqual(cam.radius_derivatives(angle), (0.025, 0.0, 0.0))

    def test_circular_cam_rotates_without_follower_motion_or_compensation_torque(self):
        module = RollerCompensator(
            cam=FourierCam(0.025), roller_radius_m=0.005,
            follower_offset_m=0.004, spring_stiffness_n_per_m=1000.0,
            preload_m=0.010, max_compression_m=0.030,
        )
        expected_height = math.sqrt(0.030**2 - 0.004**2)
        for q in (-math.pi / 4, 0.0, math.pi / 4):
            state = module.evaluate(q)
            self.assertAlmostEqual(state.follower_y_m, expected_height, delta=1e-10)
            self.assertAlmostEqual(state.displacement_m, 0.0, delta=1e-10)
            self.assertAlmostEqual(state.spring_compression_m, 0.010, delta=1e-10)
            self.assertAlmostEqual(state.spring_force_n, 10.0, delta=1e-7)
            self.assertAlmostEqual(state.energy_j, 0.05, delta=1e-9)
            self.assertAlmostEqual(state.torque_nm, 0.0, delta=1e-9)

    def test_non_circular_contact_produces_torque_from_spring_energy(self):
        module = RollerCompensator(
            cam=FourierCam(0.025, cos_m=(-0.004, 0.0004, 0.0, 0.0)),
            roller_radius_m=0.005, follower_offset_m=0.003,
            spring_stiffness_n_per_m=2000.0,
            preload_m=0.015, max_compression_m=0.040,
        )
        q, h = 0.21, 1e-5
        state = module.evaluate(q)
        derivative = -(module.evaluate(q + h).energy_j
                       - module.evaluate(q - h).energy_j) / (2 * h)
        self.assertGreater(abs(state.displacement_m), 1e-5)
        self.assertGreater(abs(state.torque_nm), 0.01)
        self.assertAlmostEqual(state.torque_nm, derivative, delta=1e-7)
        # Independently reconstruct outward normal and rotated roller centre.
        phi = state.contact_phi_rad
        r, rp, _ = module.cam.radius_derivatives(phi)
        length = math.hypot(r, rp)
        nx = (r * math.cos(phi) + rp * math.sin(phi)) / length
        ny = (r * math.sin(phi) - rp * math.cos(phi)) / length
        x = r * math.cos(phi) + module.roller_radius_m * nx
        y = r * math.sin(phi) + module.roller_radius_m * ny
        world_x = x * math.cos(q) - y * math.sin(q)
        world_y = x * math.sin(q) + y * math.cos(q)
        self.assertAlmostEqual(world_x, module.follower_offset_m, delta=1e-10)
        self.assertAlmostEqual(world_y, state.follower_y_m, delta=1e-10)

    def test_invalid_numerical_and_physical_inputs_are_rejected(self):
        for parameters in (
            {"mean_radius_m": -0.025}, {"mean_radius_m": float("nan")},
            {"mean_radius_m": 0.025, "cos_m": (0.0,)},
            {"mean_radius_m": 0.025, "sin_m": (0.0, 0.0, 0.0, float("inf"))},
        ):
            with self.subTest(cam=parameters), self.assertRaises(ValueError):
                FourierCam(**parameters)
        parameters = dict(cam=FourierCam(0.025), roller_radius_m=0.005,
                          follower_offset_m=0.0, spring_stiffness_n_per_m=1000.0,
                          preload_m=0.01, max_compression_m=0.03)
        for changes in (
            {"roller_radius_m": 0.0}, {"spring_stiffness_n_per_m": -1.0},
            {"preload_m": 0.0}, {"max_compression_m": 0.005},
            {"follower_offset_m": float("nan")}, {"contact_samples": 2},
            {"contact_samples": 512.5}, {"follower_offset_m": 0.06},
        ):
            with self.subTest(module=changes), self.assertRaises(ValueError):
                RollerCompensator(**(parameters | changes))

    def test_unoptimized_example_is_feasible_over_explicit_sixty_degree_range(self):
        module = RollerCompensator(
            cam=FourierCam(0.025, cos_m=(-0.006, 0.0, 0.0, 0.0)),
            roller_radius_m=0.004, follower_offset_m=0.0,
            spring_stiffness_n_per_m=4000.0,
            preload_m=0.018, max_compression_m=0.035,
            angle_limit_rad=math.pi / 3,
        )
        report = module.validate_range(samples=241)
        self.assertGreater(report["min_compression_m"], 0.01)
        self.assertLess(report["max_compression_m"], 0.035)
        self.assertGreater(report["min_torque_nm"], 0.0)
        self.assertLess(report["max_abs_pressure_angle_rad"], math.pi / 4)
        self.assertEqual(report["samples"], 241)

    def test_four_harmonic_derivatives_match_independent_finite_differences(self):
        cam = self.non_circular_module().cam
        for phi in (-0.4, 0.2, 1.7, 5.4):
            h = 1e-4
            r, first, second = cam.radius_derivatives(phi)
            left = cam.radius_derivatives(phi - h)[0]
            right = cam.radius_derivatives(phi + h)[0]
            self.assertAlmostEqual(first, (right - left)/(2*h), delta=1e-9)
            self.assertAlmostEqual(second, (right - 2*r + left)/(h*h), delta=2e-8)

    def test_radius_and_convexity_certificates_reject_infeasible_profiles(self):
        report = FourierCam(0.025).validate()
        self.assertEqual(report["min_radius_bound_m"], 0.025)
        self.assertGreater(report["convexity_margin_m2"], 0.0)
        # Radius stays positive, but the four-lobed curve has concave segments.
        with self.assertRaisesRegex(GeometryError, "convexity"):
            FourierCam(0.025, cos_m=(0.0, 0.0, 0.0, 0.002)).validate()
        with self.assertRaisesRegex(GeometryError, "radius"):
            FourierCam(0.039, cos_m=(0.003, 0.0, 0.0, 0.0)).validate()
        with self.assertRaisesRegex(GeometryError, "radius"):
            self.non_circular_module(max_envelope_radius_m=0.025)

    def test_range_and_spring_failures_are_not_silently_clipped(self):
        module = self.non_circular_module()
        for q in (float("nan"), float("inf"), math.pi/4 + 0.001):
            with self.subTest(q=q), self.assertRaises(GeometryError):
                module.evaluate(q)
        for parameters in ({"samples": 2}, {"samples": 12.5},
                           {"q_min_rad": 0.2, "q_max_rad": -0.2},
                           {"q_min_rad": -math.pi/2}):
            with self.subTest(range=parameters), self.assertRaises(GeometryError):
                module.validate_range(**parameters)
        with self.assertRaisesRegex(GeometryError, "Spring unloads"):
            self.non_circular_module(preload_m=0.001).evaluate(math.pi/4)
        with self.assertRaisesRegex(GeometryError, "compression limit"):
            self.non_circular_module(max_compression_m=0.016).evaluate(-math.pi/4)

    def test_passive_work_matches_energy_difference_and_closed_loop_has_zero_work(self):
        module = self.non_circular_module()
        count = 801
        angles = [-math.pi/4 + (math.pi/2)*i/(count-1) for i in range(count)]
        states = [module.evaluate(q) for q in angles]
        work = sum(0.5*(left.torque_nm + right.torque_nm)*(right.q_rad-left.q_rad)
                   for left, right in zip(states, states[1:]))
        self.assertAlmostEqual(work, states[0].energy_j - states[-1].energy_j, delta=1e-6)
        loop = states + list(reversed(states[:-1]))
        cycle_work = sum(0.5*(left.torque_nm + right.torque_nm)*(right.q_rad-left.q_rad)
                         for left, right in zip(loop, loop[1:]))
        self.assertAlmostEqual(cycle_work, 0.0, delta=1e-12)
        # This ideal conservative module has no hysteresis; loss is a later model.
        self.assertEqual(loop[0].energy_j, loop[-1].energy_j)

    def test_contact_grid_refinement_and_energy_step_refinement_are_consistent(self):
        coarse = self.non_circular_module(contact_samples=128)
        fine = self.non_circular_module(contact_samples=1024)
        for q in (-math.pi/4, -0.3, 0.0, 0.17, math.pi/4):
            left, right = coarse.evaluate(q), fine.evaluate(q)
            self.assertAlmostEqual(left.follower_y_m, right.follower_y_m, delta=1e-10)
            self.assertAlmostEqual(left.torque_nm, right.torque_nm, delta=1e-8)
        for q in (-0.7, -0.1, 0.3, 0.7):
            torque = fine.evaluate(q).torque_nm
            for h in (1e-4, 5e-5):
                difference = -(fine.evaluate(q+h).energy_j-fine.evaluate(q-h).energy_j)/(2*h)
                self.assertAlmostEqual(torque, difference, delta=1e-7)

    def test_near_tangent_or_absent_contact_is_explicitly_infeasible(self):
        for offset in (0.03, 0.0300001, 0.03*(1-1e-14)):
            with self.subTest(offset=offset), self.assertRaises(GeometryError):
                RollerCompensator(FourierCam(0.025), 0.005, offset,
                                  1000.0, 0.010, 0.030)
        with self.assertRaises(GeometryError):
            self.non_circular_module(follower_offset_m=0.05)

    def test_three_preload_presets_share_geometry_and_keep_spring_engaged(self):
        cam = FourierCam(0.025, cos_m=(-0.002, 0.0, 0.0, 0.0))
        torques = []
        for preload in (0.0043, 0.0135, 0.0227):
            module = RollerCompensator(cam, 0.004, 0.0, 10000.0, preload,
                                      0.035, angle_limit_rad=math.pi/3)
            report = module.validate_range(samples=241)
            self.assertGreater(report["min_compression_m"], 0.0025)
            self.assertLess(report["max_compression_m"], 0.025)
            self.assertLess(report["max_abs_pressure_angle_rad"], math.radians(5))
            torques.append(module.evaluate(0).torque_nm)
        for computed, expected in zip(torques, (0.0859245564, 0.2697631421, 0.4536017278)):
            self.assertAlmostEqual(computed, expected, delta=1e-9)

    def test_signed_pressure_angle_is_reported_and_audit_uses_absolute_value(self):
        for offset in (-0.02, 0.02):
            module = RollerCompensator(FourierCam(0.025), 0.005, offset,
                                      1000.0, 0.010, 0.030)
            pressure = module.evaluate(0).pressure_angle_rad
            self.assertEqual(math.copysign(1, pressure), math.copysign(1, offset))
            self.assertAlmostEqual(module.validate_range(samples=3)["max_abs_pressure_angle_rad"],
                                   abs(pressure), delta=1e-12)


if __name__ == "__main__":
    unittest.main()
