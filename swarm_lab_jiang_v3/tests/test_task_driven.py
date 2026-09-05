"""Unit tests for the v3 matrix-weighted Laplacian + task-driven control law.

Covers the spec's acceptance criteria (section 3.3): sign self-check,
static convergence, null-space decoupling, non-task-mode non-leakage, and
the two-robot 2D benchmark for the Z construction (spec section 2.4).
"""

from __future__ import annotations

import unittest

import numpy as np

from swarm.algorithms.task_driven import (
    TaskDrivenFormationController,
    build_formation_spec,
    build_task_modes,
    trivial_spec,
)
from swarm.domain.config import TaskDrivenConfig
from swarm.domain.models import MocapSnapshot, VehicleState


def _snapshot(positions, valid=True):
    states = {}
    for vehicle_id, (x, y) in positions.items():
        state = VehicleState(vehicle_id, x=x, y=y)
        state.valid = valid
        states[vehicle_id] = state
    return MocapSnapshot(0.0, states)


def _two_car_controller(w=(0.0, 0.0), schedule=(), k=0.8, limit=0.25):
    spec = build_formation_spec(
        vehicle_ids=["car1", "car2"],
        adjacency_matrix=((0, 1), (1, 0)),
        formation={"car1": (0.0, 0.0), "car2": (0.0, 1.0)},
    )
    return TaskDrivenFormationController(
        spec,
        TaskDrivenConfig(k=k),
        task_velocity=w,
        task_velocity_schedule=schedule,
        command_speed_limit_mps=limit,
    )


class TaskModeBuildTests(unittest.TestCase):
    """Spec section 2.4: two-robot 2D benchmark for the Z construction."""

    def test_two_robot_translation_modes(self) -> None:
        Z = build_task_modes(2, ("translate_x", "translate_y"))
        np.testing.assert_array_equal(
            Z, np.array([[1.0, 0.0], [0.0, 1.0], [1.0, 0.0], [0.0, 1.0]])
        )

    def test_per_robot_blocks_are_identity(self) -> None:
        spec = build_formation_spec(
            vehicle_ids=["car1", "car2"],
            adjacency_matrix=((0, 1), (1, 0)),
            formation={"car1": (0.0, 0.0), "car2": (0.0, 1.0)},
        )
        np.testing.assert_array_equal(spec.z_block(0), np.eye(2))
        np.testing.assert_array_equal(spec.z_block(1), np.eye(2))

    def test_explicit_mode_vector_accepted(self) -> None:
        Z = build_task_modes(2, ([0.0, 1.0, 0.0, 1.0],))
        self.assertEqual(Z.shape, (4, 1))

    def test_unknown_mode_rejected(self) -> None:
        with self.assertRaises(ValueError):
            build_task_modes(2, ("rotate",))

    def test_wrong_vector_length_rejected(self) -> None:
        with self.assertRaises(ValueError):
            build_task_modes(2, ([1.0, 0.0],))


class FormationSpecTests(unittest.TestCase):
    def test_b_matrix_is_target_minus_source(self) -> None:
        spec = build_formation_spec(
            vehicle_ids=["car1", "car2"],
            adjacency_matrix=((0, 1), (1, 0)),
            formation={"car1": (0.0, 0.0), "car2": (0.0, 1.0)},
        )
        np.testing.assert_allclose(spec.b_matrix[0, 1], (0.0, 1.0))
        np.testing.assert_allclose(spec.b_matrix[1, 0], (0.0, -1.0))

    def test_default_weights_are_scalar_identity(self) -> None:
        spec = build_formation_spec(
            vehicle_ids=["car1", "car2"],
            adjacency_matrix=((0, 1), (1, 0)),
            formation={"car1": (0.0, 0.0), "car2": (0.0, 1.0)},
        )
        np.testing.assert_allclose(spec.weight_blocks[0, 1], np.eye(2))
        np.testing.assert_allclose(spec.weight_blocks[0, 0], np.zeros((2, 2)))

    def test_custom_scalar_weight(self) -> None:
        spec = build_formation_spec(
            vehicle_ids=["car1", "car2"],
            adjacency_matrix=((0, 1), (1, 0)),
            formation={"car1": (0.0, 0.0), "car2": (0.0, 1.0)},
            edge_weight_matrix=((0, 2.0), (2.0, 0)),
        )
        np.testing.assert_allclose(spec.weight_blocks[0, 1], 2.0 * np.eye(2))


class ControlLawSignTests(unittest.TestCase):
    """Spec 3.3.1: the docstring self-check, encoded."""

    def test_commands_shrink_the_error(self) -> None:
        # desired p_1*=(0,0), p_2*=(0,1); actual p_2 is +0.4 too far right.
        controller = _two_car_controller()
        result = controller.step(
            _snapshot({"car1": (0.0, 0.0), "car2": (0.4, 1.0)}), 0.02
        )
        self.assertGreater(result.commands["car1"].vx, 0.0)  # +x: closes the gap
        self.assertLess(result.commands["car2"].vx, 0.0)  # -x: symmetric
        self.assertAlmostEqual(
            result.commands["car1"].vx, -result.commands["car2"].vx, places=9
        )
        # zero y error -> zero y command
        self.assertAlmostEqual(result.commands["car1"].vy, 0.0, places=9)

    def test_exact_formation_gives_zero_constraint(self) -> None:
        controller = _two_car_controller()
        result = controller.step(
            _snapshot({"car1": (1.0, 2.0), "car2": (1.0, 3.0)}), 0.02
        )
        # desired b_12 = (0, 1) is satisfied (translation-free): zero command
        self.assertAlmostEqual(result.commands["car1"].vx, 0.0, places=9)
        self.assertAlmostEqual(result.commands["car2"].vy, 0.0, places=9)


class StaticConvergenceTests(unittest.TestCase):
    """Spec 3.3.2: w=0, off-formation start -> error decays exponentially."""

    def test_error_decays_to_zero(self) -> None:
        controller = _two_car_controller()
        positions = {"car1": [0.3, -0.2], "car2": [-0.2, 1.4]}  # far off formation
        errors = []
        for _ in range(500):  # 10 s at 50 Hz
            result = controller.step(
                _snapshot({k: tuple(v) for k, v in positions.items()}), 0.02
            )
            errors.append(result.mean_error)
            for vid, command in result.commands.items():
                positions[vid][0] += command.vx * 0.02
                positions[vid][1] += command.vy * 0.02
        # Deadband floor: the shaped constraint term freezes once
        # k*|e| < deadband_mps, i.e. |e| < deadband/k ~= 0.019 m ("0 附近").
        floor = 0.015 / 0.8 + 0.006  # deadband/k + margin
        self.assertLess(errors[-1], floor)


class NullSpaceDecouplingTests(unittest.TestCase):
    """Spec 3.3.3: a task term inside null(L_A) is never fought."""

    def test_translation_task_not_cancelled(self) -> None:
        controller = _two_car_controller(w=(0.10, 0.0))
        # Start exactly on formation: the constraint term is zero, so the
        # command must equal the task velocity exactly.
        result = controller.step(
            _snapshot({"car1": (0.0, 0.0), "car2": (0.0, 1.0)}), 0.02
        )
        self.assertAlmostEqual(result.commands["car1"].vx, 0.10, places=9)
        self.assertAlmostEqual(result.commands["car2"].vx, 0.10, places=9)
        self.assertAlmostEqual(result.commands["car1"].vy, 0.0, places=9)

    def test_formation_keeps_shape_while_translating(self) -> None:
        controller = _two_car_controller(w=(0.10, 0.0))
        positions = {"car1": [0.0, 0.0], "car2": [0.0, 1.0]}
        for _ in range(500):
            result = controller.step(
                _snapshot({k: tuple(v) for k, v in positions.items()}), 0.02
            )
            for vid, command in result.commands.items():
                positions[vid][0] += command.vx * 0.02
                positions[vid][1] += command.vy * 0.02
        self.assertAlmostEqual(positions["car1"][0], 1.0, places=2)  # 0.1 m/s * 10 s
        self.assertAlmostEqual(positions["car2"][0], 1.0, places=2)
        self.assertAlmostEqual(
            positions["car2"][1] - positions["car1"][1], 1.0, places=3
        )
        self.assertLess(result.mean_error, 1e-6)


class NonTaskModeTests(unittest.TestCase):
    """Spec 3.3.4: scale-type errors are corrected, not driven by the task."""

    def test_scale_error_corrected_under_active_task(self) -> None:
        controller = _two_car_controller(w=(0.10, 0.0))
        # desired separation 1.0 m in y; actual 1.6 m (scale blown up).
        result = controller.step(
            _snapshot({"car1": (0.0, 0.0), "car2": (0.0, 1.6)}), 0.02
        )
        # car1 pushed +y (to widen... no: separation too large -> car1 pulled
        # toward car2 along -y? car1 at y=0, car2 at y=1.6, desired 1.0:
        # car1 moves +y, car2 moves -y, shrinking the separation.
        self.assertGreater(result.commands["car1"].vy, 0.0)
        self.assertLess(result.commands["car2"].vy, 0.0)
        # and both still carry the +x task velocity
        self.assertGreater(result.commands["car1"].vx, 0.09)
        self.assertGreater(result.commands["car2"].vx, 0.09)


class ScheduleAndSafetyTests(unittest.TestCase):
    def test_schedule_switches_task_velocity(self) -> None:
        controller = _two_car_controller(
            w=(0.0, 0.0), schedule=((0.0, (0.0, 0.0)), (3.0, (0.10, 0.0)))
        )
        snap = _snapshot({"car1": (0.0, 0.0), "car2": (0.0, 1.0)})
        for _ in range(100):  # 2 s: still in the standstill window
            result = controller.step(snap, 0.02)
        self.assertAlmostEqual(result.commands["car1"].vx, 0.0, places=9)
        for _ in range(100):  # past t=3 s
            result = controller.step(snap, 0.02)
        self.assertAlmostEqual(result.commands["car1"].vx, 0.10, places=9)

    def test_invalid_vehicle_gets_exact_zero(self) -> None:
        controller = _two_car_controller(w=(0.10, 0.0))
        result = controller.step(
            _snapshot({"car1": (0.0, 0.0), "car2": (0.0, 1.0)}, valid=False),
            0.02,
        )
        for command in result.commands.values():
            self.assertEqual((command.vx, command.vy), (0.0, 0.0))
            self.assertFalse(command.valid)
        self.assertFalse(result.ready)

    def test_overlap_is_not_singular(self) -> None:
        controller = _two_car_controller()
        result = controller.step(
            _snapshot({"car1": (0.5, 0.5), "car2": (0.5, 0.5)}), 0.02
        )
        # Relative positions are defined at zero separation: the residual
        # equals b_ij and the command pulls the cars apart (car1 -y, car2 +y,
        # since the desired formation has car2 above car1).
        self.assertLess(result.commands["car1"].vy, 0.0)
        self.assertGreater(result.commands["car2"].vy, 0.0)

    def test_trivial_spec_has_no_edges(self) -> None:
        spec = trivial_spec(["car1"])
        self.assertEqual(spec.Z.shape, (2, 0))


if __name__ == "__main__":
    unittest.main()
