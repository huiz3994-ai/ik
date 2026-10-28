#!/usr/bin/env python3
"""Pure and service-level tests for reusable RM75 waypoint trajectories."""

import math
import json
from pathlib import Path
from types import SimpleNamespace
import threading
import time

import numpy as np
import pytest

from rm75_sampling_gui.model.robot_model import JOINT_NAMES
from rm75_sampling_gui.trajectory.interpolation import (
    evaluate_quintic,
    generate_quintic,
)
from rm75_sampling_gui.trajectory.models import new_trajectory, validate_document
from rm75_sampling_gui.trajectory.repository import TrajectoryRepository
from rm75_sampling_gui.trajectory.service import TrajectoryService
from rm75_sampling_gui.control.joint_control_adapter import (
    JointControlAdapter,
    JointControlError,
)


def _waypoints(rows):
    return [
        {
            "waypoint_id": "wp-{}".format(index + 1),
            "sequence": index + 1,
            "joint_positions": [float(value) for value in row],
            "link7_pose": {
                "position": {"x": 0.4, "y": 0.0, "z": 0.3},
                "orientation": {"qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
            },
            "ros_timestamp": 100.0 + index,
        }
        for index, row in enumerate(rows)
    ]


def _config(speed=0.4, acceleration=0.3, period=0.01):
    return {
        "interpolation_algorithm": "piecewise_quintic_c2",
        "speed_scale": speed,
        "acceleration_scale": acceleration,
        "sample_period_s": period,
    }


def test_new_trajectory_has_versioned_rm75_defaults():
    document = new_trajectory("采样撤离路径01")

    assert document["schema_version"] == "1.0"
    assert document["name"] == "采样撤离路径01"
    assert document["robot_model"] == "RM75-6FB"
    assert document["joint_names"] == list(JOINT_NAMES)
    assert document["frame_id"] == "base_link"
    assert document["trajectory_status"]["state"] == "EMPTY"
    assert document["trajectory"] == []
    assert len(document["trajectory_id"]) == 36
    assert document["created_at"] == document["updated_at"]


def test_quintic_strictly_passes_every_waypoint_and_is_c2_at_knots():
    rows = [
        np.zeros(7),
        np.array([0.2, 0.1, -0.1, 0.05, 0.08, -0.04, 0.12]),
        np.array([0.45, 0.25, -0.22, 0.12, 0.2, -0.1, 0.3]),
        np.array([0.7, 0.4, -0.3, 0.2, 0.3, -0.15, 0.45]),
    ]
    result = generate_quintic(
        _waypoints(rows), _config(), np.full(7, 1.2), np.full(7, 2.0)
    )

    for expected, point_index in zip(rows, result["waypoint_indices"]):
        np.testing.assert_allclose(
            result["points"][point_index]["positions"], expected, atol=1e-12
        )

    segments = result["segment_coefficients"]
    durations = result["segment_durations"]
    for index in range(len(segments) - 1):
        left = evaluate_quintic(segments[index], durations[index])
        right = evaluate_quintic(segments[index + 1], 0.0)
        for derivative in ("positions", "velocities", "accelerations"):
            np.testing.assert_allclose(left[derivative], right[derivative], atol=1e-10)


def test_monotone_intermediate_waypoint_has_nonzero_pass_velocity_without_overshoot():
    rows = [np.zeros(7), np.full(7, 0.2), np.full(7, 0.5)]
    result = generate_quintic(
        _waypoints(rows), _config(), np.ones(7), np.full(7, 1.5)
    )

    middle = result["points"][result["waypoint_indices"][1]]
    assert all(value > 0.0 for value in middle["velocities"])
    positions = np.asarray([point["positions"] for point in result["points"]])
    assert np.all(np.diff(positions, axis=0) >= -1e-10)
    assert float(positions.min()) >= -1e-12
    assert float(positions.max()) <= 0.5 + 1e-12


def test_direction_reversal_sets_only_reversing_joint_knot_velocity_to_zero():
    rows = [
        np.zeros(7),
        np.array([0.3, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]),
        np.array([0.1, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]),
    ]
    result = generate_quintic(
        _waypoints(rows), _config(), np.ones(7), np.full(7, 2.0)
    )

    velocity = result["points"][result["waypoint_indices"][1]]["velocities"]
    assert abs(velocity[0]) < 1e-12
    assert all(value > 0.0 for value in velocity[1:])


def test_dense_samples_respect_scaled_velocity_and_acceleration_limits():
    rows = [
        np.zeros(7),
        np.array([0.7, -0.4, 0.2, 0.5, -0.6, 0.3, 0.8]),
        np.array([1.0, -0.1, -0.3, 0.8, -0.2, 0.7, 0.4]),
    ]
    velocity_limits = np.array([1.2, 1.1, 1.0, 1.3, 1.0, 1.1, 1.2])
    acceleration_limits = np.array([2.0, 1.8, 1.6, 2.2, 1.7, 1.9, 2.1])
    config = _config(speed=0.25, acceleration=0.2, period=0.005)

    result = generate_quintic(
        _waypoints(rows), config, velocity_limits, acceleration_limits
    )
    velocities = np.abs(np.asarray([point["velocities"] for point in result["points"]]))
    accelerations = np.abs(np.asarray([point["accelerations"] for point in result["points"]]))

    assert np.all(velocities.max(axis=0) <= velocity_limits * 0.25 + 1e-8)
    assert np.all(accelerations.max(axis=0) <= acceleration_limits * 0.2 + 1e-8)
    assert result["duration"] > 0.0


def test_model_and_generator_reject_nonfinite_joint_values():
    document = new_trajectory("bad")
    document["waypoints"] = _waypoints([np.zeros(7), np.ones(7) * 0.1])
    document["waypoints"][0]["joint_positions"][3] = math.nan

    with pytest.raises(ValueError, match="有限"):
        validate_document(document)
    with pytest.raises(ValueError, match="有限"):
        generate_quintic(
            document["waypoints"], _config(), np.ones(7), np.ones(7)
        )


def test_repository_crud_overwrite_and_restart_recovery(tmp_path):
    repository = TrajectoryRepository(str(tmp_path))
    first = new_trajectory("撤离路径", trajectory_id="sampling_retreat_01")
    saved = repository.save(first)

    assert saved["trajectory_id"] == "sampling_retreat_01"
    assert (tmp_path / "sampling_retreat_01.json").is_file()
    assert repository.list()[0]["name"] == "撤离路径"

    first["name"] = "撤离路径（更新）"
    overwritten = repository.save(first)
    assert overwritten["name"] == "撤离路径（更新）"
    assert overwritten["updated_at"] >= saved["updated_at"]

    restarted = TrajectoryRepository(str(tmp_path))
    assert restarted.get("sampling_retreat_01")["name"] == "撤离路径（更新）"
    assert restarted.delete("sampling_retreat_01") is True
    assert restarted.delete("sampling_retreat_01") is False
    with pytest.raises(KeyError, match="sampling_retreat_01"):
        restarted.get("sampling_retreat_01")


def test_repository_isolates_malformed_json_and_reports_scan_error(tmp_path):
    repository = TrajectoryRepository(str(tmp_path))
    repository.save(new_trajectory("valid", trajectory_id="valid_01"))
    (tmp_path / "broken_01.json").write_text("{not json", encoding="utf-8")

    assert [item["trajectory_id"] for item in repository.list()] == ["valid_01"]
    assert "broken_01.json" in repository.scan_errors
    assert "JSON" in repository.scan_errors["broken_01.json"]


@pytest.mark.parametrize("unsafe_id", ["../escape", "with space", "/absolute", "a/b"])
def test_repository_rejects_unsafe_resource_ids(tmp_path, unsafe_id):
    repository = TrajectoryRepository(str(tmp_path))

    with pytest.raises(ValueError, match="trajectory_id"):
        repository.get(unsafe_id)
    with pytest.raises(ValueError, match="trajectory_id"):
        new_trajectory("unsafe", trajectory_id=unsafe_id)


def test_repository_atomic_save_leaves_no_temporary_file(tmp_path):
    repository = TrajectoryRepository(str(tmp_path))
    repository.save(new_trajectory("atomic", trajectory_id="atomic_01"))

    files = sorted(path.name for path in Path(tmp_path).iterdir())
    assert files == ["atomic_01.json"]
    loaded = json.loads((tmp_path / files[0]).read_text(encoding="utf-8"))
    assert loaded["trajectory_id"] == "atomic_01"


class _ServiceRobot:
    velocity_limits = {name: 1.0 for name in JOINT_NAMES}
    limits = {name: (-2.0, 2.0) for name in JOINT_NAMES}

    def limit_violations(self, values, margin=0.0):
        return [
            name for name, value in zip(JOINT_NAMES, values)
            if not self.limits[name][0] + margin <= value <= self.limits[name][1] - margin
        ]

    @staticmethod
    def validate_vector(values):
        result = np.asarray(tuple(values), dtype=float)
        if result.shape != (7,) or not np.isfinite(result).all():
            raise ValueError("invalid joints")
        return result


def _feedback(index=0, fresh=True):
    joints = [0.01 * index + 0.001 * joint for joint in range(7)]
    return {
        "fresh": {"joints": fresh, "tcp": fresh},
        "joints": dict(zip(JOINT_NAMES, joints)),
        "tcp": {
            "x": 0.40 + index * 0.001, "y": -0.02, "z": 0.31,
            "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0,
        },
        "source_stamps": {"joints": 1000.0 + index, "tcp": 1000.0 + index},
    }


def _service(
    tmp_path, snapshot_provider, collision_checker=lambda _values: False,
    executor=None, stop_executor=None,
):
    return TrajectoryService(
        TrajectoryRepository(str(tmp_path)), _ServiceRobot(), snapshot_provider,
        acceleration_limits=[2.0] * 7,
        collision_checker=collision_checker,
        collision_required=True,
        executor=executor,
        stop_executor=stop_executor,
    )


def test_service_records_exactly_twenty_single_click_feedback_samples(tmp_path):
    counter = {"value": 0}

    def snapshot():
        value = _feedback(counter["value"])
        counter["value"] += 1
        return value

    service = _service(tmp_path, snapshot)
    resource = service.create("20点路径", trajectory_id="twenty_points")
    for _ in range(20):
        resource = service.record_current_waypoint(resource["trajectory_id"])

    assert len(resource["waypoints"]) == 20
    for index, waypoint in enumerate(resource["waypoints"]):
        assert waypoint["sequence"] == index + 1
        assert waypoint["joint_positions"] == pytest.approx(
            [0.01 * index + 0.001 * joint for joint in range(7)]
        )
        assert waypoint["link7_pose"]["position"]["x"] == pytest.approx(0.40 + index * 0.001)
        assert waypoint["ros_timestamp"] == 1000.0 + index
        assert waypoint["joint_names"] == list(JOINT_NAMES)
    assert len({point["waypoint_id"] for point in resource["waypoints"]}) == 20
    assert resource["trajectory_status"]["state"] == "STALE"


@pytest.mark.parametrize(
    "mutation",
    [
        lambda value: value["fresh"].update({"joints": False}),
        lambda value: value["tcp"].pop("qw"),
        lambda value: value["joints"].pop("joint7"),
        lambda value: value["source_stamps"].update({"tcp": None}),
    ],
)
def test_service_rejects_stale_or_partial_feedback_without_saving(tmp_path, mutation):
    feedback = _feedback()
    mutation(feedback)
    service = _service(tmp_path, lambda: feedback)
    resource = service.create("invalid feedback", trajectory_id="invalid_feedback")

    with pytest.raises(RuntimeError, match="反馈|TF|时间戳|关节"):
        service.record_current_waypoint(resource["trajectory_id"])
    assert service.get(resource["trajectory_id"])["waypoints"] == []


def test_service_reorder_delete_and_clear_invalidate_generated_points(tmp_path):
    counter = {"value": 0}

    def snapshot():
        value = _feedback(counter["value"])
        counter["value"] += 1
        return value

    service = _service(tmp_path, snapshot)
    resource = service.create("edit", trajectory_id="edit_path")
    for _ in range(3):
        resource = service.record_current_waypoint("edit_path")
    original_ids = [item["waypoint_id"] for item in resource["waypoints"]]
    resource["trajectory"] = [{
        "positions": [0.0] * 7, "velocities": [0.0] * 7,
        "accelerations": [0.0] * 7, "time_from_start": 0.0,
    }]
    resource["trajectory_status"]["state"] = "READY"
    service.save(resource)

    moved = service.move_waypoint("edit_path", original_ids[2], -1)
    assert [item["waypoint_id"] for item in moved["waypoints"]] == [
        original_ids[0], original_ids[2], original_ids[1]
    ]
    assert moved["trajectory"] == []
    assert moved["trajectory_status"]["state"] == "STALE"

    deleted = service.delete_waypoint("edit_path", original_ids[2])
    assert [item["sequence"] for item in deleted["waypoints"]] == [1, 2]
    cleared = service.clear_waypoints("edit_path")
    assert cleared["waypoints"] == []
    assert cleared["trajectory_status"]["state"] == "EMPTY"


def test_service_generation_rejects_dense_collision_and_persists_error(tmp_path):
    rows = iter([_feedback(0), _feedback(20)])
    service = _service(
        tmp_path, lambda: next(rows),
        collision_checker=lambda values: float(values[0]) > 0.15,
    )
    service.create("collision", trajectory_id="collision_path")
    service.record_current_waypoint("collision_path")
    service.record_current_waypoint("collision_path")

    with pytest.raises(RuntimeError, match="碰撞"):
        service.generate("collision_path")
    failed = service.get("collision_path")
    assert failed["trajectory"] == []
    assert failed["trajectory_status"]["state"] == "ERROR"
    assert "碰撞" in failed["trajectory_status"]["message"]


def _trajectory_points():
    return [
        {
            "positions": [0.0] * 7,
            "velocities": [0.0] * 7,
            "accelerations": [0.0] * 7,
            "time_from_start": 0.0,
        },
        {
            "positions": [0.1] * 7,
            "velocities": [0.2] * 7,
            "accelerations": [0.3] * 7,
            "time_from_start": 1.25,
        },
    ]


def _adapter(collision_checker=lambda _values: False):
    config = {
        "joint_control_execution_authorized": True,
        "supervised_commissioning_authorized": True,
        "joint_control": {
            "trajectory_start_tolerance_rad": 0.05,
            "joint_limit_margin_rad": 0.02,
        },
        "trajectory_library": {"acceleration_limits_rad_s2": [1.5] * 7},
    }
    return JointControlAdapter(
        _ServiceRobot(), config,
        collision_checker=collision_checker,
        graph_checker=lambda: ["/joint_control_server"],
    )


def test_adapter_converts_dense_points_to_ros_joint_trajectory():
    message = _adapter().to_joint_trajectory(_trajectory_points())

    assert message.joint_names == list(JOINT_NAMES)
    assert len(message.points) == 2
    assert list(message.points[1].positions) == pytest.approx([0.1] * 7)
    assert list(message.points[1].velocities) == pytest.approx([0.2] * 7)
    assert list(message.points[1].accelerations) == pytest.approx([0.3] * 7)
    assert message.points[1].time_from_start.to_sec() == pytest.approx(1.25)


def test_adapter_rejects_start_mismatch_and_any_dense_limit_or_collision_failure():
    snapshot = {
        "fresh": {"joints": True},
        "ages": {"joints": 0.0},
        "joints": dict(zip(JOINT_NAMES, [0.2] * 7)),
        "task_status": {"state": "IDLE"},
    }
    result = _adapter().validate_trajectory(
        _trajectory_points(), snapshot, SimpleNamespace(value="IDLE"), None
    )
    assert result["ok"] is False
    assert any("起点" in item["name"] and not item["ok"] for item in result["checks"])

    snapshot["joints"] = dict(zip(JOINT_NAMES, [0.0] * 7))
    collision = _adapter(collision_checker=lambda values: float(values[0]) > 0.05)
    result = collision.validate_trajectory(
        _trajectory_points(), snapshot, SimpleNamespace(value="IDLE"), None
    )
    assert result["ok"] is False
    assert any("碰撞" in item["name"] and not item["ok"] for item in result["checks"])

    unsafe = _trajectory_points()
    unsafe[1]["positions"][0] = 2.1
    result = _adapter().validate_trajectory(
        unsafe, snapshot, SimpleNamespace(value="IDLE"), None
    )
    assert result["ok"] is False
    assert any("关节限位" in item["name"] and not item["ok"] for item in result["checks"])


def test_adapter_progress_is_derived_from_nearest_actual_feedback_and_never_regresses():
    positions = [np.full(7, value) for value in (0.0, 0.1, 0.2, 0.3)]

    first = JointControlAdapter.feedback_progress(np.full(7, 0.19), positions, 0)
    second = JointControlAdapter.feedback_progress(np.full(7, 0.08), positions, first["current_point"])

    assert first["current_point"] == 2
    assert first["progress"] == pytest.approx(2.0 / 3.0)
    assert first["max_error_rad"] == pytest.approx(0.01)
    assert second["current_point"] == 2


def test_service_executes_by_id_updates_progress_from_executor_and_stops(tmp_path):
    started = threading.Event()
    cancelled = threading.Event()

    def executor(points, callback):
        callback({
            "current_point": 1, "total_points": len(points),
            "progress": 0.5, "max_error_rad": 0.001,
        })
        started.set()
        assert cancelled.wait(2.0)
        return {"cancelled": True, "status": "PREEMPTED"}

    rows = iter([_feedback(0), _feedback(10)])
    service = _service(
        tmp_path, lambda: next(rows), executor=executor,
        stop_executor=lambda: cancelled.set() or {"cancel_requested": True},
    )
    service.create("execute", trajectory_id="execute_by_id")
    service.record_current_waypoint("execute_by_id")
    service.record_current_waypoint("execute_by_id")
    generated = service.generate("execute_by_id")

    accepted = service.execute("execute_by_id")
    assert accepted["accepted"] is True
    assert started.wait(1.0)
    running = service.execution_status()
    assert running["trajectory_id"] == "execute_by_id"
    assert running["current_point"] == 1
    assert running["progress"] == pytest.approx(0.5)
    assert running["total_waypoints"] == 2
    assert running["total_points"] == len(generated["trajectory"])
    with pytest.raises(RuntimeError, match="执行中"):
        service.delete("execute_by_id")

    stopped = service.stop()
    assert stopped["cancel_requested"] is True
    deadline = time.monotonic() + 2.0
    while service.execution_status()["state"] == "EXECUTING" and time.monotonic() < deadline:
        time.sleep(0.01)
    assert service.execution_status()["state"] == "CANCELLED"
    assert service.get("execute_by_id")["trajectory_status"]["state"] == "CANCELLED"
