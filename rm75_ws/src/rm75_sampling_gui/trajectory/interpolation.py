"""C2 piecewise-quintic interpolation through fixed RM75 joint waypoints."""

import math
from typing import Any, Dict, Iterable, List, Sequence

import numpy as np


def _vector(value: Iterable[float], label: str, positive: bool = False) -> np.ndarray:
    result = np.asarray(tuple(value), dtype=float)
    if result.shape != (7,) or not np.isfinite(result).all():
        raise ValueError("{}必须是有限七维向量".format(label))
    if positive and np.any(result <= 0.0):
        raise ValueError("{}必须全部为正有限数".format(label))
    return result


def _positive_scale(config: Dict[str, Any], key: str) -> float:
    value = float(config.get(key, 0.0))
    if not math.isfinite(value) or not 0.0 < value <= 1.0:
        raise ValueError("{}必须在(0, 1]范围".format(key))
    return value


def _durations(
    positions: np.ndarray,
    velocity_limits: np.ndarray,
    acceleration_limits: np.ndarray,
    period: float,
) -> np.ndarray:
    distance = np.abs(np.diff(positions, axis=0))
    velocity_time = 1.875 * distance / velocity_limits
    acceleration_time = np.sqrt(5.8 * distance / acceleration_limits)
    return np.maximum(
        np.maximum(np.max(velocity_time, axis=1), np.max(acceleration_time, axis=1)),
        max(2.0 * period, 1e-3),
    )


def _knot_velocities(positions: np.ndarray, durations: np.ndarray) -> np.ndarray:
    count = positions.shape[0]
    velocities = np.zeros_like(positions)
    slopes = np.diff(positions, axis=0) / durations[:, None]
    for index in range(1, count - 1):
        before, after = slopes[index - 1], slopes[index]
        same_direction = before * after > 0.0
        h_before, h_after = durations[index - 1], durations[index]
        weight_before = 2.0 * h_after + h_before
        weight_after = h_after + 2.0 * h_before
        harmonic = np.zeros(7, dtype=float)
        harmonic[same_direction] = (
            (weight_before + weight_after)
            / (weight_before / before[same_direction] + weight_after / after[same_direction])
        )
        velocities[index] = harmonic
    return velocities


def quintic_coefficients(
    start: np.ndarray,
    end: np.ndarray,
    start_velocity: np.ndarray,
    end_velocity: np.ndarray,
    start_acceleration: np.ndarray,
    end_acceleration: np.ndarray,
    duration: float,
) -> np.ndarray:
    delta = end - start
    t = float(duration)
    coefficients = np.empty((6, 7), dtype=float)
    coefficients[0] = start
    coefficients[1] = start_velocity
    coefficients[2] = 0.5 * start_acceleration
    coefficients[3] = (
        20.0 * delta
        - (12.0 * start_velocity + 8.0 * end_velocity) * t
        - (3.0 * start_acceleration - end_acceleration) * t * t
    ) / (2.0 * t ** 3)
    coefficients[4] = (
        -30.0 * delta
        + (16.0 * start_velocity + 14.0 * end_velocity) * t
        + (3.0 * start_acceleration - 2.0 * end_acceleration) * t * t
    ) / (2.0 * t ** 4)
    coefficients[5] = (
        12.0 * delta
        - 6.0 * (start_velocity + end_velocity) * t
        - (start_acceleration - end_acceleration) * t * t
    ) / (2.0 * t ** 5)
    return coefficients


def evaluate_quintic(coefficients: Sequence[Sequence[float]], time_value: float) -> Dict[str, np.ndarray]:
    """Evaluate position, velocity and acceleration at local segment time."""

    values = np.asarray(coefficients, dtype=float)
    if values.shape != (6, 7) or not np.isfinite(values).all():
        raise ValueError("五次多项式系数必须是有限6x7矩阵")
    t = float(time_value)
    if not math.isfinite(t) or t < 0.0:
        raise ValueError("插值时间必须是非负有限数")
    powers = np.asarray([1.0, t, t ** 2, t ** 3, t ** 4, t ** 5])
    velocity_powers = np.asarray([0.0, 1.0, 2.0 * t, 3.0 * t ** 2, 4.0 * t ** 3, 5.0 * t ** 4])
    acceleration_powers = np.asarray([0.0, 0.0, 2.0, 6.0 * t, 12.0 * t ** 2, 20.0 * t ** 3])
    return {
        "positions": powers.dot(values),
        "velocities": velocity_powers.dot(values),
        "accelerations": acceleration_powers.dot(values),
    }


def _segments(positions: np.ndarray, durations: np.ndarray) -> List[np.ndarray]:
    velocities = _knot_velocities(positions, durations)
    accelerations = np.zeros_like(positions)
    return [
        quintic_coefficients(
            positions[index], positions[index + 1],
            velocities[index], velocities[index + 1],
            accelerations[index], accelerations[index + 1], durations[index],
        )
        for index in range(len(durations))
    ]


def trajectory_coefficients(points: Sequence[Dict[str, Any]]) -> List[np.ndarray]:
    """Reconstruct each quintic from stored position/velocity/acceleration.

    Dense samples from one polynomial reconstruct that same polynomial, so the
    action server and simulator preserve both taught knots and C2 continuity.
    """
    result = []
    for start, end in zip(points, points[1:]):
        duration = float(end["time_from_start"]) - float(start["time_from_start"])
        if not math.isfinite(duration) or duration <= 0.0:
            raise ValueError("轨迹时间必须严格递增")
        result.append(quintic_coefficients(
            _vector(start["positions"], "positions"), _vector(end["positions"], "positions"),
            _vector(start["velocities"], "velocities"), _vector(end["velocities"], "velocities"),
            _vector(start["accelerations"], "accelerations"),
            _vector(end["accelerations"], "accelerations"), duration,
        ))
    return result


def segment_check_times(coefficients: np.ndarray, duration: float) -> List[float]:
    """Include exact position/velocity/acceleration extrema and 20 ms samples."""
    stamps = set(np.linspace(0.0, duration, max(2, int(math.ceil(duration / 0.02)) + 1)))
    for joint in range(7):
        polynomial = coefficients[:, joint]
        for _ in range(3):
            polynomial = np.polynomial.polynomial.polyder(polynomial)
            for root in np.polynomial.polynomial.polyroots(polynomial):
                if abs(root.imag) < 1e-9 and 0.0 < root.real < duration:
                    stamps.add(float(root.real))
    return sorted(stamps)


def _sample_maxima(segments: List[np.ndarray], durations: np.ndarray, period: float) -> Dict[str, np.ndarray]:
    max_velocity = np.zeros(7, dtype=float)
    max_acceleration = np.zeros(7, dtype=float)
    for coefficients, duration in zip(segments, durations):
        count = max(101, int(math.ceil(float(duration) / max(period * 0.25, 1e-4))) + 1)
        for stamp in np.linspace(0.0, float(duration), count):
            evaluated = evaluate_quintic(coefficients, float(stamp))
            max_velocity = np.maximum(max_velocity, np.abs(evaluated["velocities"]))
            max_acceleration = np.maximum(max_acceleration, np.abs(evaluated["accelerations"]))
    return {"velocity": max_velocity, "acceleration": max_acceleration}


def generate_quintic(
    waypoints: Sequence[Dict[str, Any]],
    config: Dict[str, Any],
    velocity_limits: Iterable[float],
    acceleration_limits: Iterable[float],
) -> Dict[str, Any]:
    """Generate dense C2 samples that pass every joint waypoint exactly."""

    if not isinstance(waypoints, (list, tuple)) or len(waypoints) < 2:
        raise ValueError("生成轨迹至少需要两个示教点")
    positions = np.asarray([
        _vector(waypoint.get("joint_positions", ()), "示教点关节位置")
        for waypoint in waypoints
    ])
    limits_v = _vector(velocity_limits, "关节速度限制", positive=True)
    limits_a = _vector(acceleration_limits, "关节加速度限制", positive=True)
    speed_scale = _positive_scale(config, "speed_scale")
    acceleration_scale = _positive_scale(config, "acceleration_scale")
    period = float(config.get("sample_period_s", 0.0))
    if not math.isfinite(period) or period <= 0.0:
        raise ValueError("sample_period_s必须是正有限数")
    target_v = limits_v * speed_scale
    target_a = limits_a * acceleration_scale
    durations = _durations(positions, target_v, target_a, period)

    for _attempt in range(24):
        segments = _segments(positions, durations)
        maxima = _sample_maxima(segments, durations, period)
        velocity_ratio = float(np.max(maxima["velocity"] / target_v))
        acceleration_ratio = float(np.max(maxima["acceleration"] / target_a))
        scale = max(velocity_ratio, math.sqrt(acceleration_ratio))
        if scale <= 1.0 + 1e-10:
            break
        durations *= max(1.001, scale * 1.002)
    else:
        raise ValueError("轨迹时间缩放未能满足速度/加速度限制")

    knot_velocities = _knot_velocities(positions, durations)
    points: List[Dict[str, Any]] = []
    waypoint_indices = []
    elapsed = 0.0
    for segment_index, (coefficients, duration) in enumerate(zip(segments, durations)):
        count = max(2, int(math.ceil(float(duration) / period)) + 1)
        local_times = np.linspace(0.0, float(duration), count)
        if segment_index:
            local_times = local_times[1:]
        else:
            waypoint_indices.append(0)
        for local_time in local_times:
            evaluated = evaluate_quintic(coefficients, float(local_time))
            points.append({
                "positions": evaluated["positions"].tolist(),
                "velocities": evaluated["velocities"].tolist(),
                "accelerations": evaluated["accelerations"].tolist(),
                "time_from_start": float(elapsed + local_time),
            })
        waypoint_indices.append(len(points) - 1)
        elapsed += float(duration)

    # Floating-point polynomial evaluation is replaced by the defining
    # constraints at knots so persisted points preserve taught values exactly.
    for index, point_index in enumerate(waypoint_indices):
        points[point_index]["positions"] = positions[index].tolist()
        points[point_index]["velocities"] = knot_velocities[index].tolist()
        points[point_index]["accelerations"] = [0.0] * 7
        points[point_index]["time_from_start"] = float(np.sum(durations[:index]))

    maxima = _sample_maxima(segments, durations, period)
    return {
        "points": points,
        "duration": float(np.sum(durations)),
        "waypoint_indices": waypoint_indices,
        "segment_durations": durations.tolist(),
        "segment_coefficients": [coefficients.tolist() for coefficients in segments],
        "limit_summary": {
            "max_velocity": maxima["velocity"].tolist(),
            "max_acceleration": maxima["acceleration"].tolist(),
            "velocity_limits": target_v.tolist(),
            "acceleration_limits": target_a.tolist(),
        },
    }
