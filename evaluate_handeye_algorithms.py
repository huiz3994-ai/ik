#!/usr/bin/env python3
import math
import sys

import numpy as np
import rospy
from easy_handeye_msgs.srv import ComputeCalibration, ListAlgorithms, SetAlgorithm, TakeSample
from std_srvs.srv import Empty
from transforms3d.quaternions import quat2mat


def matrix_from_transform(transform):
    if hasattr(transform, "transform"):
        transform = transform.transform
    matrix = np.eye(4)
    q = transform.rotation
    matrix[:3, :3] = quat2mat([q.w, q.x, q.y, q.z])
    t = transform.translation
    matrix[:3, 3] = [t.x, t.y, t.z]
    return matrix


def residuals(samples, calibration, include_errors=False):
    x_hand_camera = matrix_from_transform(calibration.transform)
    marker_poses = []
    for hand_world, camera_marker in zip(
        samples.hand_world_samples, samples.camera_marker_samples
    ):
        marker_poses.append(
            matrix_from_transform(hand_world)
            @ x_hand_camera
            @ matrix_from_transform(camera_marker)
        )

    translations = np.array([pose[:3, 3] for pose in marker_poses])
    translation_errors = np.linalg.norm(translations - translations.mean(axis=0), axis=1)

    rotation_sum = sum(pose[:3, :3] for pose in marker_poses)
    u, _, vt = np.linalg.svd(rotation_sum)
    rotation_mean = u @ vt
    if np.linalg.det(rotation_mean) < 0:
        u[:, -1] *= -1
        rotation_mean = u @ vt
    rotation_errors = []
    for pose in marker_poses:
        delta = rotation_mean.T @ pose[:3, :3]
        cosine = np.clip((np.trace(delta) - 1.0) / 2.0, -1.0, 1.0)
        rotation_errors.append(math.degrees(math.acos(cosine)))
    rotation_errors = np.array(rotation_errors)
    result = {
        "translation_rms_mm": float(np.sqrt(np.mean(translation_errors**2)) * 1000.0),
        "translation_max_mm": float(translation_errors.max() * 1000.0),
        "rotation_rms_deg": float(np.sqrt(np.mean(rotation_errors**2))),
        "rotation_max_deg": float(rotation_errors.max()),
    }
    if include_errors:
        result["translation_errors_mm"] = (translation_errors * 1000.0).tolist()
        result["rotation_errors_deg"] = rotation_errors.tolist()
    return result


def main():
    namespace = sys.argv[1] if len(sys.argv) > 1 else "/rm75_d435i_high_precision_eye_on_hand"
    rospy.init_node("evaluate_handeye_algorithms", anonymous=True)
    list_algorithms = rospy.ServiceProxy(namespace + "/list_algorithms", ListAlgorithms)
    set_algorithm = rospy.ServiceProxy(namespace + "/set_algorithm", SetAlgorithm)
    compute = rospy.ServiceProxy(namespace + "/compute_calibration", ComputeCalibration)
    get_samples = rospy.ServiceProxy(namespace + "/get_sample_list", TakeSample)
    save = rospy.ServiceProxy(namespace + "/save_calibration", Empty)

    services = [list_algorithms, set_algorithm, compute, get_samples, save]
    for service in services:
        service.wait_for_service(timeout=10.0)

    sample_list = get_samples().samples
    sample_count = len(sample_list.hand_world_samples)
    results = []
    for algorithm in list_algorithms().algorithms:
        if not set_algorithm(algorithm).success:
            continue
        response = compute()
        if not response.valid:
            continue
        metrics = residuals(sample_list, response.calibration)
        results.append((metrics["translation_rms_mm"], metrics["rotation_rms_deg"], algorithm, response.calibration, metrics))
        print(
            "RESULT {} samples={} trans_rms_mm={:.3f} trans_max_mm={:.3f} "
            "rot_rms_deg={:.4f} rot_max_deg={:.4f}".format(
                algorithm, sample_count, metrics["translation_rms_mm"], metrics["translation_max_mm"],
                metrics["rotation_rms_deg"], metrics["rotation_max_deg"]
            )
        )

    if not results:
        raise RuntimeError("No hand-eye algorithm returned a valid calibration")
    best = min(results, key=lambda item: (item[0], item[1]))
    _, _, algorithm, calibration, metrics = best
    set_algorithm(algorithm)
    final = compute()
    if not final.valid:
        raise RuntimeError("Best algorithm failed during final computation")
    save()
    transform = calibration.transform.transform
    print(
        "BEST {} xyz=[{:.9f}, {:.9f}, {:.9f}] qxyzw=[{:.9f}, {:.9f}, {:.9f}, {:.9f}]".format(
            algorithm,
            transform.translation.x, transform.translation.y, transform.translation.z,
            transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w,
        )
    )
    print("BEST_METRICS {}".format(metrics))
    detail = residuals(sample_list, calibration, include_errors=True)
    for index, (translation_error, rotation_error) in enumerate(
        zip(detail["translation_errors_mm"], detail["rotation_errors_deg"])
    ):
        print("SAMPLE {:02d} trans_mm={:.3f} rot_deg={:.4f}".format(index, translation_error, rotation_error))


if __name__ == "__main__":
    main()
