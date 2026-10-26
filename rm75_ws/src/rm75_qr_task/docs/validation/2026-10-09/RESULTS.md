# rm75_qr_task validation — 2026-10-09

- Environment: Docker `rm75_noetic`, ROS 1 Noetic.
- Build command: `catkin build rm75_qr_task --no-status`.
- Build result: 1/1 package succeeded, no build warning.
- Test command: `catkin run_tests rm75_qr_task --no-status` followed by
  `catkin_test_results build/rm75_qr_task`.
- Test result: 16 tests, 0 errors, 0 failures, 0 skipped.
- Host pure-Python result: the same 16 tests passed.
- Mock ROS launch: server reached `IDLE`; CLI `status` succeeded; `pick 0`
  without a stable observation was rejected and remained `IDLE`.
- Marker-node smoke: `start_marker:=true` started the AprilTag/OpenCV node and
  advertised all documented pose, quality, observation, and TF interfaces;
  no camera data was injected.
- Gazebo: not run; the new package has no commissioned station geometry.
- Hardware: not run by design; no motion and no 24V/0V command was sent.
- Remaining blockers: all physical marker/TCP/tool/object/payload values,
  independent hold/release sensor, hand-eye physical validation, collision and
  stopping validation, and single-command-source verification.
