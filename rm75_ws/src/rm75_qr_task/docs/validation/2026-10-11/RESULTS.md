# RM75 marker print and commissioning-status update — 2026-10-11

- Generated `output/pdf/rm75_apriltag36h11_slots_0_3_A4_80mm.pdf`.
- Four pages, exact A4 page boxes (210 × 297 mm), one AprilTag 36h11 ID per
  page in order 0, 1, 2, 3.
- Nominal outer black-square side: 80.0 mm; module size: 10.0 mm.
- Each page contains a 100 mm verification scale and 100% print instructions.
- Rendered all pages at 150 dpi and visually checked for clipping, overlap,
  quiet-zone obstruction, and legibility.
- OpenCV AprilTag 36h11 detection found exactly one expected ID per page.
- Detected rendered side lengths: 471–472 px; theoretical 80 mm at 150 dpi is
  472.44 px.
- `markers.yaml` now contains nominal `size_m: 0.080`, but every station remains
  `enabled: false` and `calibrated: false` pending measurement after printing
  and mounting.
- Based on explicit user confirmation, `hand_eye_calibrated`,
  `voltage_interface_verified`, and `single_command_source_verified` are true.
- Remaining readiness gates stay false; no robot motion or voltage command was
  issued during this update.
- `catkin build rm75_qr_task`: succeeded with no build warnings.
- `catkin run_tests rm75_qr_task`: 16 tests, 0 errors, 0 failures, 0 skipped.
