# Supervised experiment protocol

For every attempt:

1. Confirm RM75-6FB identity, ROS time source, `base_link -> Link7`, clear
   workcell, reachable E-stop, one command publisher, force feedback and fresh
   joint feedback.
2. Run the chosen command once without `execute:=true`; review preflight,
   collision result, start/target pose, IK and duration.
3. With a human continuously supervising, rerun that one attempt with the
   explicit `execute:=true` argument.
4. Preserve the raw CSV, JSON and full console log even for STOP/FAIL.
5. Return to the reviewed start pose using the existing safe procedure before
   changing direction. Do not run the 100 attempts as an unattended loop.

Order: X low-speed ±, Y low-speed ±, Z low-speed ±, Rx ±, Ry ±, Rz ±, then a
small combined 6D motion. Only after reviewing low-speed stopping and errors may
the same translational set be repeated at 0.03 m/s.

STOP experiments are separate, supervised trials:

- tracking error: use an intentionally lower *temporary launch parameter* only
  after dry-run review; record request time, last moving sample and distance;
- timeout: interrupt only the approved feedback relay/simulator, never modify
  `rm_driver`; verify fail-closed behavior and recovery procedure;
- collision: first use PlanningScene simulation; a physical obstacle/contact
  is not part of this stage.

No Touch, force control, soil contact, tool voltage or unattended execution is
permitted.
