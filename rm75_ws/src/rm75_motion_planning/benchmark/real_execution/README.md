# Stage 4 real-robot characterization (`base_link -> Link7`)

This directory contains the supervised RM75-6FB characterization workflow. It
does not contain fabricated measurements, and none of its offline tools can
move the robot.

## Layout

- `experiments/`: operator checklist and one-run command pattern.
- `configs/`: XYZ, RPY and combined 6D protocol manifests. Every manifest is
  disarmed (`execute: false`).
- `results/`: raw CSV and per-run JSON written during supervised execution.
- `plots/`: generated SVG plots.
- `analyze_characterization.py`: raw-cycle analysis, response-delay model,
  error decomposition, report and Guard recommendation generator.

The older `record_real_trial.py` / `analyze_real_benchmark.py` pair is retained
for compatibility with Stage 3 console-summary evidence. Stage 4 conclusions
must use the raw-cycle analyzer.

## One supervised run

Create the results directory, choose exactly one motion from a reviewed
manifest, and keep the physical E-stop and workcell in view. The following
example is a dry-run because `execute` remains false:

```bash
roslaunch rm75_motion_planning cartesian_commissioning.launch \
  dx:=0.01 scenario:=x repetition:=1 run_id:=x-plus-001 \
  raw_output_csv:=$(rospack find rm75_motion_planning)/benchmark/real_execution/results/x-plus-001.csv \
  run_summary_json:=$(rospack find rm75_motion_planning)/benchmark/real_execution/results/x-plus-001.json
```

Only after the preflight output has been reviewed and a human supervisor is
ready may that same single command add `execute:=true`. Never wrap it in an
unattended loop. STOP and FAIL attempts count toward the 100 attempts; do not
discard them to collect 100 successes.

Use 0.01 m/s first. The 0.03 m/s set is a separate escalation and is permitted
only after the low-speed dataset and stop behavior have been reviewed.

## Analysis

```bash
python3 analyze_characterization.py results/*.csv \
  --output-dir results/report --plots-dir plots
```

The command writes aggregate CSV/JSON/Markdown, an SVG plot, and
`runtime_guard.recommended.yaml`. It exits `3` and marks every recommendation
`provisional_do_not_deploy` until all seven scenarios have at least 100
attempts and both signs are represented for every single-axis scenario.

Delay is identified from joint-speed cross-correlation and peak alignment.
Command WallTime, local feedback receipt WallTime, and the JointState ROS
source timestamp are stored separately. Source-to-receipt offset is reported as
a clock/transport diagnostic; neither that offset nor a raw timestamp
subtraction is labelled controller delay.

`runtime_guard.recommended.yaml` is evidence, not an automatic configuration
update. Attempt-count completion alone cannot make it eligible: every scenario
must also contain 100 normal PASS runs (with both signs for single-axis tests).
A human must review the complete dataset, stop tests and workcell risk before
copying any eligible value into `config/runtime_guard.yaml`.
