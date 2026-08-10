# Continuous grasp baseline test

This runner performs repeated, operator-labelled Camera–RPLIDAR–RoArm grasp
trials without reopening all devices between trials. It is additive: the normal
`run_modular_pipeline.sh` entry point and committed safety profile are unchanged.

## Safety model

- T=122 custom Home runs once at session startup unless `--skip-home` is used.
- Real mode requires the session phrase `EXECUTE CONTINUOUS GRASP TEST`.
- Every physical trial requires a separate uppercase `RUN` gate.
- Only one task can run at a time. `Ctrl+C` requests an arm stop.
- The generated session config sets `one_grasp_per_process: false`; the committed
  `configs/modular_pipeline.yaml` remains `true`.
- Keep the complete swept volume clear and the hardware power switch reachable.

## Validate without opening devices

```bash
cd /workspace/src
bash ./run_continuous_grasp_test.sh --validate-config
```

## Start a baseline session

```bash
cd /workspace/src
bash ./run_continuous_grasp_test.sh \
  --session-label workspace-3x3-baseline-v1
```

Recommended position labels are `P11` through `P33`. Repeat the same label for
three to five trials at each physical position; `trial_number` and `run_id`
remain unique.

For each trial:

1. Enter the position label and optional environment note.
2. Place one `tissue_pack` in the workspace and clear the arm sweep volume.
3. Enter uppercase `RUN`.
4. Wait for the terminal `COMPLETE`, `ERROR`, or `ABORTED` state.
5. Enter `y`, `n`, or `u` for successful, failed, or uncertain grasp.
6. Record the visible failure mode, such as `missed left edge`, `slipped after
   lift`, or `false detection`.
7. Reposition the target and continue. Enter `q` before a trial to end cleanly.

## Output

Each invocation creates one immutable session directory:

```text
/workspace/logs/baseline_grasp/baseline-<UTC>-<id>/
├── effective_config.yaml
├── session.json
├── runs.jsonl
├── runs.csv
├── trials.csv
├── events.jsonl
└── summary.json
```

- `runs.jsonl` is the detailed source of truth: configuration and Git commit,
  health checks, YOLO result, RPLIDAR quality, raw and calibrated targets,
  recheck shifts, waypoints, RoArm feedback, duration, and errors.
- `runs.csv` is the flattened analysis table.
- `trials.csv` adds physical position labels and human outcomes.
- `events.jsonl` records observed pipeline state transitions.
- `summary.json` is regenerated after every annotated trial and includes failure
  types and human-judged success rate.

Do not edit `runs.jsonl` while the session is active. If the terminal closes,
all trials already reaching a terminal state remain on disk.
