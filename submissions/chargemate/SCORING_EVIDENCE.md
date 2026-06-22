# ChargeMate Scoring Evidence

Registration UUID: `0d354824-2ac1-41cf-a777-583ce7b93a4e`

Participant name: `minernote`

Project name: `ChargeMate: Closed-Loop EV Charging Benchmark`

## Reproducibility

Run the full randomized benchmark:

```bash
python run_demo.py --episodes 20 --seed 42 --randomize --out outputs
```

Run the package validator:

```bash
python validate_submission.py
```

Evidence files:

- `registration.json`
- `requirements.txt`
- `outputs/metrics.json`
- `outputs/results.json`
- `outputs/episode_*.jsonl`
- `outputs/chargemate_scene.xml`
- `outputs/chargemate_trace.html`

## MuJoCo Depth

Implemented in:

- `chargemate/scene.py`
- `chargemate/benchmark.py`

MuJoCo features used:

- Generated MJCF scene without external mesh dependencies.
- Hinged charge door joint with position actuator.
- Cartesian robot slides plus yaw and pitch wrist joints.
- Plug, socket, funnel, and EV panel contact geoms.
- Frame sensors for plug tip and port center.
- Contact-force extraction through `mujoco.mj_contactForce`.
- Per-episode randomized scene generation.

## Task Design

ChargeMate models an autonomous EV charging workflow:

- Move from stow pose.
- Reach and open the charge door.
- Approach a randomized inlet.
- Align plug pose.
- Insert under contact constraints.
- Verify final insertion by position and depth.
- Export labeled trajectory data.

Randomized parameters:

- Charge port y offset.
- Charge port z offset.
- Charge port yaw.
- Cable tension disturbance.

## Control

Implemented in:

- `chargemate/controller.py`

Controller behavior:

- Finite-state planner: `stow`, `door_reach`, `door_open`, `pre_align`, `fine_align`, `insert`, `verify`, `recover`, `success`, `failed`.
- Closed-loop transitions based on measured MuJoCo site positions and door angle.
- Contact-aware recovery when insertion force exceeds the configured limit.
- Verification tolerance: lateral error below 0.018 m and depth error below 0.026 m.

## Engineering Quality

Code separation:

- Scene generation: `chargemate/scene.py`
- Controller: `chargemate/controller.py`
- Benchmark and metrics: `chargemate/benchmark.py`
- Trace visualization: `chargemate/visualize.py`
- Demo runner: `run_demo.py`
- Dataset collection: `collect_dataset.py`
- Video recording: `record_video.py`
- Submission validation: `validate_submission.py`

## Presentation

Generated presentation artifacts:

- `outputs/chargemate_trace.html`: browser-readable trajectory and stage trace.
- `outputs/chargemate_demo.mp4`: rendered MuJoCo demo video.
- `outputs/metrics.json`: aggregate benchmark metrics.
- `PR_BODY.md`: pull request summary with UUID and benchmark evidence.

## Innovation

The project focuses on a practical EV charging task rather than a generic pick-and-place scene. The benchmark combines automotive relevance, long-horizon sequencing, contact-limited insertion, randomized port placement, and data export for later imitation-learning or policy-evaluation workflows.
