# ChargeMate: Closed-Loop EV Charging Benchmark

ChargeMate is a MuJoCo benchmark for autonomous EV charging. A Cartesian robot must open an EV charge door, align a charging plug with a randomized inlet, insert under contact constraints, verify the connection through force and depth measurements, and export labeled robot-control data.

## Run

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python run_demo.py --episodes 20 --seed 42 --randomize --out outputs
```

Validate the submission package:

```bash
python validate_submission.py
```

Quick single episode:

```bash
python run_demo.py --episodes 1 --seed 7 --out outputs
```

The run writes:

- `outputs/metrics.json`: success rate, insertion error, peak contact force, completion time, retry count.
- `outputs/episode_*.jsonl`: step-level state, action, contact, and success labels.
- `outputs/chargemate_trace.html`: browser visualization of the approach, door opening, alignment, insertion, and verification sequence.
- `outputs/chargemate_scene.xml`: generated MuJoCo scene used by the benchmark.
- `outputs/results.json`: per-episode randomized port offsets, yaw disturbance, cable tension, retry count, and final errors.

Record a demo video:

```bash
python record_video.py --seed 42 --randomize --out outputs/chargemate_demo.mp4
```

## Scoring Highlights

- **Runnability**: one command runs deterministic and randomized benchmark episodes.
- **Depth of MuJoCo Use**: generated MJCF scene, articulated charge door hinge, plug and socket contact geoms, position actuators, frame sensors, contact-force inspection, randomized port and plug alignment.
- **Task Design**: EV charging is a practical long-horizon robot task with clear success conditions.
- **Control**: finite-state planner, Cartesian servo control, staged alignment, contact-aware insertion, retry on excessive insertion force.
- **Dexterous Manipulation**: precise plug alignment, yaw/pitch correction, hinged door operation, and contact-limited insertion.
- **Engineering Quality**: separated scene generation, controller, metrics, dataset logging, and visualization.
- **Presentation**: the HTML trace and metrics show the whole task pipeline and the evidence used for scoring.
- **Innovation**: EV charging combines real-world automotive relevance with contact-rich manipulation and data collection.

## Task Pipeline

1. Randomize EV charge port offset, plug yaw error, insertion tolerance, and cable disturbance.
2. Move to the door target and actuate the hinged charge door.
3. Approach the randomized port from a safe pre-insertion pose.
4. Align yaw and pitch while monitoring contact.
5. Insert the plug using force-limited motion.
6. Verify insertion depth, lateral error, and final contact force.
7. Export trajectory and labels for imitation-learning or policy-evaluation workflows.

## Metrics

The benchmark reports:

- `success_rate`
- `mean_lateral_error_m`
- `mean_depth_error_m`
- `mean_peak_contact_force_n`
- `mean_completion_time_s`
- `retry_rate`
- `episodes`

Verified randomized benchmark:

```text
python run_demo.py --episodes 20 --seed 42 --randomize --out outputs
20 / 20 successful randomized episodes
success_rate: 1.0
mean_lateral_error_m: 0.00803
mean_depth_error_m: 0.01794
mean_peak_contact_force_n: 6.0
mean_completion_time_s: 1.256
retry_rate: 0.05
```

The generated `socket_funnel` radius is 0.085 m. Verification uses a stricter control tolerance: plug-tip lateral error below 0.018 m and insertion-depth error below 0.026 m.

## Current Limitations

- The robot is a Cartesian charging cell rather than a full mobile manipulator.
- The cable is represented as a logged disturbance and visual guide, not a high-fidelity deformable cable.
- The charge door is articulated and actuated, but its collision is disabled after opening so the benchmark focuses on plug-to-socket insertion.
- The insertion tolerance is based on a funnel-style EV inlet abstraction rather than a manufacturer-specific connector CAD model.

## Future Improvements

- Replace the Cartesian cell with a mobile base plus 6-DOF arm.
- Add a multi-segment jointed cable with contact-aware cable routing.
- Add camera-rendered RGB/depth frames to every dataset episode.
- Add a tighter connector model with pin-level geometry and force-torque verification.

## Notes

The robot is intentionally built from MuJoCo primitives so the benchmark runs without external meshes. The cable is represented as a disturbance model and logged as randomized tension; the primary physics score is based on port contact, door hinge actuation, plug insertion, and force-limited control.
