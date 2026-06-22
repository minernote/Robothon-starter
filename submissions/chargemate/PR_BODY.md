Registration UUID: `0d354824-2ac1-41cf-a777-583ce7b93a4e`

## Project name

ChargeMate: Closed-Loop EV Charging Benchmark

## Robot platform

Custom MuJoCo Cartesian charging robot built entirely from MJCF primitives. The scene includes an EV side panel, a randomized charge port, a hinged charge door, a charging plug, a cable-disturbance model, position actuators, contact geoms, and frame sensors for the plug tip and port center.

## Task goal

Open the EV charge door, align a charging plug to a randomized inlet, insert through the funnel tolerance, verify insertion with measured position/contact signals, and export labeled control data.

## Technical approach

- Finite-state task planner: stow -> door reach -> door open -> pre-align -> fine-align -> insert -> verify.
- Closed-loop Cartesian servo targets from measured MuJoCo site positions.
- Contact-aware insertion: the controller monitors contact force and can retry when insertion force exceeds the configured limit.
- Randomized benchmark: port y/z offset, port yaw, and cable tension disturbance.
- Data export: each episode writes time, stage, plug tip pose, port pose, controls, door angle, contact force, and success label.
- Verification tolerance is tied to the generated socket funnel geometry: funnel radius 0.085 m, accepted plug-tip lateral error below 0.018 m and insertion-depth error below 0.026 m.

## Benchmark evidence

Command:

```bash
python run_demo.py --episodes 20 --seed 42 --randomize --out outputs
```

Observed result:

- 20 / 20 successful randomized episodes.
- 100% success rate.
- Mean lateral error: 0.00803 m.
- Mean insertion-depth error: 0.01794 m.
- Mean peak contact force: 6.0 N, clipped at 120.0 N for reporting.
- Retry rate: 0.05.
- Mean completion time: 1.256 s.
- Data logs: `outputs/episode_*.jsonl`.
- Metrics: `outputs/metrics.json`.
- Trace visualization: `outputs/chargemate_trace.html`.

Validation:

```bash
python validate_submission.py
```

## Demo video

```bash
python record_video.py --seed 42 --randomize --out outputs/chargemate_demo.mp4
```

## AI tools used

Codex.

## Current limitations

- Cartesian charging cell rather than a full mobile manipulator.
- Cable disturbance is modeled and logged, but not simulated as a high-fidelity deformable cable.
- The charge door is articulated and actuated; collision is disabled after opening so the benchmark focuses on plug-to-socket insertion.
- The inlet is a funnel-style EV connector abstraction, not a manufacturer-specific connector CAD model.

## Future improvements

- Mobile base plus 6-DOF arm.
- Jointed cable with contact-aware cable routing.
- RGB/depth dataset export.
- Pin-level connector geometry and force-torque verification.
