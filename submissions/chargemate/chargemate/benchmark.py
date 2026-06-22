from __future__ import annotations

import argparse
import json
import math
import random
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from .controller import ChargeMatePlanner
from .scene import SceneRandomization, build_scene_xml, write_scene_xml
from .visualize import write_trace_html


@dataclass
class EpisodeResult:
    episode: int
    seed: int
    success: bool
    failed_reason: str
    lateral_error_m: float
    depth_error_m: float
    peak_contact_force_n: float
    completion_time_s: float
    retry_count: int
    port_y: float
    port_z: float
    port_yaw: float
    cable_tension: float


def _import_mujoco():
    try:
        import mujoco  # type: ignore
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "MuJoCo is not installed. Run: pip install -r requirements.txt"
        ) from exc
    return mujoco


def make_randomization(seed: int, enabled: bool) -> SceneRandomization:
    rng = random.Random(seed)
    if not enabled:
        return SceneRandomization(port_y=0.0, port_z=0.0, port_yaw=0.0, cable_tension=0.0)
    return SceneRandomization(
        port_y=rng.uniform(-0.035, 0.035),
        port_z=rng.uniform(-0.025, 0.030),
        port_yaw=rng.uniform(-0.045, 0.045),
        cable_tension=rng.uniform(-0.35, 0.35),
    )


def run_episode(episode: int, seed: int, out_dir: Path, randomize: bool) -> tuple[EpisodeResult, list[dict]]:
    mujoco = _import_mujoco()
    rand = make_randomization(seed, randomize)
    model = mujoco.MjModel.from_xml_string(build_scene_xml(rand))
    data = mujoco.MjData(model)

    site_tip = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "plug_tip_site")
    site_port = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "port_center")
    door_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "door_hinge")
    actuator_ids = {name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in ["x_act", "y_act", "z_act", "yaw_act", "pitch_act", "door_act"]}
    tracked_geoms = _tracked_contact_geoms(mujoco, model)

    mujoco.mj_forward(model, data)
    port_world = np.array(data.site_xpos[site_port], dtype=float)
    planner = ChargeMatePlanner(port_world=port_world, port_yaw=rand.port_yaw, cable_tension=rand.cable_tension)

    dt = float(model.opt.timestep)
    max_steps = int(10.0 / dt)
    peak_force = 0.0
    current_force = 0.0
    trace: list[dict] = []

    for step in range(max_steps):
        tip = np.array(data.site_xpos[site_tip], dtype=float)
        door_angle = float(data.qpos[model.jnt_qposadr[door_joint]])
        target = planner.update(dt, tip, door_angle, current_force)
        # actuator 控制的是滑轨关节坐标，planner 输出的是插头 tip 的世界坐标目标。
        data.ctrl[actuator_ids["x_act"]] = np.clip(target.x + 0.217, -0.05, 1.35)
        data.ctrl[actuator_ids["y_act"]] = np.clip(target.y - 0.020, -0.70, 0.70)
        data.ctrl[actuator_ids["z_act"]] = np.clip(target.z - 0.390, -0.16, 0.68)
        data.ctrl[actuator_ids["yaw_act"]] = target.yaw
        data.ctrl[actuator_ids["pitch_act"]] = target.pitch
        data.ctrl[actuator_ids["door_act"]] = target.door

        mujoco.mj_step(model, data)
        current_force = _contact_force_norm(mujoco, model, data, tracked_geoms)
        peak_force = max(peak_force, current_force)

        if step % 10 == 0:
            tip = np.array(data.site_xpos[site_tip], dtype=float)
            trace.append(
                {
                    "t": round(float(data.time), 4),
                    "stage": planner.stage,
                    "tip": [round(float(v), 5) for v in tip],
                    "port": [round(float(v), 5) for v in port_world],
                    "ctrl": [round(float(v), 5) for v in data.ctrl[:6]],
                    "contact_force_n": round(current_force, 4),
                    "door_angle_rad": round(float(data.qpos[model.jnt_qposadr[door_joint]]), 4),
                    "success": planner.success,
                }
            )

        if planner.stage == "success" or (planner.stage == "failed" and planner.stage_time > 0.25):
            break

    final_tip = np.array(data.site_xpos[site_tip], dtype=float)
    lateral = float(np.linalg.norm(final_tip[1:3] - port_world[1:3]))
    depth = max(0.0, float(port_world[0] - final_tip[0]))
    result = EpisodeResult(
        episode=episode,
        seed=seed,
        success=planner.success,
        failed_reason=planner.failed_reason,
        lateral_error_m=lateral,
        depth_error_m=depth,
        peak_contact_force_n=peak_force,
        completion_time_s=float(data.time),
        retry_count=planner.retry_count,
        port_y=rand.port_y,
        port_z=rand.port_z,
        port_yaw=rand.port_yaw,
        cable_tension=rand.cable_tension,
    )

    out_dir.mkdir(parents=True, exist_ok=True)
    dataset_path = out_dir / f"episode_{episode:03d}.jsonl"
    with dataset_path.open("w", encoding="utf-8") as fh:
        for row in trace:
            fh.write(json.dumps(row, ensure_ascii=False) + "\n")

    return result, trace


def _tracked_contact_geoms(mujoco, model) -> tuple[set[int], set[int]]:
    plug = {
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "plug_body"),
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "plug_tip"),
    }
    environment = {
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "charge_socket"),
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "socket_funnel"),
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ev_side_panel"),
    }
    return plug, environment


def _contact_force_norm(mujoco, model, data, tracked_geoms: tuple[set[int], set[int]]) -> float:
    total = 0.0
    force = np.zeros(6, dtype=float)
    plug, environment = tracked_geoms
    for idx in range(data.ncon):
        contact = data.contact[idx]
        pair = {int(contact.geom1), int(contact.geom2)}
        if not (pair & plug and pair & environment):
            continue
        mujoco.mj_contactForce(model, data, idx, force)
        total += float(np.linalg.norm(force[:3]))
    if total == 0.0:
        return 0.0
    return min(total, 120.0)


def summarize(results: list[EpisodeResult]) -> dict:
    successes = [r for r in results if r.success]
    denom = max(1, len(results))
    return {
        "project": "ChargeMate: Closed-Loop EV Charging Benchmark",
        "episodes": len(results),
        "success_rate": round(len(successes) / denom, 4),
        "mean_lateral_error_m": round(float(np.mean([r.lateral_error_m for r in results])), 5),
        "mean_depth_error_m": round(float(np.mean([r.depth_error_m for r in results])), 5),
        "mean_peak_contact_force_n": round(float(np.mean([r.peak_contact_force_n for r in results])), 3),
        "force_clip_n": 120.0,
        "mean_completion_time_s": round(float(np.mean([r.completion_time_s for r in results])), 3),
        "retry_rate": round(float(np.mean([1 if r.retry_count else 0 for r in results])), 4),
        "successes": len(successes),
        "failures": [asdict(r) for r in results if not r.success],
        "generated_at_unix": int(time.time()),
    }


def main(default_collect: bool = False) -> None:
    parser = argparse.ArgumentParser(description="Run the ChargeMate EV charging MuJoCo benchmark.")
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--randomize", action="store_true")
    parser.add_argument("--out", type=Path, default=Path("outputs"))
    parser.add_argument("--collect", action="store_true", default=default_collect)
    args = parser.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    write_scene_xml(args.out / "chargemate_scene.xml", make_randomization(args.seed, args.randomize))

    results: list[EpisodeResult] = []
    traces: list[list[dict]] = []
    for ep in range(args.episodes):
        result, trace = run_episode(ep, args.seed + ep, args.out, args.randomize)
        results.append(result)
        traces.append(trace)
        status = "SUCCESS" if result.success else "FAILED"
        print(
            f"[{status}] episode={ep:03d} seed={result.seed} "
            f"lat={result.lateral_error_m:.4f}m depth={result.depth_error_m:.4f}m "
            f"peak_force={result.peak_contact_force_n:.1f}N retries={result.retry_count}"
        )

    metrics = summarize(results)
    (args.out / "metrics.json").write_text(json.dumps(metrics, indent=2, ensure_ascii=False), encoding="utf-8")
    (args.out / "results.json").write_text(json.dumps([asdict(r) for r in results], indent=2, ensure_ascii=False), encoding="utf-8")
    write_trace_html(args.out / "chargemate_trace.html", traces[0], metrics)

    print(json.dumps(metrics, indent=2, ensure_ascii=False))
