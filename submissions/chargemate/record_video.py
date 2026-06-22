from __future__ import annotations

import argparse
from pathlib import Path

import imageio.v2 as imageio
import numpy as np

from chargemate.benchmark import _contact_force_norm, _import_mujoco, _tracked_contact_geoms, make_randomization
from chargemate.controller import ChargeMatePlanner
from chargemate.scene import build_scene_xml


def record_video(seed: int, randomize: bool, out: Path, width: int = 640, height: int = 480) -> None:
    mujoco = _import_mujoco()
    rand = make_randomization(seed, randomize)
    model = mujoco.MjModel.from_xml_string(build_scene_xml(rand))
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=height, width=width)

    site_tip = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "plug_tip_site")
    site_port = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "port_center")
    door_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "door_hinge")
    actuator_ids = {name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in ["x_act", "y_act", "z_act", "yaw_act", "pitch_act", "door_act"]}
    tracked_geoms = _tracked_contact_geoms(mujoco, model)

    mujoco.mj_forward(model, data)
    port_world = np.array(data.site_xpos[site_port], dtype=float)
    planner = ChargeMatePlanner(port_world=port_world, port_yaw=rand.port_yaw, cable_tension=rand.cable_tension)

    frames = []
    current_force = 0.0
    frame_stride = 6
    for step in range(int(8.0 / model.opt.timestep)):
        tip = np.array(data.site_xpos[site_tip], dtype=float)
        door_angle = float(data.qpos[model.jnt_qposadr[door_joint]])
        target = planner.update(float(model.opt.timestep), tip, door_angle, current_force)
        data.ctrl[actuator_ids["x_act"]] = np.clip(target.x + 0.217, -0.05, 1.35)
        data.ctrl[actuator_ids["y_act"]] = np.clip(target.y - 0.020, -0.70, 0.70)
        data.ctrl[actuator_ids["z_act"]] = np.clip(target.z - 0.390, -0.16, 0.68)
        data.ctrl[actuator_ids["yaw_act"]] = target.yaw
        data.ctrl[actuator_ids["pitch_act"]] = target.pitch
        data.ctrl[actuator_ids["door_act"]] = target.door
        mujoco.mj_step(model, data)
        current_force = _contact_force_norm(mujoco, model, data, tracked_geoms)
        if step % frame_stride == 0:
            renderer.update_scene(data)
            frames.append(renderer.render())
        if planner.stage == "success":
            for _ in range(24):
                renderer.update_scene(data)
                frames.append(renderer.render())
            break

    out.parent.mkdir(parents=True, exist_ok=True)
    imageio.mimsave(out, frames, fps=30)
    print(f"wrote {out} frames={len(frames)}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Record ChargeMate MuJoCo demo video.")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--randomize", action="store_true")
    parser.add_argument("--out", type=Path, default=Path("outputs/chargemate_demo.mp4"))
    args = parser.parse_args()
    record_video(args.seed, args.randomize, args.out)


if __name__ == "__main__":
    main()
