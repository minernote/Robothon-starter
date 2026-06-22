from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ControllerTargets:
    x: float
    y: float
    z: float
    yaw: float
    pitch: float
    door: float


class ChargeMatePlanner:
    """状态机控制器：用闭环误差推进任务阶段。"""

    stages = (
        "stow",
        "door_reach",
        "door_open",
        "pre_align",
        "fine_align",
        "insert",
        "verify",
        "recover",
        "success",
        "failed",
    )

    def __init__(self, port_world: np.ndarray, port_yaw: float, cable_tension: float):
        self.port_world = port_world
        self.port_yaw = port_yaw
        self.cable_tension = cable_tension
        self.stage = "stow"
        self.stage_time = 0.0
        self.retry_count = 0
        self.max_retries = 2
        self.success = False
        self.failed_reason = ""

    def reset_stage(self, stage: str) -> None:
        self.stage = stage
        self.stage_time = 0.0

    def update(self, dt: float, tip: np.ndarray, door_angle: float, peak_force: float) -> ControllerTargets:
        self.stage_time += dt
        target = self._target_for_stage()

        lateral = float(np.linalg.norm(tip[1:3] - self.port_world[1:3]))
        depth = float(self.port_world[0] - tip[0])

        if self.stage == "stow" and self.stage_time > 0.35:
            self.reset_stage("door_reach")
        elif self.stage == "door_reach" and self._near(tip, np.array([self.port_world[0] - 0.24, self.port_world[1] - 0.11, self.port_world[2]]), 0.070):
            self.reset_stage("door_open")
        elif self.stage == "door_open" and door_angle > 1.05:
            self.reset_stage("pre_align")
        elif self.stage == "pre_align" and (self.stage_time > 0.65 or self._near(tip, np.array([self.port_world[0] - 0.22, self.port_world[1], self.port_world[2] + 0.012]), 0.080)):
            self.reset_stage("fine_align")
        elif self.stage == "fine_align" and (lateral < 0.012 or self.stage_time > 0.80) and abs(target.yaw - self.port_yaw) < 0.035:
            self.reset_stage("insert")
        elif self.stage == "insert":
            if self.stage_time > 0.10 and peak_force > 115 and self.retry_count < self.max_retries:
                self.retry_count += 1
                self.reset_stage("recover")
            elif lateral < 0.018 and depth < 0.026:
                self.reset_stage("verify")
        elif self.stage == "recover" and self.stage_time > 0.65:
            self.reset_stage("pre_align")
        elif self.stage == "verify":
            if lateral < 0.018 and depth < 0.026:
                self.success = True
                self.reset_stage("success")
            elif self.stage_time > 0.50:
                self.failed_reason = "verification_failed"
                self.reset_stage("failed")

        if self.stage_time > 3.5 and self.stage not in {"success", "failed"}:
            self.failed_reason = f"timeout_{self.stage}"
            self.reset_stage("failed")

        return self._target_for_stage()

    def _target_for_stage(self) -> ControllerTargets:
        px, py, pz = self.port_world
        cable_bias = 0.020 * self.cable_tension
        z_comp = 0.0
        if self.stage == "stow":
            return ControllerTargets(0.18, -0.38, 0.36, 0.0, 0.0, 0.0)
        if self.stage == "door_reach":
            return ControllerTargets(px - 0.24, py - 0.11, pz, -0.08, 0.0, 0.0)
        if self.stage == "door_open":
            return ControllerTargets(px - 0.24, py - 0.11, pz, -0.08, 0.0, 1.25)
        if self.stage == "pre_align":
            return ControllerTargets(px - 0.22, py + cable_bias, pz + 0.012 + z_comp, self.port_yaw * 0.55, 0.0, 1.25)
        if self.stage == "fine_align":
            return ControllerTargets(px - 0.080, py, pz + z_comp, self.port_yaw, 0.0, 1.25)
        if self.stage == "insert":
            return ControllerTargets(px - 0.006, py, pz + z_comp, self.port_yaw, 0.0, 1.25)
        if self.stage == "recover":
            return ControllerTargets(px - 0.22, py - 0.025 * np.sign(self.port_yaw or 1.0), pz + 0.018, self.port_yaw * 0.4, 0.0, 1.25)
        if self.stage in {"verify", "success"}:
            return ControllerTargets(px - 0.012, py, pz + z_comp, self.port_yaw, 0.0, 1.25)
        return ControllerTargets(px - 0.25, py, pz + 0.04, 0.0, 0.0, 1.25)

    @staticmethod
    def _near(a: np.ndarray, b: np.ndarray, tol: float) -> bool:
        return float(np.linalg.norm(a - b)) < tol
