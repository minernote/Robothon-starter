from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class SceneRandomization:
    port_y: float
    port_z: float
    port_yaw: float
    cable_tension: float


def build_scene_xml(rand: SceneRandomization) -> str:
    """生成不依赖外部资产的 MuJoCo 场景。"""
    return f"""<mujoco model="chargemate_ev_charging">
  <compiler angle="radian" coordinate="local" autolimits="true"/>
  <option timestep="0.004" gravity="0 0 -9.81" integrator="RK4"/>

  <default>
    <geom solref="0.008 1" solimp="0.92 0.98 0.002" friction="1.0 0.08 0.02"/>
    <joint damping="8" armature="0.03"/>
    <position kp="650" kv="45" forcerange="-450 450"/>
  </default>

  <asset>
    <material name="floor" rgba="0.10 0.11 0.12 1"/>
    <material name="robot" rgba="0.18 0.50 0.82 1"/>
    <material name="ev" rgba="0.75 0.78 0.82 1"/>
    <material name="port" rgba="0.04 0.05 0.06 1"/>
    <material name="plug" rgba="0.03 0.03 0.035 1"/>
    <material name="door" rgba="0.22 0.26 0.31 1"/>
    <material name="target" rgba="0.15 0.8 0.35 0.35"/>
  </asset>

  <worldbody>
    <light name="key" pos="-2.2 -3.0 4.0" dir="1 1 -2" diffuse="0.8 0.8 0.8"/>
    <geom name="floor" type="plane" pos="0 0 0" size="4 4 0.05" material="floor"/>

    <body name="ev_body" pos="1.05 0 0.54">
      <geom name="ev_side_panel" type="box" size="0.08 0.78 0.42" material="ev" contype="1" conaffinity="1"/>
      <geom name="charge_socket" type="cylinder" pos="-0.091 {rand.port_y:.5f} {rand.port_z:.5f}" euler="0 1.5708 {rand.port_yaw:.5f}" size="0.054 0.018" material="port" contype="1" conaffinity="1"/>
      <geom name="socket_funnel" type="cylinder" pos="-0.118 {rand.port_y:.5f} {rand.port_z:.5f}" euler="0 1.5708 {rand.port_yaw:.5f}" size="0.085 0.010" material="target" contype="1" conaffinity="1"/>
      <site name="port_center" pos="-0.132 {rand.port_y:.5f} {rand.port_z:.5f}" euler="0 1.5708 {rand.port_yaw:.5f}" size="0.012" rgba="0.1 1 0.1 1"/>
      <body name="charge_door" pos="-0.125 {rand.port_y - 0.095:.5f} {rand.port_z:.5f}">
        <joint name="door_hinge" type="hinge" axis="0 0 1" range="0 1.35" damping="1.8"/>
        <geom name="charge_door_panel" type="box" pos="0 0.055 0" size="0.010 0.075 0.095" material="door" contype="0" conaffinity="0"/>
      </body>
    </body>

    <body name="cable_anchor" pos="-0.70 -0.70 0.42">
      <geom name="charger_pedestal" type="box" size="0.10 0.12 0.42" rgba="0.16 0.16 0.18 1"/>
      <geom name="cable_hint" type="capsule" fromto="0 0 0.20 0.85 0.52 0.38" size="0.018" rgba="0.02 0.02 0.02 1" contype="0" conaffinity="0"/>
    </body>

    <body name="robot_base" pos="-0.55 0 0.25">
      <geom name="base" type="box" size="0.18 0.18 0.06" rgba="0.12 0.13 0.15 1" contype="0" conaffinity="0"/>
      <body name="slide_x" pos="0 0 0.16">
        <joint name="x_slide" type="slide" axis="1 0 0" range="-0.05 1.35"/>
        <geom name="x_carriage" type="box" size="0.07 0.09 0.055" material="robot" contype="0" conaffinity="0"/>
        <body name="slide_y">
          <joint name="y_slide" type="slide" axis="0 1 0" range="-0.70 0.70"/>
          <geom name="y_carriage" type="box" size="0.055 0.07 0.050" material="robot" contype="0" conaffinity="0"/>
          <body name="slide_z">
            <joint name="z_slide" type="slide" axis="0 0 1" range="-0.16 0.68"/>
            <geom name="z_carriage" type="box" size="0.045 0.045 0.07" material="robot" contype="0" conaffinity="0"/>
            <body name="wrist_yaw">
              <joint name="yaw" type="hinge" axis="0 0 1" range="-0.55 0.55"/>
              <geom name="yaw_hub" type="sphere" size="0.055" material="robot" contype="0" conaffinity="0"/>
              <body name="wrist_pitch">
                <joint name="pitch" type="hinge" axis="0 1 0" range="-0.35 0.35"/>
                <geom name="pitch_hub" type="sphere" size="0.043" material="robot" contype="0" conaffinity="0"/>
                <body name="plug" pos="0.11 0 0">
                  <geom name="plug_body" type="capsule" fromto="-0.10 0 0 0.10 0 0" size="0.030" material="plug" contype="1" conaffinity="1"/>
                  <geom name="plug_tip" type="cylinder" pos="0.125 0 0" euler="0 1.5708 0" size="0.036 0.026" material="plug" contype="1" conaffinity="1"/>
                  <site name="plug_tip_site" pos="0.158 0 0" size="0.010" rgba="1 0.8 0.1 1"/>
                  <site name="plug_grip_site" pos="-0.045 0 0" size="0.010" rgba="0.2 0.6 1 1"/>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <position name="x_act" joint="x_slide"/>
    <position name="y_act" joint="y_slide"/>
    <position name="z_act" joint="z_slide"/>
    <position name="yaw_act" joint="yaw" kp="280" kv="30"/>
    <position name="pitch_act" joint="pitch" kp="280" kv="30"/>
    <position name="door_act" joint="door_hinge" kp="180" kv="12" forcerange="-40 40"/>
  </actuator>

  <sensor>
    <jointpos name="door_angle" joint="door_hinge"/>
    <jointpos name="x_pos" joint="x_slide"/>
    <jointpos name="y_pos" joint="y_slide"/>
    <jointpos name="z_pos" joint="z_slide"/>
    <framepos name="plug_tip_pos" objtype="site" objname="plug_tip_site"/>
    <framepos name="port_center_pos" objtype="site" objname="port_center"/>
  </sensor>
</mujoco>
"""


def write_scene_xml(path: Path, rand: SceneRandomization) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(build_scene_xml(rand), encoding="utf-8")
