import os

from vuer_mjcf.schema.schema import MocapBody, Mjcf
from vuer_mjcf.schema.base import Xml
from utils.transform import compose
# from vuer_mjcf.basic_components.rigs.camera_rig import make_camera_rig
# from vuer_mjcf.basic_components.rigs.lighting_rig import make_lighting_rig
# from vuer_mjcf.se3.se3_mujoco import Vector3, WXYZ

class AstriBot(MocapBody):
    """
    This is Astribot
    """

    assets: str = "astribot"
    end_effector: Xml = None
    _attributes = {
        "name": "astribot",
        "childclass": "astribot",
    }

    def __init__(self, *_children, end_effector: Xml = None, mocap_pos=None, **kwargs):
        super().__init__(*_children, **kwargs)
        self.end_effector = end_effector
        if end_effector:
            self._children = self._children + (end_effector,)

        # 0.5114, 0.37265
        # [0.36575, 0, 0.36065]
        if mocap_pos is None:
            self.mocap_pos = self._pos + [0.455, 0, 0.36065]
        else:
            self.mocap_pos = mocap_pos

        # self._children = self._children + (self.mocap_pos,)
        self.mocap_pos_left = compose(self._pos, self._quat, [0.0,  0.33,  0.8])
        self.mocap_pos_right = compose(self._pos, self._quat, [0.0, -0.33,  0.8])
        self.mocap_pos_head = compose(self._pos, self._quat, [0.0, 0.0, 1.55])

        values = self._format_dict()
        self._mocaps = self._mocaps_body.format(**values)


    _mocaps_body = """
    <body mocap="true" name="{name}-mocap-1" pos="{mocap_pos_left}">
      <site name="{name}-mocap-site-1" size=".11" type="sphere" rgba="0.13 0.3 0.2 1"/>
    </body>
    <body mocap="true" name="{name}-mocap-2" pos="{mocap_pos_right}">
      <site name="{name}-mocap-site-2" size=".11" type="sphere" rgba="0.13 0.3 0.2 1"/>
    </body>
    <body mocap="true" name="{name}-mocap-3" pos="{mocap_pos_head}">
      <site name="{name}-mocap-site-3" size=".11" type="sphere" rgba="0.13 0.3 0.2 1"/>
    </body>
    """

    _mocaps_equality = """
    <equality>
        <weld body1="{name}-astribot_gripper_left_base" body2="{name}-mocap-1"/>
        <weld body1="{name}-astribot_gripper_right_base" body2="{name}-mocap-2"/>
        <weld body1="{name}-astribot_head_link_2" body2="{name}-mocap-3"/>
    </equality>
    """

    _preamble = """
      <default>
        <default class="{childclass}-robot">
          <default class="{childclass}-motor">
            <joint />
            <motor />
            <position kp="50" inheritrange="1" dampratio="0.95" forcerange="-35 35"/>
          </default>
          <default class="{childclass}-visual">
            <geom material="{name}-visualgeom" contype="0" conaffinity="0" group="2" />
          </default>
          <default class="{childclass}-collision">
            <geom material="{name}-collision_material" condim="3" contype="0" conaffinity="1" priority="1" group="1" solref="0.005 1" friction="1 0.01 0.01" />
            <equality solimp="0.99 0.999 1e-05" solref="0.005 1" />
          </default>
        </default>
      </default>

      <compiler angle="radian" />

      <asset>
        <material name="{name}-" rgba="0.898039215686275 0.917647058823529 0.929411764705882 1" />
        <material name="{name}-default_material" rgba="0.7 0.7 0.7 1" />
        <material name="{name}-collision_material" rgba="1.0 0.28 0.1 0.9" />
        <mesh name="{name}-astribot_torso_base_link.STL" file="{assets}/astribot_torso_base_link.STL" />
        <mesh name="{name}-astribot_torso_link_1.STL" file="{assets}/astribot_torso_link_1.STL" />
        <mesh name="{name}-astribot_torso_link_2.STL" file="{assets}/astribot_torso_link_2.STL" />
        <mesh name="{name}-astribot_torso_link_3.STL" file="{assets}/astribot_torso_link_3.STL" />
        <mesh name="{name}-astribot_torso_link_4.STL" file="{assets}/astribot_torso_link_4.STL" />
        <mesh name="{name}-astribot_head_base_link.STL" file="{assets}/astribot_head_base_link.STL" />
        <mesh name="{name}-astribot_head_link_1.STL" file="{assets}/astribot_head_link_1.STL" />
        <mesh name="{name}-astribot_head_link_2.STL" file="{assets}/astribot_head_link_2.STL" />
        <mesh name="{name}-astribot_arm_left_base_link.STL" file="{assets}/astribot_arm_left_base_link.STL" />
        <mesh name="{name}-astribot_arm_link_1.STL" file="{assets}/astribot_arm_link_1.STL" />
        <mesh name="{name}-astribot_arm_left_link_2.STL" file="{assets}/astribot_arm_left_link_2.STL" />
        <mesh name="{name}-astribot_arm_link_3.STL" file="{assets}/astribot_arm_link_3.STL" />
        <mesh name="{name}-astribot_arm_link_4.STL" file="{assets}/astribot_arm_link_4.STL" />
        <mesh name="{name}-astribot_arm_link_5.STL" file="{assets}/astribot_arm_link_5.STL" />
        <mesh name="{name}-astribot_arm_link_6.STL" file="{assets}/astribot_arm_link_6.STL" />
        <mesh name="{name}-astribot_arm_link_7.STL" file="{assets}/astribot_arm_link_7.STL" />
        <mesh name="{name}-astribot_gripper_base_link.STL" file="{assets}/astribot_gripper_base_link.STL" />
        <mesh name="{name}-astribot_gripper_L1_Link.STL" file="{assets}/astribot_gripper_L1_Link.STL" />
        <mesh name="{name}-astribot_gripper_L11_Link.STL" file="{assets}/astribot_gripper_L11_Link.STL" />
        <mesh name="{name}-astribot_gripper_L2_Link.STL" file="{assets}/astribot_gripper_L2_Link.STL" />
        <mesh name="{name}-astribot_gripper_R1_Link.STL" file="{assets}/astribot_gripper_R1_Link.STL" />
        <mesh name="{name}-astribot_gripper_R11_Link.STL" file="{assets}/astribot_gripper_R11_Link.STL" />
        <mesh name="{name}-astribot_gripper_R2_Link.STL" file="{assets}/astribot_gripper_R2_Link.STL" />
        <mesh name="{name}-astribot_arm_right_base_link.STL" file="{assets}/astribot_arm_right_base_link.STL" />
        <mesh name="{name}-astribot_arm_right_link_2.STL" file="{assets}/astribot_arm_right_link_2.STL" />
      </asset>
    """

    template = """
        <body name="{name}-astribot_torso_base" pos="0.00000000 0.00000000 0.10000000" quat="1 0 0 0" childclass="{childclass}-robot">
          <joint name="{name}-floating_base" />
          <geom name="{name}-astribot_torso_base_collision" pos="0 0 -0.05" quat="1.0 0.0 0.0 0.0" type="cylinder" size="0.3 0.05" class="{childclass}-collision" />
          <geom name="{name}-astribot_torso_base_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-astribot_torso_base_link.STL" class="{childclass}-visual" />
          <body name="{name}-astribot_torso_link_1" pos="0 0 0.12" quat="0.4999981633974483 -0.49999999999662686 0.49999999999662686 0.5000018366025516">
            <joint name="{name}-astribot_torso_joint_1" type="hinge" ref="0.0" class="{childclass}-motor" range="-0.04 1.3" axis="0 0 1" />
            <inertial pos="-0.15249 0.00021066 -0.0033476" quat="1.0 0.0 0.0 0.0" mass="7.3766" diaginertia="0.028834 0.1483 0.14268" />
            <geom name="{name}-astribot_torso_link_1_collision" pos="-0.15 0 0" quat="0.7073882691671998 0.0 0.706825181105366 0.0" type="cylinder" size="0.1 0.15" class="{childclass}-collision" />
            <geom name="{name}-astribot_torso_link_1_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-astribot_torso_link_1.STL" class="{childclass}-visual" />
            <body name="{name}-astribot_torso_link_2" pos="-0.38 0 0" quat="1.0 0.0 0.0 0.0">
              <joint name="{name}-astribot_torso_joint_2" type="hinge" ref="0.0" class="{childclass}-motor" range="-2.3 0.06" axis="0 0 1" />
              <inertial pos="-0.22214 0.028873 0.00059024" quat="1.0 0.0 0.0 0.0" mass="5.6183" diaginertia="0.0082545 0.012677 0.017512" />
              <geom name="{name}-astribot_torso_link_2_collision" pos="-0.15 0.03 0" quat="0.7018689904930883 0.08812324100427678 0.7013102958316921 -0.08819344385825831" type="cylinder" size="0.1 0.15" class="{childclass}-collision" />
              <geom name="{name}-astribot_torso_link_2_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-astribot_torso_link_2.STL" class="{childclass}-visual" />
              <body name="{name}-astribot_torso_link_3" pos="-0.390 0 0" quat="1.0 0.0 0.0 0.0">
                <joint name="{name}-astribot_torso_joint_3" type="hinge" ref="0.0" class="{childclass}-motor" range="-0.4 2.3" axis="0 0 1" />
                <inertial pos="-0.017519 0.00059523 0.00025821" quat="1.0 0.0 0.0 0.0" mass="0.49013" diaginertia="0.00022212 0.00036522 0.00038039" />
                <geom name="{name}-astribot_torso_link_3_collision" pos="-0.18 0.01 0" quat="0.7073882691671998 0.0 0.706825181105366 0.0" type="cylinder" size="0.09 0.175" class="{childclass}-collision" />
                <geom name="{name}-astribot_torso_link_3_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-astribot_torso_link_3.STL" class="{childclass}-visual" />
                <body name="{name}-astribot_torso_link_4" pos="0 0 0" quat="0.4999981633974483 0.49999999999662686 -0.5000018366025516 -0.49999999999662686">
                  <joint name="{name}-astribot_torso_joint_4" type="hinge" ref="0.0" class="{childclass}-motor" range="-1.2 1.2" axis="0 0 1" />
                  <inertial pos="0.00476227191516749 -4.56966810241128E-05 0.242912573510671" quat="1.0 0.0 0.0 0.0" mass="3.55695635704638" diaginertia="0.0165768322563514 0.014904355097835 0.0110629318711333" />
                  <geom name="{name}-astribot_torso_link_4_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-astribot_torso_link_4.STL" class="{childclass}-visual" />
                  <body name="{name}-astribot_torso_end_effector" pos="0 0 0.34475" quat="1.0 0.0 0.0 0.0">
                    <inertial pos="0 0 0" quat="1.0 0.0 0.0 0.0" mass="0" diaginertia="0.0 0.0 0.0" />
                    <body name="{name}-astribot_head_base_link" pos="0 0 0.1229" quat="1.0 0.0 0.0 0.0">
                      <inertial pos="-0.02816247 -0.00023176 -0.05211965" quat="1.0 0.0 0.0 0.0" mass="0.39541440" diaginertia="0.00036025 0.00049377 0.00074441" />
                      <geom name="{name}-astribot_head_base_link_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-astribot_head_base_link.STL" class="{childclass}-visual" />
                      <body name="{name}-astribot_head_link_1" pos="0 0 0" quat="1.0 0.0 0.0 0.0">
                        <joint name="{name}-astribot_head_joint_1" type="hinge" ref="0.0" class="{childclass}-motor" range="-1.57 1.57" axis="0 0 1" />
                        <inertial pos="0.00003191 -0.00038566 -0.02579910" quat="1.0 0.0 0.0 0.0" mass="0.13910581" diaginertia="4.567e-05 4.789e-05 3.779e-05" />
                        <geom name="{name}-astribot_head_link_1_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-astribot_head_link_1.STL" class="{childclass}-visual" />
                        <body name="{name}-astribot_head_link_2" pos="0 0 0" quat="0.7071054825112363 -0.7071080798594735 0.0 0.0">
                          <joint name="{name}-astribot_head_joint_2" type="hinge" ref="0.0" class="{childclass}-motor" range="-1.22 1.22" axis="0 0 1" />
                          <inertial pos="0.00705824 -0.14472601 -0.00061724" quat="1.0 0.0 0.0 0.0" mass="0.97012447" diaginertia="0.0055566 0.00151908 0.00544493" />
                          <geom name="{name}-astribot_head_link_2_collision" pos="0 -0.15 0" quat="1.0 0.0 0.0 0.0" type="sphere" size="0.11" class="{childclass}-collision" />
                          <geom name="{name}-astribot_head_link_2_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-astribot_head_link_2.STL" class="{childclass}-visual" />
                        </body>
                      </body>
                    </body>
                    <body name="{name}-astribot_arm_left_base_link" pos="0 0.06449 0.02348" quat="0.5792280697080312 -0.40557964426794313 -0.4055796551354079 -0.5792280541876752">
                      <inertial pos="-0.04309067 -0.01849796 -0.01384838" quat="1.0 0.0 0.0 0.0" mass="2.22887320" diaginertia="0.00386389 0.00306042 0.00570294" />
                      <geom name="{name}-astribot_arm_left_base_link_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-astribot_arm_left_base_link.STL" class="{childclass}-visual" />
                      <body name="{name}-astribot_arm_left_link_1" pos="0 0 0.091" quat="1.0 0.0 0.0 0.0">
                        <joint name="{name}-astribot_arm_left_joint_1" type="hinge" ref="0.0" class="{childclass}-motor" range="-3.1 3.1" axis="0 0 1" />
                        <inertial pos="0.00012026 0.00047499 -0.05682561" quat="1.0 0.0 0.0 0.0" mass="0.59936716" diaginertia="0.00064791 0.00065264 0.00050137" />
                        <geom name="{name}-astribot_arm_left_link_1_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-astribot_arm_link_1.STL" class="{childclass}-visual" />
                        <body name="{name}-astribot_arm_left_link_2" pos="0 0 0" quat="0.7071054825112363 0.7071080798594735 0.0 0.0">
                          <joint name="{name}-astribot_arm_left_joint_2" type="hinge" ref="0.0" class="{childclass}-motor" range="-1.53 0.46" axis="0 0 1" />
                          <inertial pos="-0.00396228 0.03490708 0.04270156" quat="1.0 0.0 0.0 0.0" mass="1.43694657" diaginertia="0.00360751 0.00283747 0.00172927" />
                          <geom name="{name}-astribot_arm_left_link_2_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-astribot_arm_left_link_2.STL" class="{childclass}-visual" />
                          <body name="{name}-astribot_arm_left_link_3" pos="0 0.05 0" quat="0.4999981633974483 0.49999999999662686 -0.5000018366025516 -0.49999999999662686">
                            <joint name="{name}-astribot_arm_left_joint_3" type="hinge" ref="0.0" class="{childclass}-motor" range="-3.1 3.1" axis="0 0 1" />
                            <inertial pos="0.00531263 0.00020707 0.16136713" quat="1.0 0.0 0.0 0.0" mass="2.0926399" diaginertia="0.01752179 0.01784671 0.0021843" />
                            <geom name="{name}-astribot_arm_left_link_3_collision" pos="0 0 0.15" quat="1.0 0.0 0.0 0.0" type="cylinder" size="0.05 0.09" class="{childclass}-collision" />
                            <geom name="{name}-astribot_arm_left_link_3_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-astribot_arm_link_3.STL" class="{childclass}-visual" />
                            <body name="{name}-astribot_arm_left_link_4" pos="0.03 0 0.309" quat="0.7071054825112363 -0.7071080798594735 0.0 0.0">
                              <joint name="{name}-astribot_arm_left_joint_4" type="hinge" ref="0.0" class="{childclass}-motor" range="-0.06 2.61" axis="0 0 1" />
                              <inertial pos="0.00017294 -0.01269315 0.00196140" quat="1.0 0.0 0.0 0.0" mass="0.36587933" diaginertia="0.00047693 0.00036801 0.00050082" />
                              <geom name="{name}-astribot_arm_left_link_4_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="sphere" size="0.06" class="{childclass}-collision" />
                              <geom name="{name}-astribot_arm_left_link_4_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-astribot_arm_link_4.STL" class="{childclass}-visual" />
                              <body name="{name}-astribot_arm_left_link_5" pos="0 0 0" quat="0.7071054825112363 0.7071080798594735 0.0 0.0">
                                <joint name="{name}-astribot_arm_left_joint_5" type="hinge" ref="0.0" class="{childclass}-motor" range="-2.56 2.56" axis="0 0 1" />
                                <inertial pos="-0.00032037 -0.00015185 0.15370447" quat="1.0 0.0 0.0 0.0" mass="1.24085064" diaginertia="0.00454092 0.00438922 0.00079442" />
                                <geom name="{name}-astribot_arm_left_link_5_collision" pos="0 0 0.14" quat="1.0 0.0 0.0 0.0" type="cylinder" size="0.05 0.07" class="{childclass}-collision" />
                                <geom name="{name}-astribot_arm_left_link_5_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-astribot_arm_link_5.STL" class="{childclass}-visual" />
                                <body name="{name}-astribot_arm_left_link_6" pos="0 0 0.277" quat="0.7071054825112363 -0.7071080798594735 0.0 0.0">
                                  <joint name="{name}-astribot_arm_left_joint_6" type="hinge" ref="0.0" class="{childclass}-motor" range="-0.76 0.76" axis="0 0 1" />
                                  <inertial pos="-0.00010311 0.00001218 0.00013181" quat="1.0 0.0 0.0 0.0" mass="0.22465307" diaginertia="0.00016474 0.00015608 8.555e-05" />
                                  <geom name="{name}-astribot_arm_left_link_6_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-astribot_arm_link_6.STL" class="{childclass}-visual" />
                                  <body name="{name}-astribot_arm_left_link_7" pos="0 0 0" quat="0.7071054825112363 0.0 0.7071080798594735 0.0">
                                    <joint name="{name}-astribot_arm_left_joint_7" type="hinge" ref="0.0" class="{childclass}-motor" range="-1.53 1.53" axis="0 0 1" />
                                    <inertial pos="4.333e-05 -0.00723447 -0.00362428" quat="1.0 0.0 0.0 0.0" mass="0.12772019" diaginertia="0.00013672 0.00010783 5.858e-05" />
                                    <geom name="{name}-astribot_arm_left_link_7_collision" pos="0.006 0 0" quat="1.0 0.0 0.0 0.0" type="sphere" size="0.05" class="{childclass}-collision" />
                                    <geom name="{name}-astribot_arm_left_link_7_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-astribot_arm_link_7.STL" class="{childclass}-visual" />
                                    <body name="{name}-astribot_gripper_left_base" pos="0 -0.045 0" quat="0.7071054825112363 0.7071080798594735 0.0 0.0">
                                      <inertial pos="3.63816834670974E-05 -7.64937801344797E-05 0.0339715454089654" quat="1.0 0.0 0.0 0.0" mass="0.193542076585365" diaginertia="8.74128051113331e-05 0.000114384919729933 0.000110045585286976" />
                                      <geom name="{name}-astribot_gripper_left_base_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-astribot_gripper_base_link.STL" class="{childclass}-visual" />
                                      <body name="{name}-astribot_gripper_left_Link_L1" pos="0.013001 0 0.0537" quat="1.0 0.0 0.0 0.0">
                                        <joint name="{name}-astribot_gripper_left_joint_L1" type="hinge" ref="0.0" class="{childclass}-motor" range="-1.57 1.57" axis="0 1 0" />
                                        <inertial pos="0.0202603407008252 3.25769539416252E-07 0.0195138016274243" quat="1.0 0.0 0.0 0.0" mass="0.0126193026163599" diaginertia="4.00495011266377e-06 2.98446364943016e-06 4.36665411768987e-06" />
                                        <geom name="{name}-astribot_gripper_left_Link_L1_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-default_material" type="mesh" mesh="{name}-astribot_gripper_L1_Link.STL" class="{childclass}-visual" />
                                        <body name="{name}-astribot_gripper_left_Link_L11" pos="0.038 0 0.033" quat="1.0 0.0 0.0 0.0">
                                          <joint name="{name}-astribot_gripper_left_joint_L11" type="hinge" ref="0.0" class="{childclass}-motor" range="-1.57 1.57" axis="0 1 0" />
                                          <inertial pos="0.0078499983581578 2.30865069672272E-06 0.0261255438486819" quat="1.0 0.0 0.0 0.0" mass="0.0422679242486426" diaginertia="7.52065705546739e-06 6.23981721728218e-06 3.81260098942712e-06" />
                                          <geom name="{name}-astribot_gripper_left_Link_L11_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-astribot_gripper_L11_Link.STL" class="{childclass}-collision" />
                                          <geom name="{name}-astribot_gripper_left_Link_L11_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-default_material" type="mesh" mesh="{name}-astribot_gripper_L11_Link.STL" class="{childclass}-visual" />
                                        </body>
                                      </body>
                                      <body name="{name}-astribot_gripper_left_Link_L2" pos="0.033 0 0.0537" quat="1.0 0.0 0.0 0.0">
                                        <joint name="{name}-astribot_gripper_left_joint_L2" type="hinge" ref="0.0" class="{childclass}-motor" range="-1.57 1.57" axis="0 1 0" />
                                        <inertial pos="0.00995690697258714 1.76552698823303E-10 0.0110501818995788" quat="1.0 0.0 0.0 0.0" mass="0.0233645198811733" diaginertia="3.51398687886059e-06 8.26281821632449e-06 7.05331607908947e-06" />
                                        <geom name="{name}-astribot_gripper_left_Link_L2_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-default_material" type="mesh" mesh="{name}-astribot_gripper_L2_Link.STL" class="{childclass}-visual" />
                                      </body>
                                      <body name="{name}-astribot_gripper_left_Link_R1" pos="-0.012999 0 0.0537" quat="1.0 0.0 0.0 0.0">
                                        <joint name="{name}-astribot_gripper_left_joint_R1" type="hinge" ref="0.0" class="{childclass}-motor" range="-1.57 1.57" axis="0 1 0" />
                                        <inertial pos="-0.0202603407007736 -3.25769539812667E-07 0.0195138016274777" quat="1.0 0.0 0.0 0.0" mass="0.0126193026163599" diaginertia="4.00495011267122e-06 2.98446364943016e-06 4.36665411768241e-06" />
                                        <geom name="{name}-astribot_gripper_left_Link_R1_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-default_material" type="mesh" mesh="{name}-astribot_gripper_R1_Link.STL" class="{childclass}-visual" />
                                        <body name="{name}-astribot_gripper_left_Link_R11" pos="-0.038 0 0.033" quat="1.0 0.0 0.0 0.0">
                                          <joint name="{name}-astribot_gripper_left_joint_R11" type="hinge" ref="0.0" class="{childclass}-motor" range="-1.57 1.57" axis="0 1 0" />
                                          <inertial pos="-0.00785031648408113 -2.30865069838854E-06 0.0261252819783987" quat="1.0 0.0 0.0 0.0" mass="0.0422679242485921" diaginertia="7.52064120010972e-06 6.23981721727655e-06 3.81261684477754e-06" />
                                          <geom name="{name}-astribot_gripper_left_Link_R11_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-astribot_gripper_R11_Link.STL" class="{childclass}-collision" />
                                          <geom name="{name}-astribot_gripper_left_Link_R11_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-default_material" type="mesh" mesh="{name}-astribot_gripper_R11_Link.STL" class="{childclass}-visual" />
                                        </body>
                                      </body>
                                      <body name="{name}-astribot_gripper_left_Link_R2" pos="-0.032999 0 0.0537" quat="1.0 0.0 0.0 0.0">
                                        <joint name="{name}-astribot_gripper_left_joint_R2" type="hinge" ref="0.0" class="{childclass}-motor" range="-1.57 1.57" axis="0 1 0" />
                                        <inertial pos="-0.00995690697253525 -1.76552690610476E-10 0.0110501818996256" quat="1.0 0.0 0.0 0.0" mass="0.0233645198811734" diaginertia="3.51398687889016e-06 8.26281821632449e-06 7.05331607905991e-06" />
                                        <geom name="{name}-astribot_gripper_left_Link_R2_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-default_material" type="mesh" mesh="{name}-astribot_gripper_R2_Link.STL" class="{childclass}-visual" />
                                      </body>
                                    </body>
                                  </body>
                                </body>
                              </body>
                            </body>
                          </body>
                        </body>
                      </body>
                    </body>
                    <body name="{name}-astribot_arm_right_base_link" pos="0 -0.06449 0.02348" quat="0.5792280697080312 0.40557964426794313 -0.4055796551354079 0.5792280541876752">
                      <inertial pos="-0.04450932 0.01988693 0.00162589" quat="1.0 0.0 0.0 0.0" mass="2.22887320" diaginertia="0.00396811 0.00304615 0.00574641" />
                      <geom name="{name}-astribot_arm_right_base_link_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-astribot_arm_right_base_link.STL" class="{childclass}-visual" />
                      <body name="{name}-astribot_arm_right_link_1" pos="0 0 0.091" quat="1.0 0.0 0.0 0.0">
                        <joint name="{name}-astribot_arm_right_joint_1" type="hinge" ref="0.0" class="{childclass}-motor" range="-3.1 3.1" axis="0 0 1" />
                        <inertial pos="0.00012026 0.00047499 -0.05682561" quat="1.0 0.0 0.0 0.0" mass="0.59936716" diaginertia="0.00064791 0.00065264 0.00050137" />
                        <geom name="{name}-astribot_arm_right_link_1_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-astribot_arm_link_1.STL" class="{childclass}-visual" />
                        <body name="{name}-astribot_arm_right_link_2" pos="0 0 0" quat="0.7071054825112363 0.7071080798594735 0.0 0.0">
                          <joint name="{name}-astribot_arm_right_joint_2" type="hinge" ref="0.0" class="{childclass}-motor" range="-1.53 0.46" axis="0 0 1" />
                          <inertial pos="-0.00384709 0.03502105 -0.04050705" quat="1.0 0.0 0.0 0.0" mass="1.45811037" diaginertia="0.00394196 0.00316298 0.00177295" />
                          <geom name="{name}-astribot_arm_right_link_2_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-astribot_arm_right_link_2.STL" class="{childclass}-visual" />
                          <body name="{name}-astribot_arm_right_link_3" pos="0 0.05 0" quat="0.4999981633974483 0.49999999999662686 -0.5000018366025516 -0.49999999999662686">
                            <joint name="{name}-astribot_arm_right_joint_3" type="hinge" ref="0.0" class="{childclass}-motor" range="-3.1 3.1" axis="0 0 1" />
                            <inertial pos="0.00531263 0.00020707 0.16136713" quat="1.0 0.0 0.0 0.0" mass="2.0926399" diaginertia="0.01752179 0.01784671 0.0021843" />
                            <geom name="{name}-astribot_arm_right_link_3_collision" pos="0 0 0.15" quat="1.0 0.0 0.0 0.0" type="cylinder" size="0.05 0.09" class="{childclass}-collision" />
                            <geom name="{name}-astribot_arm_right_link_3_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-astribot_arm_link_3.STL" class="{childclass}-visual" />
                            <body name="{name}-astribot_arm_right_link_4" pos="0.03 0 0.309" quat="0.7071054825112363 -0.7071080798594735 0.0 0.0">
                              <joint name="{name}-astribot_arm_right_joint_4" type="hinge" ref="0.0" class="{childclass}-motor" range="-0.06 2.61" axis="0 0 1" />
                              <inertial pos="0.00017294 -0.01269315 0.00196140" quat="1.0 0.0 0.0 0.0" mass="0.36587933" diaginertia="0.00047693 0.00036801 0.00050082" />
                              <geom name="{name}-astribot_arm_right_link_4_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="sphere" size="0.06" class="{childclass}-collision" />
                              <geom name="{name}-astribot_arm_right_link_4_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-astribot_arm_link_4.STL" class="{childclass}-visual" />
                              <body name="{name}-astribot_arm_right_link_5" pos="0 0 0" quat="0.7071054825112363 0.7071080798594735 0.0 0.0">
                                <joint name="{name}-astribot_arm_right_joint_5" type="hinge" ref="0.0" class="{childclass}-motor" range="-2.56 2.56" axis="0 0 1" />
                                <inertial pos="-0.00032037 -0.00015185 0.15370447" quat="1.0 0.0 0.0 0.0" mass="1.24085064" diaginertia="0.00454092 0.00438922 0.00079442" />
                                <geom name="{name}-astribot_arm_right_link_5_collision" pos="0 0 0.14" quat="1.0 0.0 0.0 0.0" type="cylinder" size="0.05 0.07" class="{childclass}-collision" />
                                <geom name="{name}-astribot_arm_right_link_5_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-astribot_arm_link_5.STL" class="{childclass}-visual" />
                                <body name="{name}-astribot_arm_right_link_6" pos="0 0 0.277" quat="0.7071054825112363 -0.7071080798594735 0.0 0.0">
                                  <joint name="{name}-astribot_arm_right_joint_6" type="hinge" ref="0.0" class="{childclass}-motor" range="-0.76 0.76" axis="0 0 1" />
                                  <inertial pos="-0.00010311 0.00001218 0.00013181" quat="1.0 0.0 0.0 0.0" mass="0.22465307" diaginertia="0.00016474 0.00015608 8.555e-05" />
                                  <geom name="{name}-astribot_arm_right_link_6_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-astribot_arm_link_6.STL" class="{childclass}-visual" />
                                  <body name="{name}-astribot_arm_right_link_7" pos="0 0 0" quat="0.7071054825112363 0.0 0.7071080798594735 0.0">
                                    <joint name="{name}-astribot_arm_right_joint_7" type="hinge" ref="0.0" class="{childclass}-motor" range="-1.53 1.53" axis="0 0 1" />
                                    <inertial pos="4.333e-05 -0.00723447 -0.00362428" quat="1.0 0.0 0.0 0.0" mass="0.12772019" diaginertia="0.00013672 0.00010783 5.858e-05" />
                                    <geom name="{name}-astribot_arm_right_link_7_collision" pos="0.006 0 0" quat="1.0 0.0 0.0 0.0" type="sphere" size="0.05" class="{childclass}-collision" />
                                    <geom name="{name}-astribot_arm_right_link_7_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-astribot_arm_link_7.STL" class="{childclass}-visual" />
                                    <body name="{name}-astribot_gripper_right_base" pos="0 -0.045 0" quat="0.7071054825112363 0.7071080798594735 0.0 0.0">
                                      <inertial pos="3.63816834670974E-05 -7.64937801344797E-05 0.0339715454089654" quat="1.0 0.0 0.0 0.0" mass="0.193542076585365" diaginertia="8.74128051113331e-05 0.000114384919729933 0.000110045585286976" />
                                      <geom name="{name}-astribot_gripper_right_base_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-astribot_gripper_base_link.STL" class="{childclass}-visual" />
                                      <body name="{name}-astribot_gripper_right_Link_L1" pos="0.013001 0 0.0537" quat="1.0 0.0 0.0 0.0">
                                        <joint name="{name}-astribot_gripper_right_joint_L1" type="hinge" ref="0.0" class="{childclass}-motor" range="-1.57 1.57" axis="0 1 0" />
                                        <inertial pos="0.0202603407008252 3.25769539416252E-07 0.0195138016274243" quat="1.0 0.0 0.0 0.0" mass="0.0126193026163599" diaginertia="4.00495011266377e-06 2.98446364943016e-06 4.36665411768987e-06" />
                                        <geom name="{name}-astribot_gripper_right_Link_L1_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-default_material" type="mesh" mesh="{name}-astribot_gripper_L1_Link.STL" class="{childclass}-visual" />
                                        <body name="{name}-astribot_gripper_right_Link_L11" pos="0.038 0 0.033" quat="1.0 0.0 0.0 0.0">
                                          <joint name="{name}-astribot_gripper_right_joint_L11" type="hinge" ref="0.0" class="{childclass}-motor" range="-1.57 1.57" axis="0 1 0" />
                                          <inertial pos="0.0078499983581578 2.30865069672272E-06 0.0261255438486819" quat="1.0 0.0 0.0 0.0" mass="0.0422679242486426" diaginertia="7.52065705546739e-06 6.23981721728218e-06 3.81260098942712e-06" />
                                          <geom name="{name}-astribot_gripper_right_Link_L11_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-astribot_gripper_L11_Link.STL" class="{childclass}-collision" />
                                          <geom name="{name}-astribot_gripper_right_Link_L11_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-default_material" type="mesh" mesh="{name}-astribot_gripper_L11_Link.STL" class="{childclass}-visual" />
                                        </body>
                                      </body>
                                      <body name="{name}-astribot_gripper_right_Link_L2" pos="0.033 0 0.0537" quat="1.0 0.0 0.0 0.0">
                                        <joint name="{name}-astribot_gripper_right_joint_L2" type="hinge" ref="0.0" class="{childclass}-motor" range="-1.57 1.57" axis="0 1 0" />
                                        <inertial pos="0.00995690697258714 1.76552698823303E-10 0.0110501818995788" quat="1.0 0.0 0.0 0.0" mass="0.0233645198811733" diaginertia="3.51398687886059e-06 8.26281821632449e-06 7.05331607908947e-06" />
                                        <geom name="{name}-astribot_gripper_right_Link_L2_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-default_material" type="mesh" mesh="{name}-astribot_gripper_L2_Link.STL" class="{childclass}-visual" />
                                      </body>
                                      <body name="{name}-astribot_gripper_right_Link_R1" pos="-0.012999 0 0.0537" quat="1.0 0.0 0.0 0.0">
                                        <joint name="{name}-astribot_gripper_right_joint_R1" type="hinge" ref="0.0" class="{childclass}-motor" range="-1.57 1.57" axis="0 1 0" />
                                        <inertial pos="-0.0202603407007736 -3.25769539812667E-07 0.0195138016274777" quat="1.0 0.0 0.0 0.0" mass="0.0126193026163599" diaginertia="4.00495011267122e-06 2.98446364943016e-06 4.36665411768241e-06" />
                                        <geom name="{name}-astribot_gripper_right_Link_R1_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-default_material" type="mesh" mesh="{name}-astribot_gripper_R1_Link.STL" class="{childclass}-visual" />
                                        <body name="{name}-astribot_gripper_right_Link_R11" pos="-0.038 0 0.033" quat="1.0 0.0 0.0 0.0">
                                          <joint name="{name}-astribot_gripper_right_joint_R11" type="hinge" ref="0.0" class="{childclass}-motor" range="-1.57 1.57" axis="0 1 0" />
                                          <inertial pos="-0.00785031648408113 -2.30865069838854E-06 0.0261252819783987" quat="1.0 0.0 0.0 0.0" mass="0.0422679242485921" diaginertia="7.52064120010972e-06 6.23981721727655e-06 3.81261684477754e-06" />
                                          <geom name="{name}-astribot_gripper_right_Link_R11_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-astribot_gripper_R11_Link.STL" class="{childclass}-collision" />
                                          <geom name="{name}-astribot_gripper_right_Link_R11_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-default_material" type="mesh" mesh="{name}-astribot_gripper_R11_Link.STL" class="{childclass}-visual" />
                                        </body>
                                      </body>
                                      <body name="{name}-astribot_gripper_right_Link_R2" pos="-0.032999 0 0.0537" quat="1.0 0.0 0.0 0.0">
                                        <joint name="{name}-astribot_gripper_right_joint_R2" type="hinge" ref="0.0" class="{childclass}-motor" range="-1.57 1.57" axis="0 1 0" />
                                        <inertial pos="-0.00995690697253525 -1.76552690610476E-10 0.0110501818996256" quat="1.0 0.0 0.0 0.0" mass="0.0233645198811734" diaginertia="3.51398687889016e-06 8.26281821632449e-06 7.05331607905991e-06" />
                                        <geom name="{name}-astribot_gripper_right_Link_R2_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-default_material" type="mesh" mesh="{name}-astribot_gripper_R2_Link.STL" class="{childclass}-visual" />
                                      </body>
                                    </body>
                                  </body>
                                </body>
                              </body>
                            </body>
                          </body>
                        </body>
                      </body>
                    </body>
                  </body>
                </body>
              </body>
            </body>
          </body>
          <site name="{name}-astribot_torso_base_site" pos="0 0 0" quat="1 0 0 0" />
          <camera name="{name}-front_camera" mode="track" fovy="90.0" quat="4.329780281177467e-17 4.329780281177466e-17 0.7071067811865475 0.7071067811865476" pos="0.0 2.0 0.5" />
          <camera name="{name}-side_camera" mode="track" fovy="90.0" quat="-0.5 -0.4999999999999999 0.5 0.5000000000000001" pos="-2.0 0.0 0.5" />
        </body>
    """

    _postamble = """
      <actuator>
        <motor name="{name}-astribot_torso_joint_1_ctrl" joint="{name}-astribot_torso_joint_1" class="{childclass}-motor" />
        <motor name="{name}-astribot_torso_joint_2_ctrl" joint="{name}-astribot_torso_joint_2" class="{childclass}-motor" />
        <motor name="{name}-astribot_torso_joint_3_ctrl" joint="{name}-astribot_torso_joint_3" class="{childclass}-motor" />
        <motor name="{name}-astribot_torso_joint_4_ctrl" joint="{name}-astribot_torso_joint_4" class="{childclass}-motor" />
        <motor name="{name}-astribot_head_joint_1_ctrl" joint="{name}-astribot_head_joint_1" class="{childclass}-motor" />
        <motor name="{name}-astribot_head_joint_2_ctrl" joint="{name}-astribot_head_joint_2" class="{childclass}-motor" />
        <motor name="{name}-astribot_arm_left_joint_1_ctrl" joint="{name}-astribot_arm_left_joint_1" class="{childclass}-motor" />
        <motor name="{name}-astribot_arm_left_joint_2_ctrl" joint="{name}-astribot_arm_left_joint_2" class="{childclass}-motor" />
        <motor name="{name}-astribot_arm_left_joint_3_ctrl" joint="{name}-astribot_arm_left_joint_3" class="{childclass}-motor" />
        <motor name="{name}-astribot_arm_left_joint_4_ctrl" joint="{name}-astribot_arm_left_joint_4" class="{childclass}-motor" />
        <motor name="{name}-astribot_arm_left_joint_5_ctrl" joint="{name}-astribot_arm_left_joint_5" class="{childclass}-motor" />
        <motor name="{name}-astribot_arm_left_joint_6_ctrl" joint="{name}-astribot_arm_left_joint_6" class="{childclass}-motor" />
        <motor name="{name}-astribot_arm_left_joint_7_ctrl" joint="{name}-astribot_arm_left_joint_7" class="{childclass}-motor" />
        <motor name="{name}-astribot_gripper_left_joint_L1_ctrl" joint="{name}-astribot_gripper_left_joint_L1" class="{childclass}-motor" />
        <motor name="{name}-astribot_gripper_left_joint_L11_ctrl" joint="{name}-astribot_gripper_left_joint_L11" class="{childclass}-motor" />
        <motor name="{name}-astribot_gripper_left_joint_L2_ctrl" joint="{name}-astribot_gripper_left_joint_L2" class="{childclass}-motor" />
        <motor name="{name}-astribot_gripper_left_joint_R1_ctrl" joint="{name}-astribot_gripper_left_joint_R1" class="{childclass}-motor" />
        <motor name="{name}-astribot_gripper_left_joint_R11_ctrl" joint="{name}-astribot_gripper_left_joint_R11" class="{childclass}-motor" />
        <motor name="{name}-astribot_gripper_left_joint_R2_ctrl" joint="{name}-astribot_gripper_left_joint_R2" class="{childclass}-motor" />
        <motor name="{name}-astribot_arm_right_joint_1_ctrl" joint="{name}-astribot_arm_right_joint_1" class="{childclass}-motor" />
        <motor name="{name}-astribot_arm_right_joint_2_ctrl" joint="{name}-astribot_arm_right_joint_2" class="{childclass}-motor" />
        <motor name="{name}-astribot_arm_right_joint_3_ctrl" joint="{name}-astribot_arm_right_joint_3" class="{childclass}-motor" />
        <motor name="{name}-astribot_arm_right_joint_4_ctrl" joint="{name}-astribot_arm_right_joint_4" class="{childclass}-motor" />
        <motor name="{name}-astribot_arm_right_joint_5_ctrl" joint="{name}-astribot_arm_right_joint_5" class="{childclass}-motor" />
        <motor name="{name}-astribot_arm_right_joint_6_ctrl" joint="{name}-astribot_arm_right_joint_6" class="{childclass}-motor" />
        <motor name="{name}-astribot_arm_right_joint_7_ctrl" joint="{name}-astribot_arm_right_joint_7" class="{childclass}-motor" />
        <motor name="{name}-astribot_gripper_right_joint_L1_ctrl" joint="{name}-astribot_gripper_right_joint_L1" class="{childclass}-motor" />
        <motor name="{name}-astribot_gripper_right_joint_L11_ctrl" joint="{name}-astribot_gripper_right_joint_L11" class="{childclass}-motor" />
        <motor name="{name}-astribot_gripper_right_joint_L2_ctrl" joint="{name}-astribot_gripper_right_joint_L2" class="{childclass}-motor" />
        <motor name="{name}-astribot_gripper_right_joint_R1_ctrl" joint="{name}-astribot_gripper_right_joint_R1" class="{childclass}-motor" />
        <motor name="{name}-astribot_gripper_right_joint_R11_ctrl" joint="{name}-astribot_gripper_right_joint_R11" class="{childclass}-motor" />
        <motor name="{name}-astribot_gripper_right_joint_R2_ctrl" joint="{name}-astribot_gripper_right_joint_R2" class="{childclass}-motor" />
      </actuator>

      <contact>
        <exclude body1="{name}-astribot_torso_base" body2="{name}-astribot_torso_link_1" />
        <exclude body1="{name}-astribot_torso_link_1" body2="{name}-astribot_torso_link_2" />
        <exclude body1="{name}-astribot_torso_link_2" body2="{name}-astribot_torso_link_3" />
        <exclude body1="{name}-astribot_arm_left_link_3" body2="{name}-astribot_arm_left_link_4" />
        <exclude body1="{name}-astribot_arm_left_link_4" body2="{name}-astribot_arm_left_link_5" />
        <exclude body1="{name}-astribot_arm_right_link_3" body2="{name}-astribot_arm_right_link_4" />
        <exclude body1="{name}-astribot_arm_right_link_4" body2="{name}-astribot_arm_right_link_5" />
      </contact>

      <sensor>
        <framepos name="{name}-astribot_torso_base_site_pos" objtype="site" objname="{name}-astribot_torso_base_site" />
        <framequat name="{name}-astribot_torso_base_site_quat" objtype="site" objname="{name}-astribot_torso_base_site" />
        <framelinvel name="{name}-astribot_torso_base_site_linvel" objtype="site" objname="{name}-astribot_torso_base_site" />
        <frameangvel name="{name}-astribot_torso_base_site_angvel" objtype="site" objname="{name}-astribot_torso_base_site" />
        <velocimeter name="{name}-astribot_torso_base_site_vel" site="{name}-astribot_torso_base_site" />
      </sensor>
      """

if __name__ == '__main__':
    from vuer_mjcf.basic_components.mj_ground_plane import GroundPlane
    from vuer_mjcf.schema.schema import Mjcf
    from vuer_mjcf.utils.file import Prettify
    from vuer_mjcf.utils.file import Save
    import re
    from os.path import basename, dirname, join, splitext

    ground = GroundPlane()
    robot = AstriBot(
        name="astribot",
        assets="."
    )

    # Modify the <compiler> tag by removing meshdir and texturedir
    robot._preamble = re.sub(r'(<compiler[^>]*?)\s+meshdir="[^"]*"', r'\1', robot._preamble)
    robot._preamble = re.sub(r'(<compiler[^>]*?)\s+texturedir="[^"]*"', r'\1', robot._preamble)

    robot._add_mocaps()

    scene: Mjcf = Mjcf(robot)

    file_name = splitext(basename(__file__))[0]
    folder = join(dirname(__file__), "assets", file_name)
    print(folder)
    os.makedirs(folder, exist_ok=True)

    output_path = join(folder, f"{file_name}.mjcf.xml")

    scene._xml | Prettify() | Save(output_path)
