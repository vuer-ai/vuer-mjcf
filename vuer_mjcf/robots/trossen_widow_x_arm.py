import os

from vuer_mjcf.schema.schema import Body, Mjcf, MocapBody
from vuer_mjcf.schema.base import Xml
# from utils.transform import compose
import numpy as np
from scipy.spatial.transform import Rotation as R
import vuer_mjcf.se3.se3_mujoco as m

# from vuer_mjcf.basic_components.rigs.camera_rig import make_camera_rig
# from vuer_mjcf.basic_components.rigs.lighting_rig import make_lighting_rig
# from vuer_mjcf.se3.se3_mujoco import Vector3, WXYZ
def compose(pos1, mujoco_quat1, pos2):
    """
    Compose pos1 + rotation(mujoco_quat1) applied to pos2.
    mujoco_quat1: [w, x, y, z]
    """
    # Convert [w, x, y, z] -> [x, y, z, w]
    quat1_scipy = np.array([mujoco_quat1[1], mujoco_quat1[2], mujoco_quat1[3], mujoco_quat1[0]])

    r1 = R.from_quat(quat1_scipy)
    pos2_world = r1.apply(pos2)
    global_pos = np.array(pos1) + pos2_world

    return m.Vector3(*global_pos.tolist())

class WidowXArm(MocapBody):
    """
    This is the Widow X Arm from Trossen
    """

    assets: str = "trossen_widow_x_arm"
    end_effector: Xml = None
    _attributes = {
        "name": "wx250s",
        "childclass": "wx250s",
    }

    def __init__(self, *_children, end_effector: Xml = None, mocap_pos=None, **kwargs):
        super().__init__(*_children, **kwargs)
        self.end_effector = end_effector
        if end_effector:
            self._children = self._children + (end_effector,)

        # 0.5114, 0.37265
        # [0.36575, 0, 0.36065]

        if mocap_pos is None:
            self.mocap_pos = compose(self._pos, self._quat, [0.455, 0, 0.36065])

            # m.Vector3(0.455, 0, 0.36065))#self._pos + [0.455, 0, 0.36065]
        else:
            self.mocap_pos = mocap_pos
        # self.mocap_quat = self._quat

        # self._children = self._children + (self.mocap_pos,)

        values = self._format_dict()
        self._mocaps = self._mocaps_body.format(**values)

    _mocaps_body = """
    <body mocap="true" name="{name}-mocap" pos="{mocap_pos}">
      <site name="{name}-mocap-site" size=".01" type="sphere" rgba="0.13 0.3 0.2 1"/>
    </body>"""

    _mocaps_equality = """
    <equality>
        <weld body1="{name}-gripper_link" body2="{name}-mocap"/>
    </equality>
    """

    _preamble = """
      <compiler angle="radian" meshdir="assets" texturedir="assets" autolimits="true"/>

      <option integrator="implicitfast" cone="elliptic" impratio="10"/>

      <asset>
        <texture type="2d" file="{assets}/interbotix_black.png"/>
        <material name="{name}-black" texture="interbotix_black"/>

        <mesh file="{assets}/wx250s_1_base.stl" class="{childclass}"/>
        <mesh file="{assets}/wx250s_2_shoulder.stl" class="{childclass}"/>
        <mesh file="{assets}/wx250s_3_upper_arm.stl" class="{childclass}"/>
        <mesh file="{assets}/wx250s_4_upper_forearm.stl" class="{childclass}"/>
        <mesh file="{assets}/wx250s_5_lower_forearm.stl" class="{childclass}"/>
        <mesh file="{assets}/wx250s_6_wrist.stl" class="{childclass}"/>
        <mesh file="{assets}/wx250s_7_gripper.stl" class="{childclass}"/>
        <mesh file="{assets}/wx250s_8_gripper_prop.stl" class="{childclass}"/>
        <mesh file="{assets}/wx250s_9_gripper_bar.stl" class="{childclass}"/>
        <mesh file="{assets}/wx250s_10_gripper_finger.stl" class="{childclass}"/>
      </asset>

      <default>
        <default class="{childclass}">
          <mesh scale="0.001 0.001 0.001"/>
          <joint axis="0 1 0" frictionloss="0.1" armature="0.1"/>
          <position kp="1" inheritrange="1" dampratio="0.95" forcerange="-20 20"/>
          <default class="{childclass}-visual">
            <geom type="mesh" contype="0" conaffinity="0" density="0" group="2" material="{name}-black"/>
          </default>
          <default class="{childclass}-collision">
            <geom group="3" type="mesh"/>
            <default class="{childclass}-sphere_collision">
              <geom type="sphere" size="0.0006" rgba="1 0 0 1"/>
            </default>
          </default>
        </default>
      </default>
    """

    template = """
        <body {attributes}>
      <inertial pos="-0.0380446 0.000613892 0.0193354" quat="0.509292 0.490887 -0.496359 0.503269" mass="0.538736"
        diaginertia="0.00252518 0.00211519 0.000690737"/>
      <geom quat="1 0 0 1" mesh="wx250s_1_base" class="{childclass}-visual"/>
      <geom quat="1 0 0 1" mesh="wx250s_1_base" class="{childclass}-collision"/>
      <body name="{name}-shoulder_link" pos="0 0 0.072">
        <inertial pos="2.23482e-05 4.14609e-05 0.0066287" quat="0.0130352 0.706387 0.012996 0.707586" mass="0.480879"
          diaginertia="0.000588946 0.000555655 0.000378999"/>
        <joint name="{name}-waist" axis="0 0 1" range="-3.14158 3.14158"/>
        <geom pos="0 0 -0.003" quat="1 0 0 1" mesh="wx250s_2_shoulder" class="{childclass}-visual"/>
        <geom pos="0 0 -0.003" quat="1 0 0 1" mesh="wx250s_2_shoulder" class="{childclass}-collision"/>
        <body name="{name}-upper_arm_link" pos="0 0 0.03865">
          <inertial pos="0.0171605 2.725e-07 0.191323" quat="0.705539 0.0470667 -0.0470667 0.705539" mass="0.430811"
            diaginertia="0.00364425 0.003463 0.000399348"/>
          <joint name="{name}-shoulder" range="-1.88496 1.98968"/>
          <geom quat="1 0 0 1" mesh="wx250s_3_upper_arm" class="{childclass}-visual"/>
          <geom quat="1 0 0 1" mesh="wx250s_3_upper_arm" class="{childclass}-collision"/>
          <body name="{name}-upper_forearm_link" pos="0.04975 0 0.25">
            <inertial pos="0.107963 0.000115876 0" quat="0.000980829 0.707106 -0.000980829 0.707106" mass="0.234589"
              diaginertia="0.000888 0.000887807 3.97035e-05"/>
            <joint name="{name}-elbow" range="-2.14675 1.6057"/>
            <geom mesh="wx250s_4_upper_forearm" class="{childclass}-visual"/>
            <geom mesh="wx250s_4_upper_forearm" class="{childclass}-collision"/>
            <body name="{name}-lower_forearm_link" pos="0.175 0 0">
              <inertial pos="0.0374395 0.00522252 0" quat="-0.0732511 0.703302 0.0732511 0.703302" mass="0.220991"
                diaginertia="0.0001834 0.000172527 5.88633e-05"/>
              <joint name="{name}-forearm_roll" axis="1 0 0" range="-3.14158 3.14158"/>
              <geom quat="0 1 0 0" mesh="wx250s_5_lower_forearm" class="{childclass}-visual"/>
              <geom quat="0 1 0 0" mesh="wx250s_5_lower_forearm" class="{childclass}-collision"/>
              <body name="{name}-wrist_link" pos="0.075 0 0">
                <inertial pos="0.04236 -1.0663e-05 0.010577" quat="0.608721 0.363497 -0.359175 0.606895" mass="0.084957"
                  diaginertia="3.29057e-05 3.082e-05 2.68343e-05"/>
                <joint name="{name}-wrist_angle" axis="0 1 0" range="-1.74533 2.14675"/>
                <geom quat="1 0 0 1" mesh="wx250s_6_wrist" class="{childclass}-visual"/>
                <geom quat="1 0 0 1" mesh="wx250s_6_wrist" class="{childclass}-collision"/>
                <body name="{name}-gripper_link" pos="0.065 0 0">
                  <body name="gripper_actuator_body" pos="0 0 0">
                    <joint name="gripper_joint" type="slide" axis="1 0 0" limited="true" range="0 1" ref="0" damping="5"/>
                    <inertial pos="0 0 0" mass="1e-4" diaginertia="1e-6 1e-6 1e-6"/>
                  </body>
                  <body name="gripper_inverter" pos="0 0 0">
                    <joint name="gripper_master" type="slide" axis="1 0 0" limited="true" range="0 1" ref="0" damping="5"/>
                    <inertial mass="1e-4" diaginertia="1e-6 1e-6 1e-6" pos="0 0 0"/>
                  </body>
                  <inertial pos="0.0325296 4.2061e-07 0.0090959" quat="0.546081 0.419626 0.62801 0.362371"
                    mass="0.110084" diaginertia="0.00307592 0.00307326 0.0030332"/>
                  <joint name="{name}-wrist_rotate" axis="1 0 0" range="-3.14158 3.14158"/>
                  <geom pos="-0.02 0 0" quat="1 0 0 1" mesh="wx250s_7_gripper" class="{childclass}-visual"/>
                  <geom pos="-0.02 0 0" quat="1 0 0 1" mesh="wx250s_7_gripper" class="{childclass}-collision"/>
                  <geom pos="-0.02 0 0" quat="1 0 0 1" mesh="wx250s_9_gripper_bar" class="{childclass}-visual"/>
                  <geom pos="-0.02 0 0" quat="1 0 0 1" mesh="wx250s_9_gripper_bar" class="{childclass}-collision"/>
                  <body name="{name}-left_finger_link" pos="0.066 0.01 0">
                    <inertial pos="0.013816 0 0" quat="0.705384 0.705384 -0.0493271 -0.0493271" mass="0.016246"
                      diaginertia="4.79509e-06 3.7467e-06 1.48651e-06"/>
                    <joint name="{name}-left_finger" axis="0 1 0" type="slide" range="0.01 0.04" ref="0.01"/>
                    <geom pos="0 0.005 0" quat="0 0 0 -1" mesh="wx250s_10_gripper_finger" class="{childclass}-visual"/>
                    <geom pos="0 0.005 0" quat="0 0 0 -1" mesh="wx250s_10_gripper_finger" class="{childclass}-collision"/>
                    <geom name="left/left_g0" pos="0.042 -0.009 0.012" class="{childclass}-sphere_collision"/>
                    <geom name="left/left_g1" pos="0.042 -0.009 -0.012" class="{childclass}-sphere_collision"/>
                  </body>
                  <body name="{name}-right_finger_link" pos="0.066 -0.01 0">
                    <inertial pos="0.013816 0 0" quat="0.705384 0.705384 0.0493271 0.0493271" mass="0.016246"
                      diaginertia="4.79509e-06 3.7467e-06 1.48651e-06"/>
                    <joint name="wx250s-right_finger" axis="0 1 0" type="slide" range="-0.04 -0.01" ref="-0.01"/>
                    <geom pos="0 -0.005 0" quat="0 0 1 0" mesh="wx250s_10_gripper_finger" class="{childclass}-visual"/>
                    <geom pos="0 -0.005 0" quat="0 0 1 0" mesh="wx250s_10_gripper_finger" class="{childclass}-collision"/>
                    <geom name="{name}-right-right_g0" pos="0.042 0.009 0.012" class="{childclass}-sphere_collision"/>
                    <geom name="{name}-right-right_g1" pos="0.042 0.009 -0.012" class="{childclass}-sphere_collision"/>
                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
    """

    _postamble = """
      <contact>
        <exclude body1="{name}" body2="{name}-shoulder_link"/>
      </contact>

    <equality>
        <!-- inversion: gripper_joint = 1 - gripper_master -->
        <joint name="invert_gripper"
               joint1="gripper_master"
               joint2="gripper_joint"
               polycoef="1 -1 0 0 0"
               solref="0.02 1" solimp="0.9 0.95 0.001"/>

        <!-- (keep or adjust your existing couplings from gripper_joint to fingers) -->
        <joint name="left_to_gripper"
               joint1="wx250s-left_finger"
               joint2="gripper_joint"
               polycoef="0 0.03 0 0 0"
               solref="0.02 1" solimp="0.9 0.95 0.001"/>
        <joint name="right_to_gripper"
               joint1="wx250s-right_finger"
               joint2="gripper_joint"
               polycoef="0 -0.03 0 0 0"
               solref="0.02 1" solimp="0.9 0.95 0.001"/>
        <weld body1="wx250s-gripper_link" body2="wx250s-mocap"/>

    </equality>

    <actuator>
        <!-- actuator now drives gripper_master; 0 => gripper_master=0 => gripper_joint=1 (inverted) -->
        <position name="gripper_servo" joint="gripper_master" kp="50" ctrlrange="0 1"/>
    </actuator>

      """

if __name__ == "__main__":
    from vuer_mjcf.basic_components.mj_ground_plane import GroundPlane
    from vuer_mjcf.schema.schema import Mjcf
    from vuer_mjcf.utils.file import Prettify
    from vuer_mjcf.utils.file import Save
    import re
    from os.path import basename, dirname, join, splitext

    ground = GroundPlane()
    robot = WidowXArm(name="trossen_widow_x_arm", assets=".")

    # Modify the <compiler> tag by removing meshdir and texturedir
    robot._preamble = re.sub(r'(<compiler[^>]*?)\s+meshdir="[^"]*"', r"\1", robot._preamble)
    robot._preamble = re.sub(r'(<compiler[^>]*?)\s+texturedir="[^"]*"', r"\1", robot._preamble)

    robot._add_mocaps()

    scene: Mjcf = Mjcf(robot)

    file_name = splitext(basename(__file__))[0]
    folder = join(dirname(__file__), "assets", file_name)
    print(folder)
    os.makedirs(folder, exist_ok=True)

    output_path = join(folder, f"{file_name}.mjcf.xml")

    scene._xml | Prettify() | Save(output_path)
