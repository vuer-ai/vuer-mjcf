import os

from vuer_mjcf.schema.schema import Body, Mjcf, MocapBody
from vuer_mjcf.schema.base import Xml
from utils.transform import compose
# from vuer_mjcf.basic_components.rigs.camera_rig import make_camera_rig
# from vuer_mjcf.basic_components.rigs.lighting_rig import make_lighting_rig
# from vuer_mjcf.se3.se3_mujoco import Vector3, WXYZ

class X5a(MocapBody):
    """
    This is X5A from ARX
    """

    assets: str = "x5a"
    end_effector: Xml = None
    _attributes = {
        "name": "x5a",
        "childclass": "x5a",
    }

    def __init__(self, *_children, end_effector: Xml = None, mocap_pos=None, **kwargs):
        super().__init__(*_children, **kwargs)
        self.end_effector = end_effector
        if end_effector:
            self._children = self._children + (end_effector,)

        # 0.5114, 0.37265
        # [0.36575, 0, 0.36065]
        if mocap_pos is None:
            self.mocap_pos = self._pos + [0.2, 0, 0.16]
        else:
            self.mocap_pos = mocap_pos

        # self._children = self._children + (self.mocap_pos,)

        self.mocap_pos = compose(self._pos, self._quat, [0.2,  0.0,  0.16])

        values = self._format_dict()
        self._mocaps = self._mocaps_body.format(**values)

    _mocaps_body = """
    <body mocap="true" name="{name}-mocap-1" pos="{mocap_pos}">
      <site name="{name}-mocap-site-1" size=".035" type="sphere" rgba="0.13 0.3 0.2 1"/>
    </body>
    """

    _mocaps_equality = """
    <equality>
        <weld body1="{name}-link6" body2="{name}-mocap-1"/>
    </equality>
    """

    _preamble = """
      <default>
        <default class="{childclass}">
          <default class="{childclass}-motor">
            <joint />
            <motor />
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
        <material name="{name}-" rgba="1 1 1 1" />
        <material name="{name}-default_material" rgba="0.7 0.7 0.7 1" />
        <material name="{name}-collision_material" rgba="1.0 0.28 0.1 0.9" />
        <mesh name="{name}-base_link.STL" file="{assets}/base_link.STL" />
        <mesh name="{name}-link1.STL" file="{assets}/link1.STL" />
        <mesh name="{name}-link2.STL" file="{assets}/link2.STL" />
        <mesh name="{name}-link3.STL" file="{assets}/link3.STL" />
        <mesh name="{name}-link4.STL" file="{assets}/link4.STL" />
        <mesh name="{name}-link5.STL" file="{assets}/link5.STL" />
        <mesh name="{name}-link6.STL" file="{assets}/link6.STL" />
        <mesh name="{name}-link7.STL" file="{assets}/link7.STL" />
        <mesh name="{name}-link8.STL" file="{assets}/link8.STL" />
      </asset>
    """

    template = """
        <body {attributes}>
          <joint name="{name}-floating_base" />
          <geom name="{name}-base_link_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-base_link.STL" class="{childclass}-collision" />
          <geom name="{name}-base_link_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-base_link.STL" class="{childclass}-visual" />
          <body name="{name}-link1" pos="0 0 0.0605" quat="1.0 0.0 0.0 0.0">
            <joint name="{name}-joint1" type="hinge" ref="0.0" class="{childclass}-motor" range="-10 10" axis="0 0 1" />
            <inertial pos="0.0054231 -0.0080289 0.017086" quat="1.0 0.0 0.0 0.0" mass="0.066982" diaginertia="8e-05 3e-05 8e-05" />
            <geom name="{name}-link1_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-link1.STL" class="{childclass}-collision" />
            <geom name="{name}-link1_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-link1.STL" class="{childclass}-visual" />
            <body name="{name}-link2" pos="0.02 0 0.04" quat="1.0 0.0 0.0 0.0">
              <joint name="{name}-joint2" type="hinge" ref="0.0" class="{childclass}-motor" range="-10 10" axis="0 1 0" />
              <inertial pos="-0.13237 0.0020852 0.00010549" quat="1.0 0.0 0.0 0.0" mass="1.0795" diaginertia="0.00051 0.01599 0.01605" />
              <geom name="{name}-link2_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-link2.STL" class="{childclass}-collision" />
              <geom name="{name}-link2_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-link2.STL" class="{childclass}-visual" />
              <body name="{name}-link3" pos="-0.264 0 0" quat="-3.673205103346574e-06 0.9999999999932537 0.0 -0.0">
                <joint name="{name}-joint3" type="hinge" ref="0.0" class="{childclass}-motor" range="-10 10" axis="0 1 0" />
                <inertial pos="0.18531 0.00068376 -0.051638" quat="1.0 0.0 0.0 0.0" mass="0.54534" diaginertia="0.00036 0.00423 0.0042" />
                <geom name="{name}-link3_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-link3.STL" class="{childclass}-collision" />
                <geom name="{name}-link3_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-link3.STL" class="{childclass}-visual" />
                <body name="{name}-link4" pos="0.245 0 -0.056" quat="1.0 0.0 0.0 0.0">
                  <joint name="{name}-joint4" type="hinge" ref="0.0" class="{childclass}-motor" range="-10 10" axis="0 1 0" />
                  <inertial pos="0.040231 0.0044807 -0.035335" quat="1.0 0.0 0.0 0.0" mass="0.11714" diaginertia="0.00019 0.00023 0.00014" />
                  <geom name="{name}-link4_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-link4.STL" class="{childclass}-collision" />
                  <geom name="{name}-link4_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-link4.STL" class="{childclass}-visual" />
                  <body name="{name}-link5" pos="0.06775 0.0005 -0.0865" quat="1.0 0.0 0.0 0.0">
                    <joint name="{name}-joint5" type="hinge" ref="0.0" class="{childclass}-motor" range="-10 10" axis="0 0 1" />
                    <inertial pos="0.003612 -1.5455E-05 0.055214" quat="1.0 0.0 0.0 0.0" mass="0.63488" diaginertia="0.00083 0.00082 0.00026" />
                    <geom name="{name}-link5_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-link5.STL" class="{childclass}-collision" />
                    <geom name="{name}-link5_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-link5.STL" class="{childclass}-visual" />
                    <body name="{name}-link6" pos="0.02895 0 0.0865" quat="-3.673205103346574e-06 -0.9999999999932537 -0.0 0.0">
                      <joint name="{name}-joint6" type="hinge" ref="0.0" class="{childclass}-motor" range="-10 10" axis="1 0 0" />
                      <inertial pos="0.041697 2.4368E-05 0.00014464" quat="1.0 0.0 0.0 0.0" mass="0.44089" diaginertia="0.00038 0.00028 0.0005" />
                      <geom name="{name}-link6_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-link6.STL" class="{childclass}-collision" />
                      <geom name="{name}-link6_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-link6.STL" class="{childclass}-visual" />
                      <body name="{name}-link7" pos="0.08657 0.024896 -0.0002436" quat="1.0 0.0 0.0 0.0">
                        <joint name="{name}-joint7" type="slide" ref="0.0" class="{childclass}-motor" range="0 0.044" axis="0 1 0" />
                        <inertial pos="-0.00035522 -0.007827 -0.0029883" quat="1.0 0.0 0.0 0.0" mass="0.064798" diaginertia="2e-05 3e-05 3e-05" />
                        <geom name="{name}-link7_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-link7.STL" class="{childclass}-collision" />
                        <geom name="{name}-link7_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-link7.STL" class="{childclass}-visual" />
                      </body>
                      <body name="{name}-link8" pos="0.08657 -0.0249 -0.00024366" quat="1.0 0.0 0.0 0.0">
                        <joint name="{name}-joint8" type="slide" ref="0.0" class="{childclass}-motor" range="0 0.044" axis="0 -1 0" />
                        <inertial pos="-0.00035522 0.0078277 0.0024201" quat="1.0 0.0 0.0 0.0" mass="0.064798" diaginertia="2e-05 3e-05 3e-05" />
                        <geom name="{name}-link8_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-link8.STL" class="{childclass}-collision" />
                        <geom name="{name}-link8_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-link8.STL" class="{childclass}-visual" />
                      </body>
                    </body>
                  </body>
                </body>
              </body>
            </body>
          </body>
          <site name="{name}-base_link_site" pos="0 0 0" quat="1 0 0 0" />
          <camera name="{name}-front_camera" mode="track" fovy="90.0" quat="4.329780281177467e-17 4.329780281177466e-17 0.7071067811865475 0.7071067811865476" pos="0.0 2.0 0.5" />
          <camera name="{name}-side_camera" mode="track" fovy="90.0" quat="-0.5 -0.4999999999999999 0.5 0.5000000000000001" pos="-2.0 0.0 0.5" />
        </body>
    """

    _postamble = """
      <actuator>
        <motor name="{name}-joint1_ctrl" joint="{name}-joint1" class="{childclass}-motor" />
        <motor name="{name}-joint2_ctrl" joint="{name}-joint2" class="{childclass}-motor" />
        <motor name="{name}-joint3_ctrl" joint="{name}-joint3" class="{childclass}-motor" />
        <motor name="{name}-joint4_ctrl" joint="{name}-joint4" class="{childclass}-motor" />
        <motor name="{name}-joint5_ctrl" joint="{name}-joint5" class="{childclass}-motor" />
        <motor name="{name}-joint6_ctrl" joint="{name}-joint6" class="{childclass}-motor" />
        <motor name="{name}-joint7_ctrl" joint="{name}-joint7" class="{childclass}-motor" />
        <motor name="{name}-joint8_ctrl" joint="{name}-joint8" class="{childclass}-motor" />
      </actuator>

      <contact>
        <exclude body1="{name}" body2="{name}-link1" />
        <exclude body1="{name}-link1" body2="{name}-link2" />
        <exclude body1="{name}-link2" body2="{name}-link3" />
        <exclude body1="{name}-link3" body2="{name}-link4" />
        <exclude body1="{name}-link4" body2="{name}-link5" />
        <exclude body1="{name}-link5" body2="{name}-link6" />
        <exclude body1="{name}-link6" body2="{name}-link7" />
        <exclude body1="{name}-link6" body2="{name}-link8" />
      </contact>

      <sensor>
        <framepos name="{name}-base_link_site_pos" objtype="site" objname="{name}-base_link_site" />
        <framequat name="{name}-base_link_site_quat" objtype="site" objname="{name}-base_link_site" />
        <framelinvel name="{name}-base_link_site_linvel" objtype="site" objname="{name}-base_link_site" />
        <frameangvel name="{name}-base_link_site_angvel" objtype="site" objname="{name}-base_link_site" />
        <velocimeter name="{name}-base_link_site_vel" site="{name}-base_link_site" />
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
    robot = X5a(
        name="x5a",
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
