import os

from vuer_mjcf.schema.schema import Body, Mjcf, MocapBody
from vuer_mjcf.schema.base import Xml
# from vuer_mjcf.basic_components.rigs.camera_rig import make_camera_rig
# from vuer_mjcf.basic_components.rigs.lighting_rig import make_lighting_rig
# from vuer_mjcf.se3.se3_mujoco import Vector3, WXYZ

class X7s(MocapBody):
    """
    This is X7S from ARX
    """

    assets: str = "x7s"
    end_effector: Xml = None

    def __init__(self, *_children, end_effector: Xml = None, mocap_pos=None, **kwargs):
        super().__init__(*_children, **kwargs)
        self.end_effector = end_effector
        if end_effector:
            self._children = self._children + (end_effector,)

        # 0.5114, 0.37265
        # [0.36575, 0, 0.36065]
        if mocap_pos is None:
            self.mocap_pos = self._pos + [-0.387, 0.000, 0.408]
        else:
            self.mocap_pos = mocap_pos
        values = self._format_dict()
        self._mocaps = self._mocaps_body.format(**values)
        # self._children = self._children + (self.mocap_pos,)

    mocap_quat = "0 0 1 0"
    mocap_pos_left = "0.500  0.15  0.285"
    mocap_pos_right = "0.500 -0.15  0.285"
    mocap_pos_head = "0.162 -0.025  0.725"
    pos = "0 0 0"

    _attributes = {
        "name": "x7s",
        "childclass": "x7s",
        "pos": "0 0 0",
        "quat": "1 0 0 0",
    }

    _mocaps_body = """
    <body mocap="true" name="{name}-mocap-1" pos="{mocap_pos_left}" quat="{mocap_quat}">
      <site name="{name}-mocap-site-1" size=".05" type="sphere" rgba="0.13 0.3 0.2 1"/>
    </body>
    <body mocap="true" name="{name}-mocap-2" pos="{mocap_pos_right}" quat="{mocap_quat}">
      <site name="{name}-mocap-site-2" size=".05" type="sphere" rgba="0.13 0.3 0.2 1"/>
    </body>
    <body mocap="true" name="{name}-mocap-3" pos="{mocap_pos_head}" quat="{mocap_quat}">
      <site name="{name}-mocap-site-3" size=".05" type="sphere" rgba="0.13 0.3 0.2 1"/>
    </body>
    """

    _mocaps_equality = """
    <equality>
        <weld body1="{name}-link11" body2="{name}-mocap-1"/>
        <weld body1="{name}-link20" body2="{name}-mocap-2"/>
        <weld body1="{name}-link4" body2="{name}-mocap-3"/>
    </equality>
    """

    _preamble = """
      <default>
        <default class="{childclass}-robot">
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
        <material name="{name}-" rgba="0.898039215686275 0.917647058823529 0.929411764705882 1" />
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
        <mesh name="{name}-link9.STL" file="{assets}/link9.STL" />
        <mesh name="{name}-link10.STL" file="{assets}/link10.STL" />
        <mesh name="{name}-link11.STL" file="{assets}/link11.STL" />
        <mesh name="{name}-link12.STL" file="{assets}/link12.STL" />
        <mesh name="{name}-link13.STL" file="{assets}/link13.STL" />
        <mesh name="{name}-link14.STL" file="{assets}/link14.STL" />
        <mesh name="{name}-link15.STL" file="{assets}/link15.STL" />
        <mesh name="{name}-link16.STL" file="{assets}/link16.STL" />
        <mesh name="{name}-link17.STL" file="{assets}/link17.STL" />
        <mesh name="{name}-link18.STL" file="{assets}/link18.STL" />
        <mesh name="{name}-link19.STL" file="{assets}/link19.STL" />
        <mesh name="{name}-link20.STL" file="{assets}/link20.STL" />
        <mesh name="{name}-link21.STL" file="{assets}/link21.STL" />
        <mesh name="{name}-link22.STL" file="{assets}/link22.STL" />
      </asset>
    """

    template = """
        <body name="{name}-base_link" pos="0.00000000 0.00000000 0.00000000" quat="1 0 0 0" childclass="{childclass}-robot">
          <joint name="{name}-floating_base" />
          <geom name="{name}-base_link_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-base_link.STL" class="{childclass}-collision" />
          <geom name="{name}-base_link_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-base_link.STL" class="{childclass}-visual" />
          <body name="{name}-link1" pos="0.096 0 0.31" quat="1.0 0.0 0.0 0.0">
            <joint name="{name}-joint1" type="slide" ref="0.0" class="{childclass}-motor" range="0 0.54" axis="0 0 1" />
            <inertial pos="-0.010805 -0.00069779 -0.059694" quat="1.0 0.0 0.0 0.0" mass="6.205" diaginertia="0.01994 0.06039 0.06721" />
            <geom name="{name}-link1_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-link1.STL" class="{childclass}-collision" />
            <geom name="{name}-link1_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-link1.STL" class="{childclass}-visual" />
            <body name="{name}-link2" pos="0.066 -0.0546 -0.06" quat="1.0 0.0 0.0 0.0">
              <joint name="{name}-joint2" type="hinge" ref="0.0" class="{childclass}-motor" range="-10 10" axis="0 -1 0" />
              <inertial pos="-0.000779578835400224 0.053760645280584 0.220464314501233" quat="1.0 0.0 0.0 0.0" mass="3.01262653552798" diaginertia="0.03987 0.03461 0.00967" />
              <geom name="{name}-link2_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-link2.STL" class="{childclass}-collision" />
              <geom name="{name}-link2_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-link2.STL" class="{childclass}-visual" />
              <body name="{name}-link3" pos="0 0.0544 0.356" quat="1.0 0.0 0.0 0.0">
                <joint name="{name}-joint3" type="hinge" ref="0.0" class="{childclass}-motor" range="-10 10" axis="0 0 1" />
                <inertial pos="-1.2166451790474E-08 0.000155101086835956 0.0414775143344981" quat="1.0 0.0 0.0 0.0" mass="0.429406588376073" diaginertia="0.00027 0.00026 0.00018" />
                <geom name="{name}-link3_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-link3.STL" class="{childclass}-collision" />
                <geom name="{name}-link3_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-link3.STL" class="{childclass}-visual" />
                <body name="{name}-link4" pos="0 -0.025 0.049" quat="1.0 0.0 0.0 0.0">
                  <joint name="{name}-joint4" type="hinge" ref="0.0" class="{childclass}-motor" range="-10 10" axis="0 -1 0" />
                  <inertial pos="0.0145474612596931 0.0229595215513494 0.0787914285814235" quat="1.0 0.0 0.0 0.0" mass="1.54420437039843" diaginertia="0.00576 0.00477 0.00768" />
                  <geom name="{name}-link4_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-link4.STL" class="{childclass}-collision" />
                  <geom name="{name}-link4_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-link4.STL" class="{childclass}-visual" />
                </body>
              </body>
              <body name="{name}-link5" pos="0 0.1424 0.28" quat="1.0 0.0 0.0 0.0">
                <joint name="{name}-joint5" type="hinge" ref="0.0" class="{childclass}-motor" range="-10 10" axis="0 1 0" />
                <inertial pos="0.0203118776657915 0.0282074621926044 -0.000172803675324795" quat="1.0 0.0 0.0 0.0" mass="0.131891884746085" diaginertia="0.00012 9e-05 0.00014" />
                <geom name="{name}-link5_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-link5.STL" class="{childclass}-collision" />
                <geom name="{name}-link5_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-link5.STL" class="{childclass}-visual" />
                <body name="{name}-link6" pos="0.02725 0.063 0" quat="1.0 0.0 0.0 0.0">
                  <joint name="{name}-joint6" type="hinge" ref="0.0" class="{childclass}-motor" range="-10 10" axis="-1 0 0" />
                  <inertial pos="-0.0244518842072319 0.00440119803297737 -0.0251861647177716" quat="1.0 0.0 0.0 0.0" mass="1.03161675931573" diaginertia="0.00167 0.00166 0.00088" />
                  <geom name="{name}-link6_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-link6.STL" class="{childclass}-collision" />
                  <geom name="{name}-link6_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-link6.STL" class="{childclass}-visual" />
                  <body name="{name}-link7" pos="-0.02725 0 -0.0865" quat="1.0 0.0 0.0 0.0">
                    <joint name="{name}-joint7" type="hinge" ref="0.0" class="{childclass}-motor" range="-10 10" axis="0 0 -1" />
                    <inertial pos="0.000827778295739684 -0.00148365067233044 -0.105295191761956" quat="1.0 0.0 0.0 0.0" mass="0.736099125577072" diaginertia="0.00229 0.0023 0.0006" />
                    <geom name="{name}-link7_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-link7.STL" class="{childclass}-collision" />
                    <geom name="{name}-link7_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-link7.STL" class="{childclass}-visual" />
                    <body name="{name}-link8" pos="0 -0.02725 -0.1435" quat="1.0 0.0 0.0 0.0">
                      <joint name="{name}-joint8" type="hinge" ref="0.0" class="{childclass}-motor" range="-10 10" axis="0 -1 0" />
                      <inertial pos="0.0593724453089232 0.0253235620813382 -0.0125969681001941" quat="1.0 0.0 0.0 0.0" mass="0.487506795308259" diaginertia="0.0004 0.00055 0.00069" />
                      <geom name="{name}-link8_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-link8.STL" class="{childclass}-collision" />
                      <geom name="{name}-link8_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-link8.STL" class="{childclass}-visual" />
                      <body name="{name}-link9" pos="0.0945 0.02725 -0.015" quat="1.0 0.0 0.0 0.0">
                        <joint name="{name}-joint9" type="hinge" ref="0.0" class="{childclass}-motor" range="-10 10" axis="1 0 0" />
                        <inertial pos="0.0458969495196738 -0.000535552755431795 -9.66305755581987E-05" quat="1.0 0.0 0.0 0.0" mass="0.487431771518004" diaginertia="0.00024 0.00027 0.00029" />
                        <geom name="{name}-link9_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-link9.STL" class="{childclass}-collision" />
                        <geom name="{name}-link9_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-link9.STL" class="{childclass}-visual" />
                        <body name="{name}-link10" pos="0.05 -0.02725 0" quat="1.0 0.0 0.0 0.0">
                          <joint name="{name}-joint10" type="hinge" ref="0.0" class="{childclass}-motor" range="-10 10" axis="0 -1 0" />
                          <inertial pos="0.0575036493913677 0.0264951119112793 0.00135889493052876" quat="1.0 0.0 0.0 0.0" mass="0.432944265521347" diaginertia="0.00025 0.00033 0.00041" />
                          <geom name="{name}-link10_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-link10.STL" class="{childclass}-collision" />
                          <geom name="{name}-link10_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-link10.STL" class="{childclass}-visual" />
                          <body name="{name}-link11" pos="0.066 0.02725 -0.022" quat="1.0 0.0 0.0 0.0">
                            <joint name="{name}-joint11" type="hinge" ref="0.0" class="{childclass}-motor" range="-10 10" axis="0 0 -1" />
                            <inertial pos="0.0782374674886107 2.27688611500743E-05 0.0203907673011059" quat="1.0 0.0 0.0 0.0" mass="0.471847835335863" diaginertia="0.0004 0.00043 0.00064" />
                            <geom name="{name}-link11_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-link11.STL" class="{childclass}-collision" />
                            <geom name="{name}-link11_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-link11.STL" class="{childclass}-visual" />
                            <body name="{name}-link12" pos="0.12737 0.024896 0.021756" quat="1.0 0.0 0.0 0.0">
                              <joint name="{name}-joint12" type="slide" ref="0.0" class="{childclass}-motor" range="0 0.044" axis="0 1 0" />
                              <inertial pos="-0.000355222592141713 -0.00782703850949387 -0.00298831676915334" quat="1.0 0.0 0.0 0.0" mass="0.0647981743053201" diaginertia="2e-05 3e-05 3e-05" />
                              <geom name="{name}-link12_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-link12.STL" class="{childclass}-collision" />
                              <geom name="{name}-link12_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-link12.STL" class="{childclass}-visual" />
                            </body>
                            <body name="{name}-link13" pos="0.12737 -0.0249 0.021756" quat="1.0 0.0 0.0 0.0">
                              <joint name="{name}-joint13" type="slide" ref="0.0" class="{childclass}-motor" range="0 0.044" axis="0 -1 0" />
                              <inertial pos="-0.000355223470276722 0.00782768741060949 0.00242005652832489" quat="1.0 0.0 0.0 0.0" mass="0.0647981725781686" diaginertia="2e-05 3e-05 3e-05" />
                              <geom name="{name}-link13_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-link13.STL" class="{childclass}-collision" />
                              <geom name="{name}-link13_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-link13.STL" class="{childclass}-visual" />
                            </body>
                          </body>
                        </body>
                      </body>
                    </body>
                  </body>
                </body>
              </body>
              <body name="{name}-link14" pos="0 -0.0336 0.28" quat="1.0 0.0 0.0 0.0">
                <joint name="{name}-joint14" type="hinge" ref="0.0" class="{childclass}-motor" range="-10 10" axis="0 -1 0" />
                <inertial pos="0.0203118776657806 -0.0282074621926024 -0.000172803675324684" quat="1.0 0.0 0.0 0.0" mass="0.131891884746085" diaginertia="0.00012 9e-05 0.00014" />
                <geom name="{name}-link14_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-link14.STL" class="{childclass}-collision" />
                <geom name="{name}-link14_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-link14.STL" class="{childclass}-visual" />
                <body name="{name}-link15" pos="0.02725 -0.063 0" quat="1.0 0.0 0.0 0.0">
                  <joint name="{name}-joint15" type="hinge" ref="0.0" class="{childclass}-motor" range="-10 10" axis="-1 0 0" />
                  <inertial pos="-0.0244518746422262 -0.00440118072836032 -0.0251861194488474" quat="1.0 0.0 0.0 0.0" mass="1.03161748745213" diaginertia="0.00167 0.00166 0.00088" />
                  <geom name="{name}-link15_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-link15.STL" class="{childclass}-collision" />
                  <geom name="{name}-link15_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-link15.STL" class="{childclass}-visual" />
                  <body name="{name}-link16" pos="-0.02725 0 -0.0865" quat="1.0 0.0 0.0 0.0">
                    <joint name="{name}-joint16" type="hinge" ref="0.0" class="{childclass}-motor" range="-10 10" axis="0 0 -1" />
                    <inertial pos="0.000827779970195369 0.0014843542246373 -0.105295189564578" quat="1.0 0.0 0.0 0.0" mass="0.73609919110615" diaginertia="0.00229 0.0023 0.0006" />
                    <geom name="{name}-link16_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-link16.STL" class="{childclass}-collision" />
                    <geom name="{name}-link16_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-link16.STL" class="{childclass}-visual" />
                    <body name="{name}-link17" pos="0 0.02725 -0.1435" quat="1.0 0.0 0.0 0.0">
                      <joint name="{name}-joint17" type="hinge" ref="0.0" class="{childclass}-motor" range="-10 10" axis="0 1 0" />
                      <inertial pos="0.0593724456215453 -0.025323327171846 -0.0127286630831199" quat="1.0 0.0 0.0 0.0" mass="0.48750674832378" diaginertia="0.0004 0.00055 0.00069" />
                      <geom name="{name}-link17_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-link17.STL" class="{childclass}-collision" />
                      <geom name="{name}-link17_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-link17.STL" class="{childclass}-visual" />
                      <body name="{name}-link18" pos="0.0945 -0.02725 -0.015" quat="1.0 0.0 0.0 0.0">
                        <joint name="{name}-joint18" type="hinge" ref="0.0" class="{childclass}-motor" range="-10 10" axis="1 0 0" />
                        <inertial pos="0.0458969709559334 0.000535552755438318 -2.35788959310756E-05" quat="1.0 0.0 0.0 0.0" mass="0.487431771518004" diaginertia="0.00024 0.00027 0.00029" />
                        <geom name="{name}-link18_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-link18.STL" class="{childclass}-collision" />
                        <geom name="{name}-link18_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-link18.STL" class="{childclass}-visual" />
                        <body name="{name}-link19" pos="0.05 0.02725 0" quat="1.0 0.0 0.0 0.0">
                          <joint name="{name}-joint19" type="hinge" ref="0.0" class="{childclass}-motor" range="-10 10" axis="0 1 0" />
                          <inertial pos="0.0575036493913393 -0.0265307053304072 0.00135892274814464" quat="1.0 0.0 0.0 0.0" mass="0.432944265521347" diaginertia="0.00025 0.00033 0.00041" />
                          <geom name="{name}-link19_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-link19.STL" class="{childclass}-collision" />
                          <geom name="{name}-link19_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-link19.STL" class="{childclass}-visual" />
                          <body name="{name}-link20" pos="0.066 -0.02725 -0.022" quat="1.0 0.0 0.0 0.0">
                            <joint name="{name}-joint20" type="hinge" ref="0.0" class="{childclass}-motor" range="-10 10" axis="0 0 -1" />
                            <inertial pos="0.0782374674886106 2.27688611456334E-05 0.0203907673011835" quat="1.0 0.0 0.0 0.0" mass="0.471847835335862" diaginertia="0.0004 0.00043 0.00064" />
                            <geom name="{name}-link20_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-link20.STL" class="{childclass}-collision" />
                            <geom name="{name}-link20_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-link20.STL" class="{childclass}-visual" />
                            <body name="{name}-link21" pos="0.12737 0.024896 0.021756" quat="1.0 0.0 0.0 0.0">
                              <joint name="{name}-joint21" type="slide" ref="0.0" class="{childclass}-motor" range="0 0.044" axis="0 1 0" />
                              <inertial pos="-0.000355222592142046 -0.0078270385094939 -0.00298831676915345" quat="1.0 0.0 0.0 0.0" mass="0.0647981743053201" diaginertia="2e-05 3e-05 3e-05" />
                              <geom name="{name}-link21_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-link21.STL" class="{childclass}-collision" />
                              <geom name="{name}-link21_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-link21.STL" class="{childclass}-visual" />
                            </body>
                            <body name="{name}-link22" pos="0.12737 -0.0249 0.021756" quat="1.0 0.0 0.0 0.0">
                              <joint name="{name}-joint22" type="slide" ref="0.0" class="{childclass}-motor" range="0 0.044" axis="0 -1 0" />
                              <inertial pos="-0.000355223470277055 0.00782768741060944 0.00242005652832467" quat="1.0 0.0 0.0 0.0" mass="0.0647981725781687" diaginertia="2e-05 3e-05 3e-05" />
                              <geom name="{name}-link22_collision" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-link22.STL" class="{childclass}-collision" />
                              <geom name="{name}-link22_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="{name}-" type="mesh" mesh="{name}-link22.STL" class="{childclass}-visual" />
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
        <motor name="{name}-joint9_ctrl" joint="{name}-joint9" class="{childclass}-motor" />
        <motor name="{name}-joint10_ctrl" joint="{name}-joint10" class="{childclass}-motor" />
        <motor name="{name}-joint11_ctrl" joint="{name}-joint11" class="{childclass}-motor" />
        <motor name="{name}-joint12_ctrl" joint="{name}-joint12" class="{childclass}-motor" />
        <motor name="{name}-joint13_ctrl" joint="{name}-joint13" class="{childclass}-motor" />
        <motor name="{name}-joint14_ctrl" joint="{name}-joint14" class="{childclass}-motor" />
        <motor name="{name}-joint15_ctrl" joint="{name}-joint15" class="{childclass}-motor" />
        <motor name="{name}-joint16_ctrl" joint="{name}-joint16" class="{childclass}-motor" />
        <motor name="{name}-joint17_ctrl" joint="{name}-joint17" class="{childclass}-motor" />
        <motor name="{name}-joint18_ctrl" joint="{name}-joint18" class="{childclass}-motor" />
        <motor name="{name}-joint19_ctrl" joint="{name}-joint19" class="{childclass}-motor" />
        <motor name="{name}-joint20_ctrl" joint="{name}-joint20" class="{childclass}-motor" />
        <motor name="{name}-joint21_ctrl" joint="{name}-joint21" class="{childclass}-motor" />
        <motor name="{name}-joint22_ctrl" joint="{name}-joint22" class="{childclass}-motor" />
      </actuator>

      <contact>
        <exclude body1="{name}-base_link" body2="{name}-link1" />
        <exclude body1="{name}-link1" body2="{name}-link2" />
        <exclude body1="{name}-link2" body2="{name}-link3" />
        <exclude body1="{name}-link3" body2="{name}-link4" />
        <exclude body1="{name}-link2" body2="{name}-link5" />
        <exclude body1="{name}-link5" body2="{name}-link6" />
        <exclude body1="{name}-link6" body2="{name}-link7" />
        <exclude body1="{name}-link7" body2="{name}-link8" />
        <exclude body1="{name}-link8" body2="{name}-link9" />
        <exclude body1="{name}-link9" body2="{name}-link10" />
        <exclude body1="{name}-link10" body2="{name}-link11" />
        <exclude body1="{name}-link11" body2="{name}-link12" />
        <exclude body1="{name}-link11" body2="{name}-link13" />
        <exclude body1="{name}-link2" body2="{name}-link14" />
        <exclude body1="{name}-link14" body2="{name}-link15" />
        <exclude body1="{name}-link15" body2="{name}-link16" />
        <exclude body1="{name}-link16" body2="{name}-link17" />
        <exclude body1="{name}-link17" body2="{name}-link18" />
        <exclude body1="{name}-link18" body2="{name}-link19" />
        <exclude body1="{name}-link19" body2="{name}-link20" />
        <exclude body1="{name}-link20" body2="{name}-link21" />
        <exclude body1="{name}-link20" body2="{name}-link22" />
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
    robot = X7s(
        name="x7s",
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
