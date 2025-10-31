import os

from vuer_mjcf.schema.schema import Body, Mjcf, MocapBody
from vuer_mjcf.schema.base import Xml
from utils.transform import compose
# from vuer_mjcf.basic_components.rigs.camera_rig import make_camera_rig
# from vuer_mjcf.basic_components.rigs.lighting_rig import make_lighting_rig
# from vuer_mjcf.se3.se3_mujoco import Vector3, WXYZ

class X7(MocapBody):
    """
    This is x7 from ARX
    """

    assets: str = "x7"
    end_effector: Xml = None
    _attributes = {
        "name": "x7",
        "childclass": "x7",
    }

    def __init__(self, *_children, end_effector: Xml = None, mocap_pos=None, **kwargs):
        if not kwargs.get("quat"):
            kwargs["quat"] = [0, 1, 0, 0]
            quat = [1, 0, 0, 0] # because mocap points are calculated in global frame
        else:
            quat = kwargs["quat"]
        super().__init__(*_children, **kwargs)
        self.end_effector = end_effector
        if end_effector:
            self._children = self._children + (end_effector,)

        # 0.5114, 0.37265
        # [0.36575, 0, 0.36065]
        if mocap_pos is None:
            self.mocap_pos = compose(self._pos, quat, [0.455, 0, 0.36065])
        else:
            self.mocap_pos = mocap_pos
        # self._children = self._children + (self.mocap_pos,)

        self.pos_head = compose(self._pos, quat, [0.174415, 0.02, 0.820811])
        self.pos_right_ee = compose(self._pos, quat, [0.547985, -0.15151, 0.415311])
        self.pos_left_ee = compose(self._pos, quat, [0.547985, 0.249994, 0.415311])
        self.mocap_quat = "1 0 0 0"

        values = self._format_dict()
        self._mocaps = self._mocaps_body.format(**values)


    _mocaps_body = """

    <body mocap="true" name="{name}-mocap-head" pos="{pos_head}">
      <site name="{name}-mocap-site-3" size=".025" type="sphere" rgba="0.13 0.3 0.2 1"/>
    </body>
    <body mocap="true" name="{name}-mocap-right" pos="{pos_right_ee}">
      <site name="{name}-mocap-site-2" size=".025" type="sphere" rgba="0.13 0.3 0.2 1"/>
    </body>
        <body mocap="true" name="{name}-mocap-left" pos="{pos_left_ee}">
      <site name="{name}-mocap-site-1" size=".025" type="sphere" rgba="0.13 0.3 0.2 1"/>
    </body>
    """

    _mocaps_equality = """
    <equality>
        <weld body1="{name}-head_link2_1" body2="{name}-mocap-head"/>
        <weld body1="{name}-catch_2_1" body2="{name}-mocap-right"/>
        <weld body1="{name}-catch_2_2" body2="{name}-mocap-left"/>

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
            <geom material="visualgeom" contype="0" conaffinity="0" group="2" />
          </default>
          <default class="{childclass}-collision">
            <geom material="{name}-collision_material" condim="3" contype="0" conaffinity="1" priority="1" group="1" solref="0.005 1" friction="1 0.01 0.01" />
            <equality solimp="0.99 0.999 1e-05" solref="0.005 1" />
          </default>
        </default>
      </default>
    
      <compiler angle="radian" />
    
      <asset>
        <material name="{name}-silver" rgba="0.700 0.700 0.700 1.000" />
        <material name="{name}-default_material" rgba="0.7 0.7 0.7 1" />
        <material name="{name}-collision_material" rgba="1.0 0.28 0.1 0.9" />
        <mesh name="{name}.stl" file="{assets}/base_link.stl" scale="0.001 0.001 0.001"/>
        <mesh name="{name}-body_link2.stl" file="{assets}/body_link2.stl" scale="0.001 0.001 0.001"/>
        <mesh name="{name}-head_link1.stl" file="{assets}/head_link1.stl" scale="0.001 0.001 0.001"/>
        <mesh name="{name}-head_link2.stl" file="{assets}/head_link2.stl" scale="0.001 0.001 0.001"/>
        <mesh name="{name}-arm_r_link1.stl" file="{assets}/arm_r_link1.stl" scale="0.001 0.001 0.001"/>
        <mesh name="{name}-arm_r_link2.stl" file="{assets}/arm_r_link2.stl" scale="0.001 0.001 0.001"/>
        <mesh name="{name}-arm_r_link3.stl" file="{assets}/arm_r_link3.stl" scale="0.001 0.001 0.001"/>
        <mesh name="{name}-arm_r_link4.stl" file="{assets}/arm_r_link4.stl" scale="0.001 0.001 0.001"/>
        <mesh name="{name}-arm_r_link5.stl" file="{assets}/arm_r_link5.stl" scale="0.001 0.001 0.001"/>
        <mesh name="{name}-arm_r_link6.stl" file="{assets}/arm_r_link6.stl" scale="0.001 0.001 0.001"/>
        <mesh name="{name}-arm_r_link7.stl" file="{assets}/arm_r_link7.stl" scale="0.001 0.001 0.001"/>
        <mesh name="{name}-catch_1_1.stl" file="{assets}/catch_1_1.stl" scale="0.001 0.001 0.001"/>
        <mesh name="{name}-catch_2_1.stl" file="{assets}/catch_2_1.stl" scale="0.001 0.001 0.001"/>
        <mesh name="{name}-arm_l_link1.stl" file="{assets}/arm_l_link1.stl" scale="0.001 0.001 0.001"/>
        <mesh name="{name}-arm_l_link2.stl" file="{assets}/arm_l_link2.stl" scale="0.001 0.001 0.001"/>
        <mesh name="{name}-arm_l_link3.stl" file="{assets}/arm_l_link3.stl" scale="0.001 0.001 0.001"/>
        <mesh name="{name}-arm_l_link4.stl" file="{assets}/arm_l_link4.stl" scale="0.001 0.001 0.001"/>
        <mesh name="{name}-arm_l_link5.stl" file="{assets}/arm_l_link5.stl" scale="0.001 0.001 0.001"/>
        <mesh name="{name}-arm_l_link6.stl" file="{assets}/arm_l_link6.stl" scale="0.001 0.001 0.001"/>
        <mesh name="{name}-arm_l_link7.stl" file="{assets}/arm_l_link7.stl" scale="0.001 0.001 0.001"/>
        <mesh name="{name}-catch_1_2.stl" file="{assets}/catch_1_2.stl" scale="0.001 0.001 0.001"/>
        <mesh name="{name}-catch_2_2.stl" file="{assets}/catch_2_2.stl" scale="0.001 0.001 0.001"/>
        <mesh name="{name}-wheel_2.stl" file="{assets}/wheel_2.stl" scale="0.001 0.001 0.001"/>
        <mesh name="{name}-wheel_1.stl" file="{assets}/wheel_1.stl" scale="0.001 0.001 0.001"/>
        <mesh name="{name}-wheel_3.stl" file="{assets}/wheel_3.stl" scale="0.001 0.001 0.001"/>
      </asset>
    """

    template = """
      <body {attributes}>
        <joint name="{name}-floating_base" />
        <inertial pos="-0.02788627568581226 -0.031293293601240654 -0.2137244378001211" quat="1.0 0.0 0.0 0.0" mass="68.02485174331227" diaginertia="7.896909 8.057484 2.250946" />
        <geom name="{name}_collision" pos="-0.015256844136058456 -0.1573132267869905 0.09318884834810495" quat="1.0 1.7649593006597641e-15 4.388645681776631e-30 -1.829007396268449e-15" type="mesh" mesh="{name}.stl"  class="{childclass}-collision" />
        <geom name="{name}_visual" pos="-0.015256844136058456 -0.1573132267869905 0.09318884834810495" quat="1.0 1.7649593006597641e-15 4.388645681776631e-30 -1.829007396268449e-15" material="{name}-silver" type="mesh" mesh="{name}.stl"  class="{childclass}-visual" />
        <body name="{name}-body_link2_1" pos="-0.006085 0.035008 -0.229811" quat="1.0 0.0 0.0 0.0">
          <joint name="{name}-body_joint_1" type="slide" ref="0.0" class="{childclass}-motor" range="-0.3 0.42" axis="0.0 -0.0 1.0" />
          <inertial pos="0.05424383454957651 -0.06956807233610031 -0.48433314722625964" quat="1.0 0.0 0.0 0.0" mass="24.603257212325776" diaginertia="0.156979 0.478969 0.578528" />
          <geom name="{name}-body_link2_1_collision" pos="-0.009171844136311141 -0.19232122678699098 0.323403945243197" quat="1.0 1.7649593006597641e-15 4.388645681776631e-30 -1.829007396268449e-15" type="mesh" mesh="{name}-body_link2.stl"  class="{childclass}-collision" />
          <geom name="{name}-body_link2_1_visual" pos="-0.009171844136311141 -0.19232122678699098 0.323403945243197" quat="1.0 1.7649593006597641e-15 4.388645681776631e-30 -1.829007396268449e-15" material="{name}-silver" type="mesh" mesh="{name}-body_link2.stl"  class="{childclass}-visual" />
          <body name="{name}-head_link1_1" pos="0.1805 -0.069781 -0.532" quat="1.0 0.0 0.0 0.0">
            <joint name="{name}-head_joint_1" type="hinge" ref="0.0" class="{childclass}-motor" range="-1.570796 1.570796" axis="0.0 -0.0 1.0" />
            <inertial pos="3.2551294518179397e-07 -0.00038379909113506194 -0.04083809461872978" quat="1.0 0.0 0.0 0.0" mass="1.1685832509612815" diaginertia="0.000744 0.000728 0.000505" />
            <geom name="{name}-head_link1_1_collision" pos="-0.18967184413630755 -0.122540226786993 0.8525419264502125" quat="1.0 2.004560761990327e-15 4.5420585157620636e-30 -1.8290073962684495e-15" type="mesh" mesh="{name}-head_link1.stl"  class="{childclass}-collision" />
            <geom name="{name}-head_link1_1_visual" pos="-0.18967184413630755 -0.122540226786993 0.8525419264502125" quat="1.0 2.004560761990327e-15 4.5420585157620636e-30 -1.8290073962684495e-15" material="{name}-silver" type="mesh" mesh="{name}-head_link1.stl"  class="{childclass}-visual" />
            <body name="{name}-head_link2_1" pos="0.0 0.025 -0.049" quat="1.0 0.0 0.0 0.0">
              <joint name="{name}-head_joint_2" type="hinge" ref="0.0" class="{childclass}-motor" range="-0.785398 0.785398" axis="0.0 1.0 0.0" />
              <inertial pos="0.008891517012994815 -0.02138240004232105 -0.07362536398787023" quat="1.0 0.0 0.0 0.0" mass="1.7036703045112362" diaginertia="0.006557 0.005513 0.008156" />
              <geom name="{name}-head_link2_1_collision" pos="-0.18967184413619376 -0.14754022678699194 0.9015419264501422" quat="1.0 2.0045607619903164e-15 -1.433979735290661e-16 -1.829007396268448e-15" type="mesh" mesh="{name}-head_link2.stl"  class="{childclass}-collision" />
              <geom name="{name}-head_link2_1_visual" pos="-0.18967184413619376 -0.14754022678699194 0.9015419264501422" quat="1.0 2.0045607619903164e-15 -1.433979735290661e-16 -1.829007396268448e-15" material="{name}-silver" type="mesh" mesh="{name}-head_link2.stl"  class="{childclass}-visual" />
            </body>
          </body>
          <body name="{name}-arm_r_link1_1" pos="0.1805 0.0735 -0.484" quat="1.0 0.0 0.0 0.0">
            <joint name="{name}-arm_joint_r_1" type="hinge" ref="0.0" class="{childclass}-motor" range="-0.523599 3.141593" axis="0.0 1.0 0.0" />
            <inertial pos="0.022412358358475548 0.024326137468245163 -1.5165199018785103e-07" quat="1.0 0.0 0.0 0.0" mass="0.37634949195895756" diaginertia="0.000339 0.000241 0.00038" />
            <geom name="{name}-arm_r_link1_1_collision" pos="-0.2559596201375548 -0.39320934163869536 1.02378595447682" quat="1.0 1.7649593006597641e-15 4.388645681776631e-30 -1.829007396268449e-15" type="mesh" mesh="{name}-arm_r_link1.stl"  class="{childclass}-collision" />
            <geom name="{name}-arm_r_link1_1_visual" pos="-0.2559596201375548 -0.39320934163869536 1.02378595447682" quat="1.0 1.7649593006597641e-15 4.388645681776631e-30 -1.829007396268449e-15" material="{name}-silver" type="mesh" mesh="{name}-arm_r_link1.stl"  class="{childclass}-visual" />
            <body name="{name}-arm_r_link2_1" pos="0.02725 0.0585 0.0" quat="1.0 0.0 0.0 0.0">
              <joint name="{name}-arm_joint_r_2" type="hinge" ref="0.0" class="{childclass}-motor" range="-1.570796 0.436332" axis="1.0 -0.0 -0.0" />
              <inertial pos="-0.023480503352369397 0.005705988652512234 0.023495564095083576" quat="1.0 0.0 0.0 0.0" mass="3.0700063054295343" diaginertia="0.005141 0.005172 0.003088" />
              <geom name="{name}-arm_r_link2_1_collision" pos="-0.21692184413630944 -0.3243212267869908 0.8074039452431967" quat="1.0 1.0006693441364403e-15 5.832087983681149e-30 -1.8845185474997073e-15" type="mesh" mesh="{name}-arm_r_link2.stl"  class="{childclass}-collision" />
              <geom name="{name}-arm_r_link2_1_visual" pos="-0.21692184413630944 -0.3243212267869908 0.8074039452431967" quat="1.0 1.0006693441364403e-15 5.832087983681149e-30 -1.8845185474997073e-15" material="{name}-silver" type="mesh" mesh="{name}-arm_r_link2.stl"  class="{childclass}-visual" />
              <body name="{name}-arm_r_link3_1" pos="-0.02725 0.0 0.0865" quat="1.0 0.0 0.0 0.0">
                <joint name="{name}-arm_joint_r_3" type="hinge" ref="0.0" class="{childclass}-motor" range="-1.570796 1.570796" axis="0.0 -0.0 1.0" />
                <inertial pos="-0.0019947308643903927 -0.0006814522943049683 0.1368879719819266" quat="1.0 0.0 0.0 0.0" mass="2.981863793105914" diaginertia="0.019419 0.019484 0.003344" />
                <geom name="{name}-arm_r_link3_1_collision" pos="-0.18967184413630786 -0.3243212267869912 0.7209039452431967" quat="1.0 2.6611827819075117e-15 1.1102230246251607e-15 -1.8845185474997057e-15" type="mesh" mesh="{name}-arm_r_link3.stl"  class="{childclass}-collision" />
                <geom name="{name}-arm_r_link3_1_visual" pos="-0.18967184413630786 -0.3243212267869912 0.7209039452431967" quat="1.0 2.6611827819075117e-15 1.1102230246251607e-15 -1.8845185474997057e-15" material="{name}-silver" type="mesh" mesh="{name}-arm_r_link3.stl"  class="{childclass}-visual" />
                <body name="{name}-arm_r_link4_1" pos="0.0 -0.02725 0.2165" quat="1.0 0.0 0.0 0.0">
                  <joint name="{name}-arm_joint_r_4" type="hinge" ref="0.0" class="{childclass}-motor" range="-1.570796 0.785398" axis="0.0 1.0 0.0" />
                  <inertial pos="0.06071898868694006 0.025423510668492383 6.854336575257136e-05" quat="1.0 0.0 0.0 0.0" mass="1.3116533194741544" diaginertia="0.001041 0.001167 0.001616" />
                  <geom name="{name}-arm_r_link4_1_collision" pos="-0.19049604503707818 -0.29517659560118414 0.5057543340961231" quat="1.0 2.6611827819075117e-15 1.538372173456118e-15 -5.52250917949517e-16" type="mesh" mesh="{name}-arm_r_link4.stl"  class="{childclass}-collision" />
                  <geom name="{name}-arm_r_link4_1_visual" pos="-0.19049604503707818 -0.29517659560118414 0.5057543340961231" quat="1.0 2.6611827819075117e-15 1.538372173456118e-15 -5.52250917949517e-16" material="{name}-silver" type="mesh" mesh="{name}-arm_r_link4.stl"  class="{childclass}-visual" />
                  <body name="{name}-arm_r_link5_1" pos="0.0935 0.02725 0.0" quat="1.0 0.0 0.0 0.0">
                    <joint name="{name}-arm_joint_r_5" type="hinge" ref="0.0" class="{childclass}-motor" range="-2.792527 2.792527" axis="1.0 -0.0 -0.0" />
                    <inertial pos="0.08339346922205759 -0.000980944991483601 -3.569533188296781e-05" quat="1.0 0.0 0.0 0.0" mass="1.3925397947387612" diaginertia="0.000669 0.001508 0.001551" />
                    <geom name="{name}-arm_r_link5_1_collision" pos="-0.2831718441363083 -0.3243212267869934 0.5044039452432006" quat="1.0 2.9906427336549495e-15 1.5383721734561181e-15 -3.4665863575905526e-15" type="mesh" mesh="{name}-arm_r_link5.stl"  class="{childclass}-collision" />
                    <geom name="{name}-arm_r_link5_1_visual" pos="-0.2831718441363083 -0.3243212267869934 0.5044039452432006" quat="1.0 2.9906427336549495e-15 1.5383721734561181e-15 -3.4665863575905526e-15" material="{name}-silver" type="mesh" mesh="{name}-arm_r_link5.stl"  class="{childclass}-visual" />
                    <body name="{name}-arm_r_link6_1" pos="0.0955 -0.02725 0.0" quat="1.0 0.0 0.0 0.0">
                      <joint name="{name}-arm_joint_r_6" type="hinge" ref="0.0" class="{childclass}-motor" range="-1.570796 0.785398" axis="-0.0 1.0 0.0" />
                      <inertial pos="0.056354003284026954 0.026892753329476937 -0.0015318308576494633" quat="1.0 0.0 0.0 0.0" mass="1.192468848199776" diaginertia="0.000735 0.00096 0.001201" />
                      <geom name="{name}-arm_r_link6_1_collision" pos="-0.378671844136308 -0.29707122678699516 0.5044039452432009" quat="1.0 2.9906427336549333e-15 3.4376411145233294e-15 -1.7457406694215533e-15" type="mesh" mesh="{name}-arm_r_link6.stl"  class="{childclass}-collision" />
                      <geom name="{name}-arm_r_link6_1_visual" pos="-0.378671844136308 -0.29707122678699516 0.5044039452432009" quat="1.0 2.9906427336549333e-15 3.4376411145233294e-15 -1.7457406694215533e-15" material="{name}-silver" type="mesh" mesh="{name}-arm_r_link6.stl"  class="{childclass}-visual" />
                      <body name="{name}-arm_r_link7_1" pos="0.066 0.02725 0.022" quat="1.0 0.0 0.0 0.0">
                        <joint name="{name}-arm_joint_r_7" type="hinge" ref="0.0" class="{childclass}-motor" range="-1.570796 1.570796" axis="0.0 -0.0 1.0" />
                        <inertial pos="0.07446127977197714 -8.750794004427909e-05 -0.020217378023051347" quat="1.0 0.0 0.0 0.0" mass="1.238313218461711" diaginertia="0.00087 0.001078 0.001374" />
                        <geom name="{name}-arm_r_link7_1_collision" pos="-0.44467184413630756 -0.3243212267869946 0.4824039452432018" quat="1.0 3.0513780315628706e-15 -8.326672684688495e-17 -1.745740669421564e-15" type="mesh" mesh="{name}-arm_r_link7.stl"  class="{childclass}-collision" />
                        <geom name="{name}-arm_r_link7_1_visual" pos="-0.44467184413630756 -0.3243212267869946 0.4824039452432018" quat="1.0 3.0513780315628706e-15 -8.326672684688495e-17 -1.745740669421564e-15" material="{name}-silver" type="mesh" mesh="{name}-arm_r_link7.stl"  class="{childclass}-visual" />
                        <body name="{name}-catch_1_1" pos="0.11857 0.085002 -0.0265" quat="1.0 0.0 0.0 0.0">
                          <joint name="{name}-catch_joint_r_1" type="slide" ref="0.0" class="{childclass}-motor" range="-0.04765 -0.003172" axis="0.0 1.0 0.0" />
                          <inertial pos="0.016354221810850333 -0.06789088940201943 0.003427702773795116" quat="1.0 0.0 0.0 0.0" mass="0.22819516637084067" diaginertia="7.9e-05 0.00014 0.000104" />
                          <geom name="{name}-catch_1_1_collision" pos="-0.547985 -0.25201 0.415311" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-catch_1_1.stl"  class="{childclass}-collision" />
                          <geom name="{name}-catch_1_1_visual" pos="-0.547985 -0.25201 0.415311" quat="1.0 0.0 0.0 0.0" material="{name}-silver" type="mesh" mesh="{name}-catch_1_1.stl"  class="{childclass}-visual" />
                        </body>
                        <body name="{name}-catch_2_1" pos="0.11857 -0.084998 -0.0265" quat="1.0 0.0 0.0 0.0">
                          <joint name="{name}-catch_joint_r_2" type="slide" ref="0.0" class="{childclass}-motor" range="0.003172 0.04765" axis="0.0 1.0 0.0" />
                          <inertial pos="0.018360095385717035 0.06860801547457226 0.007749721032955903" quat="1.0 0.0 0.0 0.0" mass="0.20305384054458703" diaginertia="7.3e-05 0.00013 9.4e-05" />
                          <geom name="{name}-catch_2_1_collision" pos="-0.547985 -0.08201 0.415311" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-catch_2_1.stl"  class="{childclass}-collision" />
                          <geom name="{name}-catch_2_1_visual" pos="-0.547985 -0.08201 0.415311" quat="1.0 0.0 0.0 0.0" material="{name}-silver" type="mesh" mesh="{name}-catch_2_1.stl"  class="{childclass}-visual" />
                        </body>
                      </body>
                    </body>
                  </body>
                </body>
              </body>
            </body>
          </body>
          <body name="{name}-arm_l_link1_1" pos="0.1805 -0.2135 -0.484" quat="1.0 0.0 0.0 0.0">
            <joint name="{name}-arm_joint_l_1" type="hinge" ref="0.0" class="{childclass}-motor" range="-3.141593 0.523599" axis="0.0 1.0 0.0" />
            <inertial pos="0.022412358358537027 -0.02432567247619527 -1.5165202593703242e-07" quat="1.0 0.0 0.0 0.0" mass="0.3763494919589559" diaginertia="0.000339 0.000241 0.00038" />
            <geom name="{name}-arm_l_link1_1_collision" pos="-0.2559596201374901 0.3932098066307419 1.0237859544767858" quat="1.8375891793576184e-15 -1.0 1.662473942574675e-15 1.9968371199901748e-16" type="mesh" mesh="{name}-arm_l_link1.stl"  class="{childclass}-collision" />
            <geom name="{name}-arm_l_link1_1_visual" pos="-0.2559596201374901 0.3932098066307419 1.0237859544767858" quat="1.8375891793576184e-15 -1.0 1.662473942574675e-15 1.9968371199901748e-16" material="{name}-silver" type="mesh" mesh="{name}-arm_l_link1.stl"  class="{childclass}-visual" />
            <body name="{name}-arm_l_link2_1" pos="0.02725 -0.0585 0.0" quat="1.0 0.0 0.0 0.0">
              <joint name="{name}-arm_joint_l_2" type="hinge" ref="0.0" class="{childclass}-motor" range="-0.436332 1.570796" axis="1.0 -0.0 -0.0" />
              <inertial pos="-0.023480503352307003 -0.00570552366046273 0.02349556417614007" quat="1.0 0.0 0.0 0.0" mass="3.0700063054295326" diaginertia="0.005141 0.005172 0.003088" />
              <geom name="{name}-arm_l_link2_1_collision" pos="-0.21692184413624516 0.32432169177903564 0.807403945243161" quat="2.947812203982775e-15 -1.0 1.4126737620340148e-15 1.9968371199901448e-16" type="mesh" mesh="{name}-arm_l_link2.stl"  class="{childclass}-collision" />
              <geom name="{name}-arm_l_link2_1_visual" pos="-0.21692184413624516 0.32432169177903564 0.807403945243161" quat="2.947812203982775e-15 -1.0 1.4126737620340148e-15 1.9968371199901448e-16" material="{name}-silver" type="mesh" mesh="{name}-arm_l_link2.stl"  class="{childclass}-visual" />
              <body name="{name}-arm_l_link3_1" pos="-0.02725 0.0 0.0865" quat="1.0 0.0 0.0 0.0">
                <joint name="{name}-arm_joint_l_3" type="hinge" ref="0.0" class="{childclass}-motor" range="-1.570796 1.570796" axis="-0.0 0.0 -1.0" />
                <inertial pos="-0.0019947278011221703 0.0006819209913284108 0.13688794355607903" quat="1.0 0.0 0.0 0.0" mass="2.9818626930617" diaginertia="0.019419 0.019484 0.003344" />
                <geom name="{name}-arm_l_link3_1_collision" pos="-0.18967184413624447 0.3243216917790366 0.720903945243162" quat="7.273661547324637e-16 -1.0 1.4126737620340122e-15 1.4432899320127094e-15" type="mesh" mesh="{name}-arm_l_link3.stl"  class="{childclass}-collision" />
                <geom name="{name}-arm_l_link3_1_visual" pos="-0.18967184413624447 0.3243216917790366 0.720903945243162" quat="7.273661547324637e-16 -1.0 1.4126737620340122e-15 1.4432899320127094e-15" material="{name}-silver" type="mesh" mesh="{name}-arm_l_link3.stl"  class="{childclass}-visual" />
                <body name="{name}-arm_l_link4_1" pos="0.0 0.02725 0.2165" quat="1.0 0.0 0.0 0.0">
                  <joint name="{name}-arm_joint_l_4" type="hinge" ref="0.0" class="{childclass}-motor" range="-0.785398 1.570796" axis="-0.0 -1.0 -0.0" />
                  <inertial pos="0.06071898868700076 -0.025423045676445905 6.854336571421316e-05" quat="1.0 0.0 0.0 0.0" mass="1.3116533194741542" diaginertia="0.001041 0.001167 0.001616" />
                  <geom name="{name}-arm_l_link4_1_collision" pos="-0.19049604503701328 0.2951770605932289 0.5057543340960878" quat="7.273661547324659e-16 -1.0 2.8837192696623486e-15 1.4920576997812724e-15" type="mesh" mesh="{name}-arm_l_link4.stl"  class="{childclass}-collision" />
                  <geom name="{name}-arm_l_link4_1_visual" pos="-0.19049604503701328 0.2951770605932289 0.5057543340960878" quat="7.273661547324659e-16 -1.0 2.8837192696623486e-15 1.4920576997812724e-15" material="{name}-silver" type="mesh" mesh="{name}-arm_l_link4.stl"  class="{childclass}-visual" />
                  <body name="{name}-arm_l_link5_1" pos="0.0935 -0.02725 0.0" quat="1.0 0.0 0.0 0.0">
                    <joint name="{name}-arm_joint_l_5" type="hinge" ref="0.0" class="{childclass}-motor" range="-2.792527 2.792527" axis="1.0 -0.0 -0.0" />
                    <inertial pos="0.08339346922211793 0.0009814099835294132 -3.5695331919993745e-05" quat="1.0 0.0 0.0 0.0" mass="1.392539794738761" diaginertia="0.000669 0.001508 0.001551" />
                    <geom name="{name}-arm_l_link5_1_collision" pos="-0.2831718441362437 0.3243216917790372 0.5044039452431649" quat="7.273661547324616e-16 -1.0 -3.0616169978691146e-17 1.4920576997812604e-15" type="mesh" mesh="{name}-arm_l_link5.stl"  class="{childclass}-collision" />
                    <geom name="{name}-arm_l_link5_1_visual" pos="-0.2831718441362437 0.3243216917790372 0.5044039452431649" quat="7.273661547324616e-16 -1.0 -3.0616169978691146e-17 1.4920576997812604e-15" material="{name}-silver" type="mesh" mesh="{name}-arm_l_link5.stl"  class="{childclass}-visual" />
                    <body name="{name}-arm_l_link6_1" pos="0.0955 0.02725 0.0" quat="1.0 0.0 0.0 0.0">
                      <joint name="{name}-arm_joint_l_6" type="hinge" ref="0.0" class="{childclass}-motor" range="-0.785398 1.570796" axis="-0.0 -1.0 -0.0" />
                      <inertial pos="0.056354003284060594 -0.026892288120110075 -0.0015318308576687256" quat="1.0 0.0 0.0 0.0" mass="1.1924688481985686" diaginertia="0.000735 0.00096 0.001201" />
                      <geom name="{name}-arm_l_link6_1_collision" pos="-0.3786718441362456 0.29707169177903814 0.5044039452431677" quat="1.837589179357618e-15 -1.0 -3.0616169978690474e-17 3.756830234103052e-15" type="mesh" mesh="{name}-arm_l_link6.stl"  class="{childclass}-collision" />
                      <geom name="{name}-arm_l_link6_1_visual" pos="-0.3786718441362456 0.29707169177903814 0.5044039452431677" quat="1.837589179357618e-15 -1.0 -3.0616169978690474e-17 3.756830234103052e-15" material="{name}-silver" type="mesh" mesh="{name}-arm_l_link6.stl"  class="{childclass}-visual" />
                      <body name="{name}-arm_l_link7_1" pos="0.066 -0.02725 0.022" quat="1.0 0.0 0.0 0.0">
                        <joint name="{name}-arm_joint_l_7" type="hinge" ref="0.0" class="{childclass}-motor" range="-1.570796 1.570796" axis="-0.0 0.0 -1.0" />
                        <inertial pos="0.07446127977687811 8.795383301993254e-05 -0.020217378023171473" quat="1.0 0.0 0.0 0.0" mass="1.2383132186826822" diaginertia="0.00087 0.001078 0.001374" />
                        <geom name="{name}-arm_l_link7_1_collision" pos="-0.44467184413624505 0.32432169177903625 0.482403945243167" quat="9.494107596574928e-16 -1.0 -3.061616997868734e-17 -9.714451465469919e-17" type="mesh" mesh="{name}-arm_l_link7.stl"  class="{childclass}-collision" />
                        <geom name="{name}-arm_l_link7_1_visual" pos="-0.44467184413624505 0.32432169177903625 0.482403945243167" quat="9.494107596574928e-16 -1.0 -3.061616997868734e-17 -9.714451465469919e-17" material="{name}-silver" type="mesh" mesh="{name}-arm_l_link7.stl"  class="{childclass}-visual" />
                        <body name="{name}-catch_1_2" pos="0.11857 0.084998 -0.0265" quat="1.0 0.0 0.0 0.0">
                          <joint name="{name}-catch_joint_l_1" type="slide" ref="0.0" class="{childclass}-motor" range="0.003172 0.04765" axis="0.0 -1.0 -0.0" />
                          <inertial pos="0.016354221810908398 -0.0678905382585942 0.0034277027737359966" quat="1.0 0.0 0.0 0.0" mass="0.22819516637084067" diaginertia="7.9e-05 0.00014 0.000104" />
                          <geom name="{name}-catch_1_2_collision" pos="-0.547985 0.151994 0.415311" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-catch_1_2.stl"  class="{childclass}-collision" />
                          <geom name="{name}-catch_1_2_visual" pos="-0.547985 0.151994 0.415311" quat="1.0 0.0 0.0 0.0" material="{name}-silver" type="mesh" mesh="{name}-catch_1_2.stl"  class="{childclass}-visual" />
                        </body>
                        <body name="{name}-catch_2_2" pos="0.11857 -0.085002 -0.0265" quat="1.0 0.0 0.0 0.0">
                          <joint name="{name}-catch_joint_l_2" type="slide" ref="0.0" class="{childclass}-motor" range="-0.04765 -0.003172" axis="0.0 -1.0 -0.0" />
                          <inertial pos="0.018360095385775987 0.0686083666179989 0.007749721032946355" quat="1.0 0.0 0.0 0.0" mass="0.20305384054458703" diaginertia="7.3e-05 0.00013 9.4e-05" />
                          <geom name="{name}-catch_2_2_collision" pos="-0.547985 0.321994 0.415311" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-catch_2_2.stl"  class="{childclass}-collision" />
                          <geom name="{name}-catch_2_2_visual" pos="-0.547985 0.321994 0.415311" quat="1.0 0.0 0.0 0.0" material="{name}-silver" type="mesh" mesh="{name}-catch_2_2.stl"  class="{childclass}-visual" />
                        </body>
                      </body>
                    </body>
                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>
        <body name="{name}-wheel_2" pos="0.175463 0.280847 -0.026811" quat="1.0 0.0 0.0 0.0">
          <joint name="{name}-wheel2" type="hinge" ref="0.0" class="{childclass}-motor" axis="0.5 0.866025 0.0" />
          <inertial pos="0.0013051432894030657 0.0022591910259763814 -4.439392070759718e-07" quat="1.0 0.0 0.0 0.0" mass="4.248031072877931" diaginertia="0.008031 0.010834 0.006629" />
          <geom name="{name}-wheel_2_collision" pos="-0.175463 -0.280847 0.026811" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-wheel_2.stl"  class="{childclass}-collision" />
          <geom name="{name}-wheel_2_visual" pos="-0.175463 -0.280847 0.026811" quat="1.0 0.0 0.0 0.0" material="{name}-silver" type="mesh" mesh="{name}-wheel_2.stl"  class="{childclass}-visual" />
        </body>
        <body name="{name}-wheel_1" pos="-0.371585 -0.034992 -0.026811" quat="1.0 0.0 0.0 0.0">
          <joint name="{name}-wheel1" type="hinge" ref="0.0" class="{childclass}-motor" axis="-1.0 0.0 0.0" />
          <inertial pos="-0.002608882124192413 3.476519027864611e-07 -4.439392085470173e-07" quat="1.0 0.0 0.0 0.0" mass="4.248031072877931" diaginertia="0.012235 0.006629 0.006629" />
          <geom name="{name}-wheel_1_collision" pos="0.371585 0.034992 0.026811" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-wheel_1.stl"  class="{childclass}-collision" />
          <geom name="{name}-wheel_1_visual" pos="0.371585 0.034992 0.026811" quat="1.0 0.0 0.0 0.0" material="{name}-silver" type="mesh" mesh="{name}-wheel_1.stl"  class="{childclass}-visual" />
        </body>
        <body name="{name}-wheel_3" pos="0.175463 -0.35083 -0.026811" quat="1.0 0.0 0.0 0.0">
          <joint name="{name}-wheel3" type="hinge" ref="0.0" class="{childclass}-motor" axis="0.5 -0.866025 -0.0" />
          <inertial pos="0.0013049438335925867 -0.002259841189761247 -4.4393920997642944e-07" quat="1.0 0.0 0.0 0.0" mass="4.248031072877931" diaginertia="0.008031 0.010834 0.006629" />
          <geom name="{name}-wheel_3_collision" pos="-0.175463 0.35083 0.026811" quat="1.0 0.0 0.0 0.0" type="mesh" mesh="{name}-wheel_3.stl"  class="{childclass}-collision" />
          <geom name="{name}-wheel_3_visual" pos="-0.175463 0.35083 0.026811" quat="1.0 0.0 0.0 0.0" material="{name}-silver" type="mesh" mesh="{name}-wheel_3.stl"  class="{childclass}-visual" />
      </body>
      <site name="{name}-world_site" pos="0 0 0" quat="1 0 0 0" />
      <camera name="{name}-front_camera" mode="track" fovy="90.0" quat="4.329780281177467e-17 4.329780281177466e-17 0.7071067811865475 0.7071067811865476" pos="0.0 2.0 0.5" />
      <camera name="{name}-side_camera" mode="track" fovy="90.0" quat="-0.5 -0.4999999999999999 0.5 0.5000000000000001" pos="-2.0 0.0 0.5" />
    </body>
    """

    _postamble = """
              <actuator>
            <motor name="{name}-body_joint_1_ctrl" joint="{name}-body_joint_1" class="{childclass}-motor" />
            <motor name="{name}-head_joint_1_ctrl" joint="{name}-head_joint_1" class="{childclass}-motor" />
            <motor name="{name}-head_joint_2_ctrl" joint="{name}-head_joint_2" class="{childclass}-motor" />
            <motor name="{name}-arm_joint_r_1_ctrl" joint="{name}-arm_joint_r_1" class="{childclass}-motor" />
            <motor name="{name}-arm_joint_r_2_ctrl" joint="{name}-arm_joint_r_2" class="{childclass}-motor" />
            <motor name="{name}-arm_joint_r_3_ctrl" joint="{name}-arm_joint_r_3" class="{childclass}-motor" />
            <motor name="{name}-arm_joint_r_4_ctrl" joint="{name}-arm_joint_r_4" class="{childclass}-motor" />
            <motor name="{name}-arm_joint_r_5_ctrl" joint="{name}-arm_joint_r_5" class="{childclass}-motor" />
            <motor name="{name}-arm_joint_r_6_ctrl" joint="{name}-arm_joint_r_6" class="{childclass}-motor" />
            <motor name="{name}-arm_joint_r_7_ctrl" joint="{name}-arm_joint_r_7" class="{childclass}-motor" />
            <motor name="{name}-catch_joint_r_1_ctrl" joint="{name}-catch_joint_r_1" class="{childclass}-motor" />
            <motor name="{name}-catch_joint_r_2_ctrl" joint="{name}-catch_joint_r_2" class="{childclass}-motor" />
            <motor name="{name}-arm_joint_l_1_ctrl" joint="{name}-arm_joint_l_1" class="{childclass}-motor" />
            <motor name="{name}-arm_joint_l_2_ctrl" joint="{name}-arm_joint_l_2" class="{childclass}-motor" />
            <motor name="{name}-arm_joint_l_3_ctrl" joint="{name}-arm_joint_l_3" class="{childclass}-motor" />
            <motor name="{name}-arm_joint_l_4_ctrl" joint="{name}-arm_joint_l_4" class="{childclass}-motor" />
            <motor name="{name}-arm_joint_l_5_ctrl" joint="{name}-arm_joint_l_5" class="{childclass}-motor" />
            <motor name="{name}-arm_joint_l_6_ctrl" joint="{name}-arm_joint_l_6" class="{childclass}-motor" />
            <motor name="{name}-arm_joint_l_7_ctrl" joint="{name}-arm_joint_l_7" class="{childclass}-motor" />
            <motor name="{name}-catch_joint_l_1_ctrl" joint="{name}-catch_joint_l_1" class="{childclass}-motor" />
            <motor name="{name}-catch_joint_l_2_ctrl" joint="{name}-catch_joint_l_2" class="{childclass}-motor" />
            <motor name="{name}-wheel2_ctrl" joint="{name}-wheel2" class="{childclass}-motor" />
            <motor name="{name}-wheel1_ctrl" joint="{name}-wheel1" class="{childclass}-motor" />
            <motor name="{name}-wheel3_ctrl" joint="{name}-wheel3" class="{childclass}-motor" />
          </actuator>
        
          <contact>
            <exclude body1="{name}" body2="{name}-body_link2_1" />
            <exclude body1="{name}-body_link2_1" body2="{name}-head_link1_1" />
            <exclude body1="{name}-head_link1_1" body2="{name}-head_link2_1" />
            <exclude body1="{name}-body_link2_1" body2="{name}-arm_r_link1_1" />
            <exclude body1="{name}-arm_r_link1_1" body2="{name}-arm_r_link2_1" />
            <exclude body1="{name}-arm_r_link2_1" body2="{name}-arm_r_link3_1" />
            <exclude body1="{name}-arm_r_link3_1" body2="{name}-arm_r_link4_1" />
            <exclude body1="{name}-arm_r_link4_1" body2="{name}-arm_r_link5_1" />
            <exclude body1="{name}-arm_r_link5_1" body2="{name}-arm_r_link6_1" />
            <exclude body1="{name}-arm_r_link6_1" body2="{name}-arm_r_link7_1" />
            <exclude body1="{name}-body_link2_1" body2="{name}-arm_l_link1_1" />
            <exclude body1="{name}-arm_l_link1_1" body2="{name}-arm_l_link2_1" />
            <exclude body1="{name}-arm_l_link2_1" body2="{name}-arm_l_link3_1" />
            <exclude body1="{name}-arm_l_link3_1" body2="{name}-arm_l_link4_1" />
            <exclude body1="{name}-arm_l_link4_1" body2="{name}-arm_l_link5_1" />
            <exclude body1="{name}-arm_l_link5_1" body2="{name}-arm_l_link6_1" />
            <exclude body1="{name}-arm_l_link6_1" body2="{name}-arm_l_link7_1" />
            <exclude body1="{name}" body2="{name}-wheel_2" />
            <exclude body1="{name}" body2="{name}-wheel_1" />
            <exclude body1="{name}" body2="{name}-wheel_3" />
            <exclude body1="{name}-arm_r_link7_1" body2="{name}-catch_1_1" />
            <exclude body1="{name}-arm_r_link7_1" body2="{name}-catch_2_1" />
            <exclude body1="{name}-arm_l_link7_1" body2="{name}-catch_1_2" />
            <exclude body1="{name}-arm_l_link7_1" body2="{name}-catch_2_2" />
          </contact>
        
          <sensor>
            <framepos name="{name}-world_site_pos" objtype="site" objname="{name}-world_site" />
            <framequat name="{name}-world_site_quat" objtype="site" objname="{name}-world_site" />
            <framelinvel name="{name}-world_site_linvel" objtype="site" objname="{name}-world_site" />
            <frameangvel name="{name}-world_site_angvel" objtype="site" objname="{name}-world_site" />
            <velocimeter name="{name}-world_site_vel" site="{name}-world_site" />
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
    robot = X7(
        name="lift",
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
