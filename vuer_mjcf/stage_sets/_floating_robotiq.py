from pathlib import Path

from vuer_mjcf.basic_components.mj_ground_plane import GroundPlane
from vuer_mjcf.robots.robotiq_2f85 import Robotiq2F85
from vuer_mjcf.robots.shadow_hand import ShadowHandRight
from vuer_mjcf.robots.ur5e import UR5e
from vuer_mjcf.schema import Body, Mjcf
from vuer_mjcf.basic_components.rigs.camera_rig import make_camera_rig

# from vuer_mjcf.tasks._camera_rig_stereo import make_stereo_camera_rig
from vuer_mjcf.basic_components.rigs.lighting_rig import make_lighting_rig


class FloatingRobotiq2f85(Mjcf):
    name = "floating-robotiq-2f85"
    _attributes = {
        "model": "dual UR5e setup",
    }

    _preamble = """
    <compiler angle="radian" autolimits="true" assetdir="{assets}" meshdir="{assets}" texturedir="{assets}"/>
    <!--<statistic center="0.2 0 0.4" extent=".65"/>-->

    <visual>
      <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
      <rgba haze="0.15 0.25 0.35 1"/>
      <global azimuth="150" elevation="-20" offwidth="1280" offheight="1024"/>
    </visual>

    <asset>
      <texture type="skybox" builtin="gradient" rgb1="0.9 0.7 0.9" rgb2="0.94 0.97 0.97"  width="512" height="3072"/>
      <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
        markrgb="0.8 0.8 0.8" width="300" height="300"/>
      <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
    </asset>
    """

    # _children_raw = """
    # <geom type="plane" size="1 1 .01" pos="0 0 1.21" rgba="1 1 1 1"/>
    # """

    def __init__(self, *_children, assets="assets", dual_gripper=False, include_ur5=False, camera_rig=None, free_joint=True, quat=None, **kwargs):
        super().__init__(*_children, assets=assets, **kwargs)
        if camera_rig is None:
            camera_rig = make_camera_rig(self._pos)
        else:
            print("Using passed in camera rig!")
        # camera_stereo_rig = make_stereo_camera_rig(self)
        light_rig = make_lighting_rig(self._pos)

        quat_input = quat if quat is not None else [0, 0, 1, 0]

        if include_ur5:
            # Create UR5 robot with Robotiq gripper
            robot = UR5e(
                name="ur5",
                assets="ur5e",
                pos=self._pos,
                quat=[1, 0, 0, 1],  # Aligned with table scene
                end_effector=Robotiq2F85(
                    assets="robotiq_2f85",
                    attributes={"name": "gripper"},
                    gripper_mass=0.05,
                    pos=[0, 0, 0],  # Position at the end of UR5
                    quat=[1, 0, 0, 1],  # Aligned with table scene
                    wrist_mount=camera_rig.wrist_camera(name="wrist"),
                    mocap_pos=f"{self._pos[0]} {self._pos[1] + 0.3} {self._pos[2] + 0.2}",  # Position mocap at robot's end effector
                ),
            )
            base_1 = FloatingBase(robot, attributes={"name": "robot-float"})
            _children = (base_1, robot.end_effector._mocaps)

            if dual_gripper:
                robot_2 = UR5e(
                    name="ur5-2",
                    assets="ur5e",
                    pos=self._pos + [0, -0.3, 0],
                    quat=[1, 0, 0, 1],  # Aligned with table scene
                    end_effector=Robotiq2F85(
                        assets="robotiq_2f85",
                        attributes={"name": "gripper-2"},
                        gripper_mass=0.05,
                        pos=[0, 0, 0],  # Position at the end of UR5
                        quat=[1, 0, 0, 1],  # Aligned with table scene
                        wrist_mount=camera_rig.wrist_camera(name="wrist_2"),
                        mocap_pos=f"{self._pos[0]} {self._pos[1] - 0.3} {self._pos[2] + 0.1}",  # Position mocap at robot's end effector
                    ),
                )
                base_2 = FloatingBase(robot_2, attributes={"name": "robot-2-float"})
                _children = (base_1, base_2, robot.end_effector._mocaps, robot_2.end_effector._mocaps)
        else:
            # Original floating gripper implementation
            gripper = Robotiq2F85(
                assets="robotiq_2f85",
                attributes={"name": "gripper"},
                gripper_mass=0.05,
                pos=self._pos + [0, 0, 0.3] if not dual_gripper else self._pos + [0, 0.15, 0.3],
                quat=quat_input,
                wrist_mount=camera_rig.wrist_camera(name="wrist"),
            )
            base_1 = FloatingBase(gripper, attributes={"name": "gripper-float"}) if free_joint else FloatingBaseXY(gripper, attributes={"name": "gripper-float"})
            _children = (base_1, gripper._mocaps)

            if dual_gripper:
                gripper_2 = Robotiq2F85(
                    assets="robotiq_2f85",
                    attributes={"name": "gripper-2"},
                    gripper_mass=0.05,
                    pos=self._pos + [0, -0.15, 0.3],
                    quat=[0, 0, 1, 0],
                    wrist_mount=camera_rig.wrist_camera(name="wrist_2"),
                )
                base_2 = FloatingBase(gripper_2, attributes={"name": "gripper-2-float"}) if free_joint else FloatingBaseXY(gripper_2, attributes={"name": "gripper-2-float"})
                _children = (base_1, base_2, gripper._mocaps, gripper_2._mocaps)

        if all(not isinstance(child, str) or "light" not in child for child in _children):
            self._children = (
                light_rig.key,
                light_rig.fill,
                light_rig.back,
                *self._children,
            )

        self._children = (
            *self._children,
            # always at the end, mocap last. - Ge
            *_children,
        )

class FloatingBase(Body):
    _attributes = {
        "name": "floating-base",
    }
    _children_raw = """
      <joint name="{name}-floating-base" type="free"/>
    """

class FloatingBaseXY(Body):
    _attributes = {
        "name": "floating-base",
    }
    _children_raw = """
      <joint name="{name}-floating-base_x" type="slide" axis="1 0 0"/>
       <joint name="{name}-floating-base_y" type="slide" axis="0 1 0"/>
    """


def make_schema(**options):
    from vuer_mujoco.schemas.utils.file import Prettify
    from pathlib import Path

    assets = str(Path(__file__).parent.parent.parent / "assets" / "robots")
    ground = GroundPlane()
    scene = FloatingRobotiq2f85(ground, assets=assets, **options)

    return scene._xml | Prettify()


if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    xml_str = make_schema(dual_gripper=True)
    print("Generated XML for Robotiq 2F85 scene")
    print(xml_str)

    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ Robotiq 2F85 scene loaded successfully!")
            print(f"  - Number of bodies: {model.nbody}")

            data = mujoco.MjData(model)
            print("Launching interactive viewer...")
            mujoco.viewer.launch(model, data)
        finally:
            Path(temp_path).unlink(missing_ok=True)
    except ImportError:
        print("MuJoCo not available")
    except Exception as e:
        print(f"✗ Error: {e}")
        raise
