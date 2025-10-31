from vuer_mjcf.basic_components.mj_ground_plane import GroundPlane
from vuer_mjcf.basic_components.rigs.camera_rig import make_camera_rig
from vuer_mjcf.basic_components.rigs.lighting_rig import make_lighting_rig
from vuer_mjcf.robots.shadow_hand import ShadowHandRight
from vuer_mjcf.schema import Mjcf

from vuer_mjcf.robots.ur5e import UR5e

class UR5ShadowHand(Mjcf):
    """A class that combines UR5 robot with Robotiq gripper."""

    name = "ur5-shadowhand"

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
    </asset>z
    """

    def __init__(self, *_children, assets="assets", dual_robot=False, camera_rig=None, **kwargs):
        super().__init__(*_children, assets=assets, **kwargs)

        if camera_rig is None:
            camera_rig = make_camera_rig(self._pos)
        else:
            print("Using passed in camera rig!")
        light_rig = make_lighting_rig(self._pos)

        # Create UR5 robot with Robotiq gripper
        robot = UR5e(
            name="ur5",
            assets="ur5e",
            pos=self._pos,
            quat=[1, 0, 0, 1],  # Aligned with table scene
            end_effector=ShadowHandRight(
                free=False,
                assets="shadow_hand",
                # attributes={"name": "shadow_hand_left"},
                pos=[0, -0.01, -0.2],  # Position at the end of UR5
                quat=[1, 0, 0, 1],  # Aligned with table scene
                wrist_mount=camera_rig.wrist_camera(name="wrist"),
                # mocap_pos=f"{self._pos[0]} {self._pos[1] + 0.3} {self._pos[2] + 0.2}",
            ),
        )
        _children = (robot, robot.end_effector._mocaps)
        # _children = (robot,)

        if all(not isinstance(child, str) or "light" not in child for child in _children):
            self._children = (
                light_rig.key,
                light_rig.fill,
                light_rig.back,
                *self._children,
            )

        self._children = (
            *self._children,
            *_children,
        )

def make_schema(**options):
    from vuer_mujoco.schemas.utils.file import Prettify
    from pathlib import Path

    assets = str(Path(__file__).parent.parent.parent / "assets" / "robots")
    ground = GroundPlane()
    scene = UR5ShadowHand(ground, assets=assets, **options)

    return scene._xml | Prettify()

if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    xml_str = make_schema()
    print("Generated XML for UR5e scene")
    print(xml_str)

    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ UR5e scene loaded successfully!")
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