from pathlib import Path

from vuer_mjcf.basic_components.mj_ground_plane import GroundPlane
from vuer_mjcf.schema.schema import Mjcf, Body
from vuer_mjcf.robots.tomika_gripper import TomikaGripper
from vuer_mjcf.basic_components.rigs.camera_rig import make_camera_rig
from vuer_mjcf.basic_components.rigs.lighting_rig import make_lighting_rig


class FloatingBase(Body):
    _attributes = {
        "name": "floating-base",
    }
    _children_raw = """
      <joint name="{name}-floating-base" type="free"/>
    """


class FloatingTomikaGripper(Mjcf):
    _attributes = {
        "model": "tomika gripper setup",
    }

    _preamble = """
    <compiler angle="radian" autolimits="true" assetdir="{assets}" meshdir="{assets}" texturedir="{assets}"/>

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

    def __init__(self, *_children, assets="assets", dual_gripper=False, camera_rig=None, free_joint=True, quat=None, **kwargs):
        super().__init__(*_children, assets=assets, **kwargs)

        if camera_rig is None:
            camera_rig = make_camera_rig(self._pos)
        light_rig = make_lighting_rig(self._pos)

        quat_input = quat if quat is not None else [0, 0, 1, 0]

        # Create first gripper
        gripper = TomikaGripper(
            assets="tomika_gripper",
            attributes={"name": "gripper"},
            pos=self._pos + [0, 0, 0.3] if not dual_gripper else self._pos + [0, 0.15, 0.3],
            quat=quat_input,
            mocap_pos=self._pos + [0, 0, 0.15] if not dual_gripper else self._pos + [0, 0.15, 0.15],
            wrist_mount=camera_rig.wrist_camera(name="wrist"),
        )
        base_1 = FloatingBase(gripper, attributes={"name": "gripper-float"}) if free_joint else FloatingBase(gripper, attributes={"name": "gripper-float"})
        _children = (base_1, gripper._mocaps)

        if dual_gripper:
            gripper_2 = TomikaGripper(
                assets="tomika_gripper",
                attributes={"name": "gripper-2"},
                pos=self._pos + [0, -0.15, 0.3],
                quat=[0, 0, 1, 0],
                mocap_pos=self._pos + [0, -0.15, 0.15],
                wrist_mount=camera_rig.wrist_camera(name="wrist_2"),
            )
            base_2 = FloatingBase(gripper_2, attributes={"name": "gripper-2-float"}) if free_joint else FloatingBase(gripper_2, attributes={"name": "gripper-2-float"})
            _children = (base_1, base_2, gripper._mocaps, gripper_2._mocaps)

        self._children = (
            light_rig.key,
            light_rig.fill,
            light_rig.back,
            *self._children,
            *_children,
        )


def make_schema(**options):
    from vuer_mujoco.schemas.utils.file import Prettify
    from pathlib import Path

    assets = str(Path(__file__).parent.parent.parent / "assets" / "robots")
    ground = GroundPlane()
    scene = FloatingTomikaGripper(ground, assets=assets, **options)

    return scene._xml | Prettify()


if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    xml_str = make_schema()
    print("Generated XML for Tomika Gripper scene")
    print(xml_str)

    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ Tomika Gripper scene loaded successfully!")
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
