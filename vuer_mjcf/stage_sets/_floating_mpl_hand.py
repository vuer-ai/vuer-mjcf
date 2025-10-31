from pathlib import Path

from vuer_mjcf.basic_components.mj_ground_plane import GroundPlane
from vuer_mjcf.schema.schema import Mjcf
from vuer_mjcf.robots.mpl_hand import MPLRight
from vuer_mjcf.basic_components.rigs.camera_rig import make_camera_rig
from vuer_mjcf.basic_components.rigs.lighting_rig import make_lighting_rig


class FloatingMPLHand(Mjcf):
    _attributes = {
        "model": "mpl hand setup",
    }

    _preamble = """
    <compiler angle="radian" autolimits="true" assetdir="{assets}" meshdir="{assets}" texturedir="{assets}"/>

    <visual>
      <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
      <rgba haze="0.15 0.25 0.35 1"/>
      <global azimuth="150" elevation="-20" offwidth="1280" offheight="1024"/>
    </visual>

    <asset>
      <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
      <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
        markrgb="0.8 0.8 0.8" width="300" height="300"/>
      <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
    </asset>
    """

    # _children_raw = """
    # <geom type="plane" size="1 1 .01" pos="0 0 1.21" rgba="1 1 1 1"/>
    # """

    def __init__(self, *_children, assets="assets", **kwargs):
        super().__init__(*_children, assets=assets, **kwargs)

        camera_rig = make_camera_rig(self._pos)
        light_rig = make_lighting_rig(self._pos)

        hand1 = MPLRight(
            assets="mpl",
            attributes={"name": "mpl_right"},
            # set this to a small number to avoid unreasonable forces.
            pos=self._pos + [0, 0, 0.3],
            # wrist_mount=camera_rig.wrist_camera,
        )

        self._children = (
            camera_rig.front_camera,
            camera_rig.top_camera,
            camera_rig.left_camera,
            camera_rig.right_camera,
            light_rig.key,
            light_rig.fill,
            light_rig.back,
            *self._children,
            hand1,
            # now add the mocap points.
            hand1._mocaps,
        )


def make_schema(**options):
    from vuer_mujoco.schemas.utils.file import Prettify
    from pathlib import Path

    assets = str(Path(__file__).parent.parent.parent / "assets" / "robots")
    ground = GroundPlane()
    scene = FloatingMPLHand(ground, assets=assets, **options)

    return scene._xml | Prettify()


if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    xml_str = make_schema()
    print("Generated XML for MPL Hand scene")
    print(xml_str)

    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ MPL Hand scene loaded successfully!")
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
