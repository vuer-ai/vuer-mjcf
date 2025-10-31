from vuer_mjcf.robots.franka_panda import Panda
from vuer_mjcf.robots.tomika_gripper import TomikaGripper
from vuer_mjcf.schema import Mjcf, Body
from vuer_mjcf.se3.se3_mujoco import z_rot, π


class ConcreteSlab(Body):
    _attributes = {
        "name": "concrete-slab",
        # "childclass": "concrete-slab",
    }
    _preamble = """
    <asset>
      <material name="{name}-concrete" rgba="0.4 0.4 0.4 1"/>
    </asset>
    """
    # use {childclass} when you want to use defaults. Just {name}- if no
    # defaults are involved.
    # todo: how do I make sure it has collision? what about friction?
    _children_raw = """
    <geom name="{name}-concrete" type="box" material="{name}-concrete" size="0.7 0.7 0.05" pos="0 0 -0.05"/>
    """


class DualPanda(Mjcf):
    _attributes = {
        "model": "dual panda setup",
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
      <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
      <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
        markrgb="0.8 0.8 0.8" width="300" height="300"/>
      <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
    </asset>
    """

    def __init__(self, *_children, assets="../robots", **kwargs):
        super().__init__(*_children, assets=assets, **kwargs)

        left_gripper = TomikaGripper(assets="tomika_gripper", attributes={"name": "left-gripper"})
        right_gripper = TomikaGripper(assets="tomika_gripper", attributes={"name": "right-gripper"})

        self._children = (
            Panda(
                assets="franka_panda",
                pos=self._pos + [-0.25, 0.5, 0],
                quat=Panda._quat + z_rot(π / 2),
                attributes={"name": "left_panda"},
                end_effector=left_gripper,
            ),
            Panda(
                assets="franka_panda",
                pos=self._pos + [0.25, 0.5, 0],
                quat=Panda._quat + z_rot(π / 2),
                attributes={"name": "right_panda"},
                end_effector=right_gripper,
            ),
            *self._children,
            # now add the mocap points.
            left_gripper._mocaps,
            right_gripper._mocaps,
        )


def make_schema(**options):
    from vuer_mjcf.utils.file import Prettify

    scene = DualPanda(**options)
    return scene._xml | Prettify()


if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    # Set correct assets path
    assets = str(Path(__file__).parent.parent.parent / "assets" / "robots")

    xml_str = make_schema(assets=assets)
    print("Generated XML for Dual Panda")

    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ Dual Panda loaded successfully!")
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
