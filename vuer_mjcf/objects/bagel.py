import tempfile
from pathlib import Path

from vuer_mjcf.schema import Body, Xml


class Bagel(Body):
    assets: str = 'bagel'
    end_effector: Xml

    def __init__(self, name: str = "bagel", **kwargs):
        super().__init__(name=name, **kwargs)

    _attributes = {
        "name": "bagel",
        "childclass": "bagel",
        "pos": "0 0 0",
        "quat": "0.707 -0.707 0 0",
    }

    _preamble = """
    <default>
      <default class="{childclass}">
        <default class="{childclass}-visual">
          <geom type="mesh" contype="0" conaffinity="0" group="2"/>
        </default>
        <default class="{childclass}-collision">
          <geom type="box" group="3" contype="1" conaffinity="1"/>
        </default>
      </default>
    </default>

    <asset>
      <mesh file="{assets}/bagel_0.obj" scale="0.05 0.05 0.05"/>
      <mesh file="{assets}/bagel_1.obj" scale="0.05 0.05 0.05"/>
    </asset>
    """

    template = """
    <body {attributes}>
      <freejoint/>
      <inertial mass="0.1" pos="0 0 0" diaginertia="0.00443333156 0.00443333156 0.0072"/>
      <body name="{name}-base">
        <inertial mass="0.1" pos="0 0 0" diaginertia="0.0102675 0.0102675 0.00666"/>
        <geom mesh="bagel_0" class="{childclass}-visual"/>
        <geom mesh="bagel_1" class="{childclass}-visual"/>
        <geom class="{childclass}-collision" mesh="bagel_0"/>
        <geom class="{childclass}-collision" mesh="bagel_1"/>
      </body>
    </body>
    """


if __name__ == "__main__":
    from vuer_mjcf.objects.bagel import Bagel
    from vuer_mjcf.stage_sets.default_scene import DefaultStage
    from vuer_mjcf.utils.file import Prettify

    # Create a Bagel instance
    obj = Bagel(name="test_bagel", pos=[0, 0, 1])

    # Wrap in MuJoCo scene
    scene = DefaultStage(obj, model="test_bagel_scene")

    # Generate XML
    xml_str = scene._xml | Prettify()

    print(f"Generated XML:\n{xml_str}")

    # Try to load into MuJoCo
    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print(f"✓ Bagel model loaded successfully!")
            print(f"  - Number of bodies: {model.nbody}")
            print(f"  - Number of geoms: {model.ngeom}")

            # Launch interactive viewer
            data = mujoco.MjData(model)
            print("Launching interactive viewer...")
            mujoco.viewer.launch(model, data)
        finally:
            Path(temp_path).unlink(missing_ok=True)

    except ImportError:
        print("MuJoCo not available, skipping load test")
        print("✓ XML generated successfully!")
    except Exception as e:
        print(f"✗ Error loading model: {e}")
        raise
