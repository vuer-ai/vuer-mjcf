import tempfile
from pathlib import Path

from vuer_mjcf.schema import Body


class BigymPlate(Body):
    """
    This class represents a Vuer Mug SDF body instance with pre-configured
    assets and attributes. The Signed Distance Field (SDF) is computed
    only once and reused for all instances, ensuring efficient field
    computation regardless of the number of instances.
    """

    assets = "bigym-plate"

    _attributes = {
        "name": "bigym-plate",
    }

    _preamble = """
    <asset>
      <material name="{name}-plate" specular="0.5" shininess="0.25"/>
      <mesh name="{name}-plate_01" file="{assets}/plate_01.obj"/>
      <mesh name="{name}-plate_01_collision" file="{assets}/plate_01_collision.obj"/>
    </asset>
    
    <default>
      <default class="{name}-plate">
        <default class="{name}-visual">
          <geom type="mesh" mass="0.4" contype="0" conaffinity="0" group="2" euler="1.5708 0 0"/>
        </default>
        <default class="{name}-collision">
          <geom type="mesh" mass="0" group="3" euler="1.5708 0 0" solimp=".95 .99 0.001" solref="0.004 1"/>
        </default>
      </default>
    </default>
    """

    _children_raw = """
    <body name="{name}-plate" childclass="{name}-plate">
      <geom name="{name}-mesh" material="{name}-plate" mesh="{name}-plate_01" class="{name}-visual"/>
      <geom name="{name}-collider" mesh="{name}-plate_01_collision" class="{name}-collision"/>
    </body>
    """


if __name__ == "__main__":
    from vuer_mjcf.objects.bigym_plate import BigymPlate
    from vuer_mjcf.stage_sets.default_scene import DefaultStage
    from vuer_mjcf.utils.file import Prettify

    # Create a BigymPlate instance
    obj = BigymPlate(name="test_bigym_plate", pos=[0, 0, 1])

    # Wrap in MuJoCo scene
    scene = DefaultStage(obj, model="test_bigym_plate_scene")

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
            print(f"✓ BigymPlate model loaded successfully!")
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
