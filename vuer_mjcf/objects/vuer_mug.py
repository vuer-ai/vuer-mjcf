import tempfile
from pathlib import Path

from vuer_mjcf.schema import Body


class VuerMug(Body):
    """
    This class represents a Vuer Mug SDF body instance with pre-configured
    assets and attributes. The Signed Distance Field (SDF) is computed
    only once and reused for all instances, ensuring efficient field
    computation regardless of the number of instances.
    """

    assets = "vuer-mug"
    prefix = "v-mug"

    _attributes = {
        "name": "vuer-mug",
    }
    _preamble = """
    <option sdf_iterations="10" sdf_initpoints="20"/>
    
    <asset>
      <texture name="texspot" type="2d" file="{assets}/vuer.png"/>
      <material name="matspot" texture="texspot"/>
      <mesh name="spot" file="{assets}/mug.obj" scale="0.00933 0.0142 0.00933"/>
    </asset>
    """

    _children_raw = """
    <freejoint/>
    <geom type="sdf" name="{name}" mesh="spot" material="matspot" mass="0.075" quat="0 0 1 1" condim="4" solimp="0.95 0.99 0.001" solref="0.003 1"/>
    <site name="mug_handle_1" pos="-0.054 0 0.075" size="0.005" rgba="1 1 1 1"/>
    <site name="mug_handle_2" pos="-0.054 0 0.056" size="0.005" rgba="1 1 1 1"/>
    <site name="mug_handle_3" pos="-0.054 0 0.037" size="0.005" rgba="1 1 1 1"/>
    """


if __name__ == "__main__":
    from vuer_mjcf.objects.vuer_mug import VuerMug
    from vuer_mjcf.stage_sets.default_scene import DefaultStage
    from vuer_mjcf.utils.file import Prettify

    # Create a VuerMug instance
    obj = VuerMug(name="test_vuer_mug", pos=[0, 0, 1])

    # Wrap in MuJoCo scene
    scene = DefaultStage(obj, model="test_vuer_mug_scene")

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
            print(f"✓ VuerMug model loaded successfully!")
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
