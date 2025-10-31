from vuer_mjcf.schema import Body


class PaperWeight(Body):
    mass = ".0347"
    rgba = ".8 .4 .8 1"

    _attributes = {
        "name": "weight",
    }
    _children_raw = """
      <freejoint/>
      <light pos="1 0 3" dir="-1 0 -3" mode="trackcom"/>
      <light pos="-1 0 3" dir="1 0 -3" mode="trackcom"/>
      <!-- The mass of the weight was chosen to be slightly bigger than the total buoyancy of the balloons. -->
      <geom name="weight" type="box" size=".015 .015 .015" mass="{mass}" rgba="{rgba}"/>
      <site name="{name}1" pos=" .013  .013 .013" size="0.005" rgba="{rgba}"/>
      <site name="{name}2" pos="-.013 -.013 .013" size="0.005" rgba="{rgba}"/>
    """

    _preamble = """
    <default>
    <tendon limited="true" width="0.003" rgba="1 1 1 1"/>
    <geom friction=".2"/>
  </default>
    """


if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    from vuer_mjcf.stage_sets.default_scene import DefaultStage
    from vuer_mjcf.utils.file import Prettify

    # Create a paper weight instance
    weight = PaperWeight(name="test_weight", pos=[0, 0, 1])

    # Wrap in MuJoCo scene
    scene = DefaultStage(weight, model="test_paper_weight_scene")

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
            print("✓ PaperWeight model loaded successfully!")
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
