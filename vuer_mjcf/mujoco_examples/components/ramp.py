from vuer_mjcf.schema import Body


class Ramp(Body):
    size = ".4 .2 .03"
    euler = "0 20 0"

    _attributes = {
        "name": "ramp",
    }
    _children_raw = """
      <geom name="{name}" type="box" size="{size}" pos="0 0 -.4" euler="{euler}" rgba="1 1 1 1"/>
    """


if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    from vuer_mjcf.stage_sets.default_scene import DefaultStage
    from vuer_mjcf.utils.file import Prettify

    # Create a ramp instance
    ramp = Ramp(name="test_ramp", pos=[0, 0, 0.5])

    # Wrap in MuJoCo scene
    scene = DefaultStage(ramp, model="test_ramp_scene")

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
            print("✓ Ramp model loaded successfully!")
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
