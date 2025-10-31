from vuer_mjcf.schema import Body


class Cylinder(Body):
    radius = 0.0175
    halflength = 0.0175
    rgba = "1 0 0 0.1"

    _attributes = {
        "name": "cylinder",
    }
    _children_raw = """
    <joint type="free" name="{name}"/>
    <geom name="{name}-cylinder" type="cylinder" size="{radius} {halflength}" rgba="{rgba}" mass="0.1" condim="4" solimp="0.998 0.998 0.001" solref="0.001 1" friction="3 0.003 0.001" density="50"/>
    <site name="{name}" pos="0 0 0" size="0.01" rgba="1 0 0 0" type="sphere"/>
    """


if __name__ == "__main__":
    import tempfile
    from pathlib import Path
    from vuer_mjcf.stage_sets.default_scene import DefaultStage
    from vuer_mjcf.utils.file import Prettify

    # Create a cylinder instance
    cylinder = Cylinder(name="test_cylinder", pos=[0, 0, 1])

    # Wrap in MuJoCo scene
    scene = DefaultStage(cylinder, model="test_cylinder_scene")

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
            print(f"✓ Cylinder model loaded successfully!")
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
