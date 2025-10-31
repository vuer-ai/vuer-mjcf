from vuer_mjcf.schema import Body


class Ball(Body):
    size = 0.03
    rgba = "1 0 0 0.1"

    _attributes = {
        "name": "ball",
    }
    _children_raw = """
    <joint type="free" name="{name}"/>
    <geom name="{name}-sphere" type="sphere" size="{size}" rgba="{rgba}" mass="0.01" condim="4" solimp="0.998 0.998 0.001" solref="0.001 1" friction="10 0.3 0.1" density="50"/>
    <site name="{name}" pos="0 0 0" size="0.01" rgba="1 0 0 0" type="sphere"/>
    """


if __name__ == "__main__":
    import tempfile
    from pathlib import Path
    from vuer_mjcf.stage_sets.default_scene import DefaultStage
    from vuer_mjcf.utils.file import Prettify

    # Create a ball instance
    ball = Ball(name="test_ball", pos=[0, 0, 4])

    # Wrap in MuJoCo scene
    scene = DefaultStage(ball, model="test_ball_scene")

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
            print(f"✓ Ball model loaded successfully!")
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
