from vuer_mjcf.schema import Body


class Balloon(Body):
    size = ".11 .11 .15"
    knot_size = ".02"
    knot_pos = "0 0 -.15"
    site_pos = "0 0 -.17"
    rgba = "1 0 0 0.1"

    _attributes = {
        "name": "balloon",
        "gravcomp":"7.2",
    }
    _children_raw = """
      <freejoint />
      <geom name="{name}" rgba="{rgba}" type="ellipsoid" size="{size}" density="0.167" fluidshape="ellipsoid"/>
      <geom name="{name}_knot" rgba="{rgba}" pos="{knot_pos}" size="{knot_size}" density="0.167" fluidshape="ellipsoid"/>
      <site name="{name}" rgba="{rgba}" pos="{site_pos}" size="0.01"/>
    """

class PinkBalloon(Balloon):
    size = ".11 .11 .15"
    knot_pos = "0 0 -.15"
    site_pos = "0 0 -.17"
    rgba = "1 .6 .7 1"

class BlueBalloon(Balloon):
    size = ".12 .12 .15"
    knot_pos = "0 0 -.15"
    site_pos = "0 0 -.17"
    rgba = ".3 .7 .9 1"

class GreenBalloon(Balloon):
    size = ".12 .12 .14"
    knot_pos = "0 0 -.14"
    site_pos = "0 0 -.16"
    rgba = ".4 .9 .5 1"

class OrangeBalloon(Balloon):
    size = ".12 .12 .13"
    knot_pos = "0 0 -.13"
    site_pos = "0 0 -.15"
    rgba = "1 .4 0 1"


if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    from vuer_mjcf.stage_sets.default_scene import DefaultStage
    from vuer_mjcf.utils.file import Prettify

    # Create balloon instances
    pink_balloon = PinkBalloon(name="pink_balloon", pos=[-0.3, 0, 2])
    blue_balloon = BlueBalloon(name="blue_balloon", pos=[0, 0, 2])
    green_balloon = GreenBalloon(name="green_balloon", pos=[0.3, 0, 2])
    orange_balloon = OrangeBalloon(name="orange_balloon", pos=[0.6, 0, 2])

    # Wrap in MuJoCo scene
    scene = DefaultStage(pink_balloon, blue_balloon, green_balloon, orange_balloon, model="test_balloon_scene")

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
            print("✓ Balloon model loaded successfully!")
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
