from vuer_mjcf.schema import Mjcf, Raw
from vuer_mjcf.utils.file import Prettify
from vuer_mjcf.mujoco_examples.components.paper_weight import PaperWeight
from vuer_mjcf.mujoco_examples.components.balloon import PinkBalloon, BlueBalloon, GreenBalloon, OrangeBalloon


def make_schema(**options):
    """
    Generate the balloons scene matching balloons.xml.

    Features:
    - 4 colorful balloons with helium buoyancy (gravcomp=7.2)
    - Paper weight to hold them down with tracking lights
    - Spatial tendons connecting balloons to weight
    - Ramp for interaction
    - Ground plane with checker grid texture
    - Physics options: air density (1.204) and viscosity (1.8e-5)
    - Visual elevation setting (-10 degrees)
    """

    # Preamble with physics options, visual, defaults, and assets
    preamble = """
  <option density="1.204" viscosity="1.8e-5" integrator="implicit"/>

  <visual>
    <global elevation="-10"/>
  </visual>

  <asset>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512" rgb2="0 0 0" rgb1="1 1 1"/>
    <material name="grid" texture="grid" texrepeat="2 2" texuniform="true" reflectance=".6"/>
  </asset>
    """

    # Tendons connecting balloons to weight
    tendons = """
  <tendon>
    <spatial range="0 0.6">
      <site site="pink"/>
      <site site="weight1"/>
    </spatial>
    <spatial range="0 0.4">
      <site site="blue"/>
      <site site="weight1"/>
    </spatial>
    <spatial range="0 0.3">
      <site site="green"/>
      <site site="weight2"/>
    </spatial>
    <spatial range="0 0.5">
      <site site="orange"/>
      <site site="weight2"/>
    </spatial>
  </tendon>
    """

    # Create components
    weight = PaperWeight(name="weight", pos=[0.3, 0, 0.2])
    pink = PinkBalloon(name="pink", pos=[-0.2, 0.1, 0.2])
    blue = BlueBalloon(name="blue", pos=[0.1, 0.1, 0.2])
    green = GreenBalloon(name="green", pos=[0.1, -0.1, 0.2])
    orange = OrangeBalloon(name="orange", pos=[-0.12, -0.12, 0.2])

    # Add ground and ramp as Raw XML
    ground = Raw('<geom name="ground" type="plane" size="5 5 .05" pos="0 0 -.5" material="grid"/>')
    ramp = Raw('<geom name="ramp" type="box" size=".4 .2 .03" pos="0 0 -.4" euler="0 20 0" rgba="1 1 1 1"/>')

    # Compose scene with Mjcf
    scene = Mjcf(
        ground,
        ramp,
        weight,
        pink,
        blue,
        green,
        orange,
        preamble=preamble,
        postamble=tendons,
        model="balloons"
    )

    return scene._xml | Prettify()


if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    # Generate XML
    xml_str = make_schema()

    print(f"Generated XML:\n{xml_str}")

    # Try to load into MuJoCo and launch viewer
    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ Balloons scene loaded successfully!")
            print(f"  - Number of bodies: {model.nbody}")
            print(f"  - Number of geoms: {model.ngeom}")

            # Launch interactive viewer
            data = mujoco.MjData(model)
            print("Launching interactive viewer with balloons, ramp, and paper weight...")
            mujoco.viewer.launch(model, data)
        finally:
            Path(temp_path).unlink(missing_ok=True)

    except ImportError:
        print("MuJoCo not available, skipping load test")
        print("✓ XML generated and saved successfully!")
    except Exception as e:
        print(f"✗ Error loading model: {e}")
        raise
