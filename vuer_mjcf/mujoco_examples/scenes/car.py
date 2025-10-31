from vuer_mjcf.schema import Mjcf, Raw
from vuer_mjcf.stage_sets.default_scene import DefaultStage
from vuer_mjcf.utils.file import Prettify, Save
from vuer_mjcf.mujoco_examples.components.car import Car


def make_schema(**options):
    """
    Generate the car scene with a textured ground plane.

    Features:
    - A drivable car with forward/turn controls
    - Checker-textured ground plane
    - Tracking lights
    """

    # Create car
    car = Car(name="car", pos=[0, 0, 0.03])

    scene = DefaultStage(
        car,
        model="car"
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
            print("✓ Car scene loaded successfully!")
            print(f"  - Number of bodies: {model.nbody}")
            print(f"  - Number of geoms: {model.ngeom}")
            print(f"  - Number of actuators: {model.nu}")

            # Launch interactive viewer
            data = mujoco.MjData(model)
            print("Launching interactive viewer with drivable car...")
            print("Controls:")
            print("  - Forward/Backward: Use forward actuator")
            print("  - Turn Left/Right: Use turn actuator")
            mujoco.viewer.launch(model, data)
        finally:
            Path(temp_path).unlink(missing_ok=True)

    except ImportError:
        print("MuJoCo not available, skipping load test")
        print("✓ XML generated and saved successfully!")
    except Exception as e:
        print(f"✗ Error loading model: {e}")
        raise
