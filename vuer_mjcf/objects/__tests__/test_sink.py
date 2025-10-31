"""Test file for Sink object"""
import tempfile
from pathlib import Path


def test_sink():
    from vuer_mjcf.objects.sink import KitchenSink
    from vuer_mjcf.stage_sets.default_scene import DefaultStage
    from vuer_mjcf.utils.file import Prettify

    # Create a KitchenSink instance
    obj = KitchenSink(name="test_sink", pos=[0, 0, 1])

    # Wrap in MuJoCo scene
    scene = DefaultStage(obj, model="test_sink_scene")

    # Generate XML
    xml_str = scene._xml | Prettify()

    print(f"Generated XML:\n{xml_str}")

    # Try to load into MuJoCo
    try:
        import mujoco
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print(f"✓ KitchenSink model loaded successfully!")
            print(f"  - Number of bodies: {model.nbody}")
            print(f"  - Number of geoms: {model.ngeom}")
            pass  # Test passed
        finally:
            Path(temp_path).unlink(missing_ok=True)

    except ImportError:
        print("MuJoCo not available, skipping load test")
        print("✓ XML generated successfully!")
        pass  # Test passed
    except Exception as e:
        print(f"✗ Error loading model: {e}")
        assert False, f"Failed to load model into MuJoCo: {e}"


if __name__ == "__main__":
    test_sink()
    print("\n✓ Test passed!")
