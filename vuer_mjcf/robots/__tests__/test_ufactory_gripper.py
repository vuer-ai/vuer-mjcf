"""Test file for Ufactory Gripper robot scene"""
import tempfile
from pathlib import Path


def test_ufactory_gripper():
    from vuer_mjcf.stage_sets._floating_ufactory_gripper import make_schema

    xml_str = make_schema()
    print(f"Generated XML for Ufactory Gripper scene")
    print(xml_str)

    # Try to load into MuJoCo
    try:
        import mujoco
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print(f"✓ Ufactory Gripper model loaded successfully!")
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
    test_ufactory_gripper()
    print("\n✓ Test passed!")
