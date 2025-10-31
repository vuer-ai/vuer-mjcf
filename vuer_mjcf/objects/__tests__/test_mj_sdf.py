"""Test file for MjSDF object"""
import tempfile
from pathlib import Path


def test_mj_sdf():
    from vuer_mjcf.objects.mj_sdf import MjSDF
    from vuer_mjcf.stage_sets.default_scene import DefaultStage
    from vuer_mjcf.utils.file import Prettify

    # Create a ball sorting toy MjSDF instance
    box = MjSDF(
        pos=[0.465, 0.07, 0.82],
        quat=[1, 0, 0, 0],
        assets="ball_sorting_toy",
        geom_quat="-0.5 -0.5 0.5 0.5",
        _attributes={"name": "ball-sorting-toy"},
        scale="0.12 0.12 0.12",
        additional_children_raw="""
        <site name="hole-1" pos="0.08 0 0.05" size="0.03" rgba="1 0 0 0" type="sphere"/>
        """
    )

    # Wrap in MuJoCo scene
    scene = DefaultStage(box, model="test_mj_sdf_scene")

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
            print(f"✓ MjSDF model loaded successfully!")
            print(f"  - Number of bodies: {model.nbody}")
            print(f"  - Number of geoms: {model.ngeom}")
            pass  # Test passed
        finally:
            Path(temp_path).unlink(missing_ok=True)

    except ImportError:
        print("MuJoCo not available, skipping load test")
        print("✓ XML generated successfully!")
        pass  # Test passed
    except ValueError as e:
        if "plugin mujoco.sdf.sdflib not found" in str(e):
            print("MuJoCo SDF plugin not available, skipping load test")
            print("✓ XML generated successfully!")
            pass  # Test passed - plugin not installed is OK
        else:
            print(f"✗ Error loading model: {e}")
            assert False, f"Failed to load model into MuJoCo: {e}"
    except Exception as e:
        print(f"✗ Error loading model: {e}")
        assert False, f"Failed to load model into MuJoCo: {e}"


if __name__ == "__main__":
    test_mj_sdf()
    print("\n✓ Test passed!")
