"""Test file for DecomposedObj object"""
import tempfile
from pathlib import Path


def test_decomposed_obj():
    from vuer_mjcf.objects.decomposed_obj import ObjMujocoObject
    from vuer_mjcf.stage_sets.default_scene import DefaultStage
    from vuer_mjcf.utils.file import Prettify

    # Create basketball hoop
    basketball_hoop = ObjMujocoObject(
        name="basketball_hoop",
        assets="basketball_hoop",
        prefix="hoop",
        visual_count=1,
        pos=[-0.7, 0, 0.85],
        quat=[0.7071068, 0.7071068, 0, 0],
        scale=0.5,
        collision_count=12,
        textures=["DefaultMaterial_baseColor"],
        free=False,
        randomize_colors=True,
        _additional_children_raw="""
        <site name="{prefix}_corner1" pos="0.425 0.45 -0.09" size="0.01" rgba="1 1 0 0"/>
        <site name="{prefix}_corner2" pos="0.605 0.45 -0.09" size="0.01" rgba="1 1 0 0"/>
        <site name="{prefix}_corner3" pos="0.605 0.45  0.09" size="0.01" rgba="1 1 0 0"/>
        <site name="{prefix}_corner4" pos="0.425 0.45  0.09" size="0.01" rgba="1 1 0 0"/>
        """
    )

    # Create basketball
    basketball = ObjMujocoObject(
        name="basketball",
        assets="basketball",
        visual_count=1,
        pos=[0, 0, 0.85],
        scale=0.0215,
        collision_count=1,
        textures=["Basketball_size6_baseColor"],
        randomize_colors=True,
        _additional_children_raw="""
        <site name="{name}" pos="0 0.07 0" size="0.0215" rgba="1 1 0 0"/>
        """
    )

    # Wrap in MuJoCo scene
    scene = DefaultStage(basketball_hoop, basketball, model="test_decomposed_obj_scene")

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
            print(f"✓ ObjMujocoObject model loaded successfully!")
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
    test_decomposed_obj()
    print("\n✓ Test passed!")
