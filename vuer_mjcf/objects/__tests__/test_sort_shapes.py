"""Test file for SortShapes object"""
import tempfile
from pathlib import Path


def test_sort_shapes():
    from vuer_mjcf.objects.sort_shapes import HexBlock, LeBox, SquareBlock, TriangleBlock, CircleBlock
    from vuer_mjcf.stage_sets.default_scene import DefaultStage
    from vuer_mjcf.utils.file import Prettify

    ibox = LeBox(attributes={"name": "insertion-box"}, assets="sort_shape", pos=[0.5, 0, 0.82], scale=0.08)
    square_block = SquareBlock(attributes={"name": "square"}, assets="sort_shape", pos=[0.3, -0.0, 0.82], scale=0.07)
    triangle_block = TriangleBlock(attributes={"name": "triangle"}, assets="sort_shape", pos=[0.4, -0.0, 0.82], scale=0.07)
    hex_block = HexBlock(attributes={"name": "hex"}, assets="sort_shape", pos=[0.3, 0.1, 0.82], scale=0.07)
    circle_block = CircleBlock(attributes={"name": "circle"}, assets="sort_shape", pos=[0.4, 0.1, 0.82], scale=0.07)

    # Wrap in MuJoCo scene
    scene = DefaultStage(ibox,
                    square_block,
                    triangle_block,
                    hex_block,
                    circle_block,
         model="test_sort_shapes_scene")

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
            print(f"✓ HexBlock model loaded successfully!")
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
    test_sort_shapes()
    print("\n✓ Test passed!")
