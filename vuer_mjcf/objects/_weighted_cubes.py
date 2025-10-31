import tempfile
from pathlib import Path

from vuer_mjcf.schema import Body
from vuer_mjcf.basic_components.concrete_slab import ConcreteSlab


def create_boxes():
    # work_area = ForcePlate(
    #     name="start-area",
    #     pos=(0, 0, 0.6052),
    #     quat=(0, 1, 0, 0),
    #     type="box",
    #     size="0.2 0.2 0.005",
    #     rgba="0.7 0.7 0.0 1.0",
    # )
    table = ConcreteSlab(pos=[0, 0, 0.6], rgba="0.8 0.8 0.8 1")

    box1 = Body(
        attributes=dict(name="box-1", pos="-0.1 0 0.7"),
        _children_raw="""
            <joint type="free" name="{name}"/>
            <geom name="{name}" type="box" size="0.015 0.015 0.015" rgba="1 0 0 1" mass="1"/>
            """,
    )

    box2 = Body(
        attributes=dict(name="box-2", pos="0 0 0.7"),
        _children_raw="""
            <joint type="free" name="{name}"/>
            <geom name="{name}" type="box" size="0.015 0.015 0.015" rgba="0 1 0 1" mass="1"/>
            """,
    )

    box3 = Body(
        attributes=dict(name="box-3", pos="0.1 0 0.7"),
        _children_raw="""
            <joint type="free" name="{name}"/>
            <geom name="{name}" type="box" size="0.015 0.015 0.015" rgba="0 0 1 1" mass="1"/>
            """,
    )
    # return work_area, table, box1, box2, box3
    return table, box1, box2, box3


if __name__ == "__main__":
    from vuer_mjcf.objects._weighted_cubes import create_boxes
    from vuer_mjcf.stage_sets.default_scene import DefaultStage
    from vuer_mjcf.utils.file import Prettify

    # Create a WeightedCubes instance
    children = create_boxes()

    # Wrap in MuJoCo scene
    scene = DefaultStage(*children, model="test__weighted_cubes_scene")

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
            print(f"✓ WeightedCubes model loaded successfully!")
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
