import tempfile
from pathlib import Path

from vuer_mjcf.schema import Body


class ConcreteSlab(Body):
    _attributes = {
        "name": "concrete-slab",
        # "childclass": "concrete-slab",
    }
    _preamble = """
    <asset>
      <material name="{name}-concrete" rgba="0.2 0.2 0.2 1" shininess="0.5"/>
    </asset>
    """
    # use {childclass} when you want to use defaults. Just {name}- if no
    # defaults are involved.
    # todo: how do I make sure it has collision? what about friction?
    _children_raw = """
    <geom name="{name}-concrete" type="box" material="{name}-concrete" size="0.7 0.7 0.05" pos="0 0 -0.05"/>
    """


if __name__ == "__main__":
    from vuer_mjcf.objects.concrete_slab import ConcreteSlab
    from vuer_mjcf.stage_sets.default_scene import DefaultStage
    from vuer_mjcf.utils.file import Prettify

    # Create a concrete slab instance
    slab = ConcreteSlab(name="test_slab", pos=[0, 0, 0])

    # Wrap in MuJoCo scene
    scene = DefaultStage(slab, model="test_concrete_slab_scene")

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
            print(f"✓ ConcreteSlab model loaded successfully!")
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
