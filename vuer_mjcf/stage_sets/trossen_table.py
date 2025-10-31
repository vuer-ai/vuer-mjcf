from vuer_mjcf.basic_components.mj_ground_plane import GroundPlane
from vuer_mjcf.basic_components.concrete_slab import ConcreteSlab

from single_widow_x_arm import FloatingWidowXArm

def make_schema(mode="cameraready", robot="trossen_widow_x_arm", show_robot=True, **options):
    from vuer_mujoco.schemas.utils.file import Prettify

    ground = GroundPlane()
    table = ConcreteSlab(pos=[0, 0, 0.6], rgba="0.8 0.8 0.8 1", roughness="0.2")


    children = (ground,table,)
    scene = FloatingWidowXArm(
        # *children,
        pos=[0.5,0,0.6],
        quat=[0,0,0,-1],
        **options,
    )

    return scene._xml | Prettify()


if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    # Set correct assets path
    assets = str(Path(__file__).parent.parent.parent / "assets" / "robots")

    xml_str = make_schema(assets=assets)
    print("Generated XML for Trossen Table")

    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ Trossen Table loaded successfully!")
            print(f"  - Number of bodies: {model.nbody}")

            data = mujoco.MjData(model)
            print("Launching interactive viewer...")
            mujoco.viewer.launch(model, data)
        finally:
            Path(temp_path).unlink(missing_ok=True)
    except ImportError:
        print("MuJoCo not available")
    except Exception as e:
        print(f"✗ Error: {e}")
        raise