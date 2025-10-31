import tempfile
from pathlib import Path

from vuer_mjcf.schema import Composite


class MuJoCoRope(Composite):
    rgba = "0.7 0 0 1"
    mass = "0.001"
    geom_size = ".004"
    damping = ".015"
    condim = "1"
    twist = "1e7"
    bend = "3e5"
    vmax = "0.005"

    _attributes = {
        "prefix": "rope_",
        "type": "cable",
        "curve": "s",
        "count": "41 1 1",
        "size": 1,
        "initial": "free",
        "offset": "0 0 0.7",
    }

    _preamble = """
        <extension>
            <plugin plugin="mujoco.elasticity.cable"/>
        </extension>
    """

    _children_raw = """
    <plugin plugin="mujoco.elasticity.cable">
        <!--Units are in Pa (SI)-->
        <config key="twist" value="{twist}"/>
        <config key="bend" value="{bend}"/>
        <config key="vmax" value="{vmax}"/>
    </plugin>
    <joint kind="main" damping="{damping}"/>
    <geom type="capsule" size="{geom_size}" rgba="{rgba}" condim="{condim}" mass="{mass}"/>
    """


if __name__ == "__main__":
    from vuer_mjcf.objects.rope import MuJoCoRope
    from vuer_mjcf.stage_sets.default_scene import DefaultStage
    from vuer_mjcf.utils.file import Prettify

    # Create a MuJoCoRope instance
    obj = MuJoCoRope(name="test_rope")

    # Wrap in MuJoCo scene
    scene = DefaultStage(obj, model="test_rope_scene")

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
            print(f"✓ MuJoCoRope model loaded successfully!")
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
