import tempfile
from pathlib import Path

from vuer_mjcf.schema import Body


class MjSDF(Body):
    assets = "objects"
    scale = "1 1 1"
    mass = 0.1
    additional_children_raw = ""
    model = "model.obj"
    texture = "texture.png"
    geom_quat = "1 0 0 0"
    additional_geom_attributes = ""

    _attributes = {
        "name": "object",
    }
    _preamble = """
    <option timestep="0.002" integrator="implicitfast" sdf_iterations="20" sdf_initpoints="40"/>
    <extension>
      <plugin plugin="mujoco.sdf.sdflib">
        <instance name="{assets}-sdf">
          <config key="aabb" value="0"/>
        </instance>
      </plugin>
    </extension>
    
    <asset>
        <texture type="2d" name="{name}-texture" file="{assets}/{texture}"/>
        <material name="{name}-material" texture="{name}-texture" specular="0.2" shininess="0.2"/>
      
        <mesh name="{name}-model" file="{assets}/{model}" scale="{scale}">
            <plugin instance="{assets}-sdf"/>
        </mesh>
    </asset>
    """

    _children_raw = """
        <joint type="free"/>
        <geom type="sdf" name="{name}-sdf" mesh="{name}-model" quat="{geom_quat}" mass="{mass}" material="{name}-material" contype="1" conaffinity="1" {additional_geom_attributes}>
            <plugin instance="{assets}-sdf"/>
        </geom>
        {additional_children_raw}
    """


if __name__ == "__main__":
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
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print(f"✓ MjSDF model loaded successfully!")
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
    except ValueError as e:
        if "plugin mujoco.sdf.sdflib not found" in str(e):
            print("MuJoCo SDF plugin not available, skipping load test")
            print("✓ XML generated successfully!")  # plugin not installed is OK
        else:
            print(f"✗ Error loading model: {e}")
            raise
    except Exception as e:
        print(f"✗ Error loading model: {e}")
        raise
