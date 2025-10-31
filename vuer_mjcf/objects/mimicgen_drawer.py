import tempfile
from pathlib import Path

from vuer_mjcf.schema import Body


class MimicGenDrawer(Body):
    """
    This class represents a Vuer Mug SDF body instance with pre-configured
    assets and attributes. The Signed Distance Field (SDF) is computed
    only once and reused for all instances, ensuring efficient field
    computation regardless of the number of instances.
    """

    assets = "mimicgen-drawer"
    prefix = "mg-drawer"

    _attributes = {
        "name": "mimicgen-drawer",
    }

    _preamble = """
    <option timestep="0.002" integrator="implicitfast"/>
    <asset>
      <texture type="cube" name="{prefix}-MatRedWood" file="{assets}/ceramic.png" />
      <material name="{prefix}-MatRedWood" texture="{prefix}-MatRedWood" specular="0.4" shininess="0.1" texrepeat="1 1"/>
    
    </asset>
    """

    _children_raw = """
    <body>
      <body name="object" pos="0 0 0">
        <body name="base" pos="0 0 0" quat="1 0 0 0">
    
          <geom material="{prefix}-MatRedWood" pos="-0.165 0 0.0975" size="0.012 0.168 0.096" group="0" type="box" mass="0.16875"/>
          <geom material="{prefix}-MatRedWood" pos="0.165 0 0.0975" size="0.012 0.168 0.096" group="0" type="box" mass="0.16875"/>
          <geom material="{prefix}-MatRedWood" pos="0 0.138 0.093" size="0.153 0.012 0.096" group="0" type="box" mass="0.16875"/>
          <geom material="{prefix}-MatRedWood" pos="0 -0.012 0.006" size="0.153 0.153 0.021" group="0" type="box" mass="0.16875"/>
          <geom material="{prefix}-MatRedWood" pos="0 0.015 0.1815" size="0.153 0.153 0.012" group="0" type="box" mass="0.16875"/>
          <geom material="{prefix}-MatRedWood" pos="-0.153 -0.15 0.0975" size="0.018 0.018 0.096" group="0" type="box" mass="0.16875"/>
          <geom material="{prefix}-MatRedWood" pos="0.153 -0.15 0.0975" size="0.018 0.018 0.096" group="0" type="box" mass="0.16875"/>
    
          <body name="drawer_link" pos="0 -0.015 0.114">
            <inertial pos="0 0 0.525" quat="0.5 0.5 0.5 0.5" mass="26.5" diaginertia="7.01 5.81 1.28"/>
            
            <joint type="slide" range="-1.0 0" axis="0 1 0" name="goal_slidey" pos="0 0 0" damping="100.0"/>
            
            <geom material="{prefix}-MatRedWood" pos="0 -0.1425 0.0105" size="0.132 0.015 0.069" group="0" type="box" mass="0.135"/>
            <geom material="{prefix}-MatRedWood" pos="0 0.0975 0.012" size="0.1425 0.012 0.042" group="0" type="box" mass="0.135"/>
            <geom material="{prefix}-MatRedWood" pos="-0.108 0 0.0" size="0.012 0.126 0.042" group="0" type="box" mass="0.135"/>
            <geom material="{prefix}-MatRedWood" pos="0.108 0 0.0" size="0.012 0.126 0.042" group="0" type="box" mass="0.135"/>
            <geom name="drawer_bottom" material="{prefix}-MatRedWood" pos="0 0 -0.06" size="0.12 0.135 0.012" group="0" type="box" mass="0.135"/>
            
            <geom material="{prefix}-MatRedWood" euler="1.571 0 0" pos="-0.075 -0.195 0.06" size="0.0135 0.045" group="0" type="capsule" mass="0.2025"/>
            <geom material="{prefix}-MatRedWood" euler="0 1.57 0" pos="0 -0.24 0.06" size="0.0135 0.075" group="0" type="capsule" mass="0.2025"/>
            <geom material="{prefix}-MatRedWood" euler="1.57 0 0" pos="0.075 -0.195 0.06" size="0.0135 0.045" group="0" type="capsule" mass="0.2025"/>
    
          </body>
        </body>
      </body>
    
      <site name="bottom_site" pos="0 0 -0.45" rgba="0 0 0 0" size="0.0075"/>
      <site name="top_site" pos="0 0 0.45" rgba="0 0 0 0" size="0.0075"/>
      <site name="horizontal_radius_site" pos="0.45 0 0" rgba="0 0 0 0" size="0.15"/>
    </body>
    """


if __name__ == "__main__":
    from vuer_mjcf.objects.mimicgen_drawer import MimicGenDrawer
    from vuer_mjcf.stage_sets.default_scene import DefaultStage
    from vuer_mjcf.utils.file import Prettify

    # Create a MimicgenDrawer instance
    obj = MimicGenDrawer(name="test_mimicgen_drawer", pos=[0, 0, 1])

    # Wrap in MuJoCo scene
    scene = DefaultStage(obj, model="test_mimicgen_drawer_scene")

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
            print(f"✓ MimicgenDrawer model loaded successfully!")
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
