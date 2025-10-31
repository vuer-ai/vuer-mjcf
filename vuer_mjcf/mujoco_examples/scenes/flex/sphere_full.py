from pathlib import Path
from vuer_mjcf.schema import Mjcf, Raw
from vuer_mjcf.utils.file import Prettify


def make_schema(**options):
    """
    Generate the full-flex sphere scene matching sphere_full.xml.

    Features:
    - Ellipsoid-based flexcomp with 8x8x8 elements
    - Edge equality constraints
    - Angled ramp for rolling
    - Default scene lighting and ground plane
    """

    # Asset path
    assets = str(Path(__file__).parent.parent.parent / "official_xmls" / "flex" / "asset")

    # Preamble: includes scene.xml content + flex-specific options
    preamble = f"""
  <compiler meshdir="{assets}" texturedir="{assets}"/>

  <statistic meansize=".05"/>

  <option solver="CG" tolerance="1e-6" timestep=".001" integrator="implicitfast"/>

  <size memory="10M"/>

  <visual>
    <rgba haze="0.15 0.25 0.35 1"/>
    <quality shadowsize="4096"/>
    <map stiffness="500" shadowscale="0.5" fogstart="1" fogend="15" zfar="40" haze="1"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="512"/>
    <texture name="texplane" type="2d" builtin="checker" rgb1=".2 .3 .4" rgb2=".1 0.15 0.2"
      width="512" height="512" mark="cross" markrgb=".8 .8 .8"/>
    <material name="matplane" reflectance="0.3" texture="texplane" texrepeat="10 10" texuniform="true"/>
  </asset>
    """

    # World body with lights, ground, ramp, and flex sphere
    ground_and_lights = Raw('''
    <light diffuse=".4 .4 .4" specular="0.1 0.1 0.1" pos="0 0 2.0" dir="0 0 -1" castshadow="false"/>
    <light directional="true" diffuse=".8 .8 .8" specular="0.2 0.2 0.2" pos="0 0 4" dir="0 0 -1"/>
    <geom name="ground" type="plane" size="0 0 1" pos="0 0 0" quat="1 0 0 0" material="matplane" condim="1"/>
    ''')

    ramp = Raw('''
    <geom type="box" pos="1.5 0 0.25" size=".5 2 .25"/>
    <geom type="box" pos="0 0 0.25" size="2 2 .05" euler="0 15 0"/>
    ''')

    flex_sphere = Raw('''
    <flexcomp type="ellipsoid" count="8 8 8" spacing=".07 .07 .07" pos="-.5 0 1" dim="3"
              radius=".001" rgba="0 .7 .7 1" mass="5" name="slow">
      <edge equality="true"/>
      <contact selfcollide="none" internal="false"/>
    </flexcomp>
    ''')

    # Compose scene
    scene = Mjcf(
        ground_and_lights,
        ramp,
        flex_sphere,
        preamble=preamble,
        model="Full-flex sphere"
    )

    return scene._xml | Prettify()


if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    # Generate XML
    xml_str = make_schema()

    print(f"Generated XML:\n{xml_str}")

    # Try to load into MuJoCo and launch viewer
    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ Full-flex sphere scene loaded successfully!")
            print(f"  - Number of bodies: {model.nbody}")
            print(f"  - Number of geoms: {model.ngeom}")

            # Launch interactive viewer
            data = mujoco.MjData(model)
            print("Launching interactive viewer with flex sphere...")
            mujoco.viewer.launch(model, data)
        finally:
            Path(temp_path).unlink(missing_ok=True)

    except ImportError:
        print("MuJoCo not available, skipping load test")
        print("✓ XML generated successfully!")
    except Exception as e:
        print(f"✗ Error loading model: {e}")
        raise