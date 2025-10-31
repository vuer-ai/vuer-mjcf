from pathlib import Path
from vuer_mjcf.schema import Mjcf, Raw
from vuer_mjcf.utils.file import Prettify


def make_schema(**options):
    """
    Generate the radial flex sphere scene matching sphere_radial.xml.

    Features:
    - Ellipsoid-based flexcomp with radial DOF
    - Attached to a body with freejoint
    - Edge constraints with custom solimp/solref
    - Angled ramp for rolling
    """

    assets = str(Path(__file__).parent.parent.parent / "official_xmls" / "flex" / "asset")

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

    ground_and_lights = Raw('''
    <light diffuse=".4 .4 .4" specular="0.1 0.1 0.1" pos="0 0 2.0" dir="0 0 -1" castshadow="false"/>
    <light directional="true" diffuse=".8 .8 .8" specular="0.2 0.2 0.2" pos="0 0 4" dir="0 0 -1"/>
    <geom name="ground" type="plane" size="0 0 1" pos="0 0 0" quat="1 0 0 0" material="matplane" condim="1"/>
    <geom type="box" pos="1.5 0 0.25" size=".5 2 .25"/>
    <geom type="box" pos="0 0 0.25" size="2 2 .05" euler="0 15 0"/>
    ''')

    flex_body = Raw('''
    <body name="body" pos="-.5 0 1">
      <freejoint/>
      <geom size=".1" contype="0" conaffinity="0" group="4"/>
      <flexcomp type="ellipsoid" count="8 8 8" spacing=".07 .07 .07" dim="3"
                radius=".001" rgba="0 .7 .7 1" mass="5" name="radial" dof="radial">
        <edge equality="true" solimp="0 0.9 0.01" solref=".02 1"/>
        <contact selfcollide="none" internal="false"/>
      </flexcomp>
    </body>
    ''')

    scene = Mjcf(
        ground_and_lights,
        flex_body,
        preamble=preamble,
        model="Radial flex sphere"
    )

    return scene._xml | Prettify()


if __name__ == "__main__":
    import tempfile

    xml_str = make_schema()
    print(f"Generated XML:\n{xml_str}")

    try:
        import mujoco
        import mujoco.viewer
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_str)
            temp_path = f.name

        try:
            model = mujoco.MjModel.from_xml_path(temp_path)
            print("✓ Radial flex sphere scene loaded successfully!")
            data = mujoco.MjData(model)
            mujoco.viewer.launch(model, data)
        finally:
            Path(temp_path).unlink(missing_ok=True)
    except ImportError:
        print("MuJoCo not available")
    except Exception as e:
        print(f"✗ Error: {e}")
        raise