from pathlib import Path
from vuer_mjcf.schema import Mjcf, Raw
from vuer_mjcf.utils.file import Prettify


def make_schema(**options):
    """Generate the passive flex sphere scene."""
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

    world = Raw('''
    <light diffuse=".4 .4 .4" specular="0.1 0.1 0.1" pos="0 0 2.0" dir="0 0 -1" castshadow="false"/>
    <light directional="true" diffuse=".8 .8 .8" specular="0.2 0.2 0.2" pos="0 0 4" dir="0 0 -1"/>
    <geom name="ground" type="plane" size="0 0 1" pos="0 0 0" quat="1 0 0 0" material="matplane" condim="1"/>
    <geom type="box" pos="1.5 0 0.25" size=".5 2 .25"/>
    <geom type="box" pos="0 0 0.25" size="2 2 .05" euler="0 15 0"/>
    <flexcomp type="ellipsoid" count="8 8 8" spacing=".07 .07 .07" pos="-.5 0 1" dim="3"
              radius=".001" rgba="0 .7 .7 1" mass="5" name="slow">
      <edge equality="true"/>
      <contact selfcollide="none" internal="false" passive="true"/>
    </flexcomp>
    ''')

    return Mjcf(world, preamble=preamble, model="Full-flex sphere")._xml | Prettify()


if __name__ == "__main__":
    import tempfile, mujoco, mujoco.viewer
    xml_str = make_schema()
    with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
        f.write(xml_str)
        model = mujoco.MjModel.from_xml_path(f.name)
        mujoco.viewer.launch(model, mujoco.MjData(model))
        Path(f.name).unlink()