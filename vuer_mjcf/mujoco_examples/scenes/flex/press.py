from pathlib import Path
from vuer_mjcf.schema import Mjcf, Raw
from vuer_mjcf.utils.file import Prettify


def make_schema(**options):
    """Generate press scene with two different stiffness soft bodies."""
    assets = str(Path(__file__).parent.parent.parent / "official_xmls" / "flex" / "asset")

    preamble = f"""
  <compiler meshdir="{assets}" texturedir="{assets}" autolimits="true"/>
  <statistic meansize=".05"/>
  <option solver="Newton" tolerance="1e-6" timestep=".001" integrator="implicitfast"/>
  <size memory="10M"/>
  <visual>
    <rgba haze="0.15 0.25 0.35 1"/>
    <quality shadowsize="4096"/>
    <map stiffness="100" shadowscale="0.5" fogstart="1" fogend="15" zfar="40" haze="1"/>
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
    <flexcomp name="A" type="grid" count="4 4 4" spacing=".2 .2 .2" pos="0 0 .5"
              radius=".005" rgba="0 .7 .7 1" dim="3" mass="5">
      <contact condim="3" solref="0.01 1" solimp=".95 .99 .0001" selfcollide="none"/>
      <edge damping="1"/>
      <elasticity young="1e4" poisson="0.4"/>
    </flexcomp>
    <flexcomp name="B" type="grid" count="4 4 4" spacing=".2 .2 .2" pos="1 0 .5"
              radius=".005" rgba="0 .7 .7 1" dim="3" mass="5">
      <contact condim="3" solref="0.01 1" solimp=".95 .99 .0001" selfcollide="none"/>
      <edge damping="1"/>
      <elasticity young="5e4" poisson="0"/>
    </flexcomp>
    <body>
      <joint name="soft" type="slide" axis="0 0 1" damping="500"/>
      <geom type="box" size=".35 .35 0.05" pos=".1 .1 1" density="300"/>
    </body>
    <body>
      <joint name="stiff" type="slide" axis="0 0 1" damping="500"/>
      <geom type="box" size=".35 .35 0.05" pos="1.1 .1 1" density="300"/>
    </body>
    ''')

    postamble = '''
  <actuator>
    <position name="soft" joint="soft" gear="-1 0 0 0 0 0" ctrlrange="-.7 .1" kp="10000"/>
    <position name="stiff" joint="stiff" gear="-1 0 0 0 0 0" ctrlrange="-.7 .1" kp="10000"/>
  </actuator>
    '''

    return Mjcf(world, preamble=preamble, postamble=postamble, model="Press")._xml | Prettify()


if __name__ == "__main__":
    import tempfile, mujoco, mujoco.viewer
    xml_str = make_schema()
    with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
        f.write(xml_str)
        model = mujoco.MjModel.from_xml_path(f.name)
        mujoco.viewer.launch(model, mujoco.MjData(model))
        Path(f.name).unlink()