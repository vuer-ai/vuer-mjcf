from pathlib import Path
from vuer_mjcf.schema import Mjcf, Raw
from vuer_mjcf.utils.file import Prettify


def make_schema(**options):
    """Generate floppy soft body with rotating cylinder."""
    assets = str(Path(__file__).parent.parent.parent / "official_xmls" / "flex" / "asset")

    preamble = f"""
  <compiler meshdir="{assets}" texturedir="{assets}" autolimits="true"/>
  <statistic meansize=".05"/>
  <option solver="CG" tolerance="1e-6" timestep=".001"/>
  <size memory="100M"/>
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
    <flexcomp type="grid" count="24 4 4" spacing=".1 .1 .1" pos=".1 0 1.5"
              radius=".0" rgba="0 .7 .7 1" name="softbody" dim="3" mass="25">
      <contact condim="3" solref="0.01 1" solimp=".95 .99 .0001" selfcollide="none"/>
      <elasticity young="5e4" damping="0.002" poisson="0.2"/>
    </flexcomp>
    <body>
      <joint name="hinge" pos="0 0 .5" axis="0 1 0" damping="50"/>
      <geom type="cylinder" size=".4" fromto="0 -.5 .5 0 .5 .5" density="300"/>
    </body>
    ''')

    postamble = '''
  <actuator>
    <motor name="cylinder" joint="hinge" gear="1 0 0 0 0 0" ctrlrange="-100 100"/>
  </actuator>
    '''

    return Mjcf(world, preamble=preamble, postamble=postamble, model="Floppy")._xml | Prettify()


if __name__ == "__main__":
    import tempfile, mujoco, mujoco.viewer
    xml_str = make_schema()
    with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
        f.write(xml_str)
        model = mujoco.MjModel.from_xml_path(f.name)
        mujoco.viewer.launch(model, mujoco.MjData(model))
        Path(f.name).unlink()