from pathlib import Path
from vuer_mjcf.schema import Mjcf, Raw
from vuer_mjcf.utils.file import Prettify


def make_schema(**options):
    """Generate pulley with cable and cylinders."""
    assets = str(Path(__file__).parent.parent.parent / "official_xmls" / "flex" / "asset")

    preamble = f"""
  <compiler meshdir="{assets}" texturedir="{assets}"/>
  <statistic center="0 -.3 1" extent="2.5" meansize=".05"/>
  <option>
    <flag gravity="disable"/>
  </option>
  <visual>
    <rgba haze="0.15 0.25 0.35 1"/>
    <quality shadowsize="4096"/>
    <map stiffness="700" shadowscale="0.5" fogstart="1" fogend="15" zfar="40" haze="1"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="512"/>
    <texture name="texplane" type="2d" builtin="checker" rgb1=".2 .3 .4" rgb2=".1 0.15 0.2"
      width="512" height="512" mark="cross" markrgb=".8 .8 .8"/>
    <material name="matplane" reflectance="0.3" texture="texplane" texrepeat="10 10" texuniform="true"/>
    <texture name="texcyl" type="2d" builtin="checker" rgb1=".3 .3 .3" rgb2=".15 0.15 0.15"
      width="512" height="512" mark="cross" markrgb=".22 .22 .22"/>
    <material name="matcyl" reflectance="0.3" texture="texcyl" texrepeat="3 3" texuniform="true"/>
  </asset>
    """

    world = Raw('''
    <light diffuse=".4 .4 .4" specular="0.1 0.1 0.1" pos="0 0 2.0" dir="0 0 -1" castshadow="false"/>
    <light directional="true" diffuse=".8 .8 .8" specular="0.2 0.2 0.2" pos="0 0 4" dir="0 0 -1"/>
    <geom name="ground" type="plane" size="0 0 1" pos="0 0 0" quat="1 0 0 0" material="matplane" condim="1"/>
    <flexcomp name="cable" type="circle" count="40 1 1" spacing=".15 1 1" dim="1"
              radius="0.02" pos="0 0 1" rgba="1 0 0 1">
      <edge equality="true"/>
    </flexcomp>
    <body name="left" pos="-.7 0 1">
      <joint name="expand_left" type="slide" axis="-1 0 0" range="0 2" damping="20"/>
      <joint name="rotate" type="hinge" axis="0 0 1"/>
      <geom type="cylinder" size=".2 .2" density="1" material="matcyl"/>
    </body>
    <body name="right" pos=".7 0 1">
      <joint name="expand_right" type="slide" axis="1 0 0" range="0 2" damping="20"/>
      <joint type="hinge" axis="0 0 1"/>
      <geom type="cylinder" size=".2 .2" density="1" material="matcyl"/>
    </body>
    ''')

    postamble = '''
   <actuator>
    <position name="expand_left" joint="expand_left" ctrlrange="0 2" kp="100"/>
    <position name="expand_right" joint="expand_right" ctrlrange="0 2" kp="100"/>
  </actuator>
  <equality>
    <connect body1="cable_0" anchor="0 0 0" body2="left"/>
    <connect body1="cable_20" anchor="0 0 0" body2="right"/>
  </equality>
    '''

    return Mjcf(world, preamble=preamble, postamble=postamble, model="Pulley")._xml | Prettify()


if __name__ == "__main__":
    import tempfile, mujoco, mujoco.viewer
    xml_str = make_schema()
    with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
        f.write(xml_str)
        model = mujoco.MjModel.from_xml_path(f.name)
        mujoco.viewer.launch(model, mujoco.MjData(model))
        Path(f.name).unlink()