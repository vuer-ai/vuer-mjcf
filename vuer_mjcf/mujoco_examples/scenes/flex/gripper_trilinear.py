from pathlib import Path
from vuer_mjcf.schema import Mjcf, Raw
from vuer_mjcf.utils.file import Prettify


def make_schema(**options):
    """Generate gripper with trilinear elasticity caps."""
    assets = str(Path(__file__).parent.parent.parent / "official_xmls" / "flex" / "asset")

    preamble = f"""
  <compiler meshdir="{assets}" texturedir="{assets}"/>
  <statistic meansize=".05"/>
  <option cone="elliptic" impratio="10" integrator="implicitfast"/>
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
  </asset>
    """

    world = Raw('''
    <light diffuse=".4 .4 .4" specular="0.1 0.1 0.1" pos="0 0 2.0" dir="0 0 -1" castshadow="false"/>
    <light directional="true" diffuse=".8 .8 .8" specular="0.2 0.2 0.2" pos="0 0 4" dir="0 0 -1"/>
    <geom name="ground" type="plane" size="0 0 1" pos="0 0 0" quat="1 0 0 0" material="matplane" condim="1"/>
    <body name="hand" pos="0 0 .37">
      <joint name="lift" type="slide" range="0 1"/>
      <geom type="box" size=".2 .1 .05" rgba=".2 .2 .2 1"/>
      <body name="right_gripper">
        <joint name="right" type="slide" axis="-1 0 0"/>
        <geom type="box" size=".02 .1 .15" pos=".18 0 -.2" rgba=".2 .2 .2 1"/>
        <flexcomp type="mesh" file="cap.obj" pos=".16 0 -.25" dim="3" euler="0 -90 0"
                  origin="0 0 0" radius=".001" rgba="0 .7 .7 1" mass=".5" name="left" dof="trilinear">
          <elasticity young="1e4" poisson="0.1" damping=".05"/>
          <contact selfcollide="none" internal="false" contype="2" conaffinity="2"/>
          <pin id="4 5 6 7"/>
        </flexcomp>
      </body>
      <body name="left_gripper">
        <joint name="left" type="slide" axis="1 0 0"/>
        <geom type="box" size=".02 .1 .15" pos="-.18 0 -.2" rgba=".2 .2 .2 1"/>
        <flexcomp type="mesh" file="cap.obj" pos="-.16 0 -.25" dim="3" euler="0 90 0"
                  origin="0 0 0" radius=".001" rgba="0 .7 .7 1" mass=".5" name="right" dof="trilinear">
          <elasticity young="1e4" poisson="0.1" damping=".05"/>
          <contact selfcollide="none" internal="false" contype="2" conaffinity="2"/>
          <pin id="0 1 2 3"/>
        </flexcomp>
      </body>
    </body>
    <body>
      <freejoint/>
      <geom type="box" size=".05 .1 .1" pos="0 0 .1" rgba=".5 .5 0 1" priority="1" contype="2" condim="6"/>
    </body>
    ''')

    postamble = '''
  <equality>
    <joint joint1="right" joint2="left"/>
  </equality>
  <tendon>
    <fixed name="grasp">
      <joint joint="right" coef="1"/>
      <joint joint="left" coef="1"/>
    </fixed>
  </tendon>
  <actuator>
    <position name="lift" joint="lift" kp="600" dampratio="1" ctrlrange="0 1"/>
    <position name="grasp" tendon="grasp" kp="200" dampratio="1" ctrlrange="0 1"/>
  </actuator>
    '''

    return Mjcf(world, preamble=preamble, postamble=postamble, model="Gripper")._xml | Prettify()


if __name__ == "__main__":
    import tempfile, mujoco, mujoco.viewer
    xml_str = make_schema()
    with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
        f.write(xml_str)
        model = mujoco.MjModel.from_xml_path(f.name)
        mujoco.viewer.launch(model, mujoco.MjData(model))
        Path(f.name).unlink()