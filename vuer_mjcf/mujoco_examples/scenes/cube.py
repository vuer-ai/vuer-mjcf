from vuer_mjcf.schema import Mjcf, Raw
from vuer_mjcf.utils.file import Prettify
from vuer_mjcf.mujoco_examples.components.rubiks_cube_core import RubiksCubeCore


def make_schema(**options):
    """
    Generate the Rubik's Cube 3x3x3 scene matching cube_3x3x3.xml.

    Features:
    - Interactive 3x3x3 Rubik's cube
    - 26 cubelets (6 faces + 12 edges + 8 corners)
    - 6 actuators for rotating each face
    - Ball joints for edges and corners
    - Hinge joints for face centers
    """

    # Preamble with compiler, options, visual, defaults, and all assets
    assets = str(Path(__file__).parent.parent / "official_xmls" / "cube" / "assets")
    preamble =f"""
  <compiler autolimits="true" texturedir="{assets}"/>

  <option timestep="0.01" integrator="implicitfast"/>

  <size memory="600K"/>

  <visual>
    <global azimuth="180" elevation="-20"/>
    <headlight ambient="0.3 0.3 0.3" diffuse="0.6 0.6 0.6" specular="0 0 0"/>
  </visual>

  <statistic meansize="0.0087" extent="0.1"/>

  <default>
    <geom mass="0.00253704"/>
    <motor ctrlrange="-0.05 0.05"/>
    <default class="cubelet">
      <joint type="ball" armature="0.0001" damping="0.0005" frictionloss="0.001"/>
      <geom type="mesh" condim="1" mesh="cubelet" euler="0 0 90"/>
    </default>
    <default class="core">
      <geom type="sphere" contype="0" conaffinity="0" group="4" size="0.01"/>
    </default>
  </default>

  <asset>
    <texture type="skybox" builtin="gradient" width="512" height="512"/>
    <texture rgb1="0 0 0" gridsize="3 4" gridlayout=".....F......" file="white.png"/>
    <texture rgb1="0 0 0" gridsize="3 4" gridlayout=".....B......" file="yellow.png"/>
    <texture rgb1="0 0 0" gridsize="3 4" gridlayout=".....D......" file="red.png"/>
    <texture rgb1="0 0 0" gridsize="3 4" gridlayout=".....U......" file="orange.png"/>
    <texture rgb1="0 0 0" gridsize="3 4" gridlayout=".....R......" file="blue.png"/>
    <texture rgb1="0 0 0" gridsize="3 4" gridlayout=".....L......" file="green.png"/>
    <texture rgb1="0 0 0" gridsize="3 4" gridlayout=".....DF....." file="red_white.png"/>
    <texture rgb1="0 0 0" gridsize="3 4" gridlayout=".....UF....." file="orange_white.png"/>
    <texture rgb1="0 0 0" gridsize="3 4" gridlayout=".....RF....." file="blue_white.png"/>
    <texture rgb1="0 0 0" gridsize="3 4" gridlayout=".....LF....." file="green_white.png"/>
    <texture rgb1="0 0 0" gridsize="3 4" gridlayout=".....DB....." file="red_yellow.png"/>
    <texture rgb1="0 0 0" gridsize="3 4" gridlayout=".....UB....." file="orange_yellow.png"/>
    <texture rgb1="0 0 0" gridsize="3 4" gridlayout=".....RB....." file="blue_yellow.png"/>
    <texture rgb1="0 0 0" gridsize="3 4" gridlayout=".....LB....." file="green_yellow.png"/>
    <texture rgb1="0 0 0" gridsize="3 4" gridlayout=".....UD....." file="orange_red.png"/>
    <texture rgb1="0 0 0" gridsize="3 4" gridlayout=".....RD....." file="blue_red.png"/>
    <texture rgb1="0 0 0" gridsize="3 4" gridlayout=".....LD....." file="green_red.png"/>
    <texture rgb1="0 0 0" gridsize="3 4" gridlayout=".....RU....." file="blue_orange.png"/>
    <texture rgb1="0 0 0" gridsize="3 4" gridlayout=".....LU....." file="green_orange.png"/>
    <texture rgb1="0 0 0" gridsize="3 4" gridlayout=".....RDF...." file="blue_red_white.png"/>
    <texture rgb1="0 0 0" gridsize="3 4" gridlayout=".....LDF...." file="green_red_white.png"/>
    <texture rgb1="0 0 0" gridsize="3 4" gridlayout=".....RUF...." file="blue_orange_white.png"/>
    <texture rgb1="0 0 0" gridsize="3 4" gridlayout=".....LUF...." file="green_orange_white.png"/>
    <texture rgb1="0 0 0" gridsize="3 4" gridlayout=".....RDB...." file="blue_red_yellow.png"/>
    <texture rgb1="0 0 0" gridsize="3 4" gridlayout=".....LDB...." file="green_red_yellow.png"/>
    <texture rgb1="0 0 0" gridsize="3 4" gridlayout=".....RUB...." file="blue_orange_yellow.png"/>
    <texture rgb1="0 0 0" gridsize="3 4" gridlayout=".....LUB...." file="green_orange_yellow.png"/>
    <material name="white" texture="white"/>
    <material name="yellow" texture="yellow"/>
    <material name="red" texture="red"/>
    <material name="orange" texture="orange"/>
    <material name="blue" texture="blue"/>
    <material name="green" texture="green"/>
    <material name="red_white" texture="red_white"/>
    <material name="orange_white" texture="orange_white"/>
    <material name="blue_white" texture="blue_white"/>
    <material name="green_white" texture="green_white"/>
    <material name="red_yellow" texture="red_yellow"/>
    <material name="orange_yellow" texture="orange_yellow"/>
    <material name="blue_yellow" texture="blue_yellow"/>
    <material name="green_yellow" texture="green_yellow"/>
    <material name="orange_red" texture="orange_red"/>
    <material name="blue_red" texture="blue_red"/>
    <material name="green_red" texture="green_red"/>
    <material name="blue_orange" texture="blue_orange"/>
    <material name="green_orange" texture="green_orange"/>
    <material name="blue_red_white" texture="blue_red_white"/>
    <material name="green_red_white" texture="green_red_white"/>
    <material name="blue_orange_white" texture="blue_orange_white"/>
    <material name="green_orange_white" texture="green_orange_white"/>
    <material name="blue_red_yellow" texture="blue_red_yellow"/>
    <material name="green_red_yellow" texture="green_red_yellow"/>
    <material name="blue_orange_yellow" texture="blue_orange_yellow"/>
    <material name="green_orange_yellow" texture="green_orange_yellow"/>
    <mesh name="cubelet" scale="1e-3 1e-3 1e-3"
          vertex="8.075   9.5    -8.075  -8.075   9.5    -8.075   8.075   9.5     8.075
                 -8.075   9.5     8.075  -9.5     8.075  -8.075  -9.5    -8.075  -8.075
                 -9.5     8.075   8.075  -9.5    -8.075   8.075   8.075  -9.5    -8.075
                  8.075  -9.5     8.075  -8.075  -9.5    -8.075  -8.075  -9.5     8.075
                  9.5     8.075   8.075   9.5    -8.075   8.075   9.5     8.075  -8.075
                  9.5    -8.075  -8.075   8.075   8.075   9.5    -8.075   8.075   9.5
                  8.075  -8.075   9.5    -8.075  -8.075   9.5     8.075  -8.075  -9.5
                 -8.075  -8.075  -9.5     8.075   8.075  -9.5    -8.075   8.075  -9.5"/>
  </asset>
    """

    # Actuators for rotating each face
    actuators = """
  <actuator>
    <motor name="red" joint="pX"/>
    <motor name="orange" joint="nX"/>
    <motor name="blue" joint="pY"/>
    <motor name="green" joint="nY"/>
    <motor name="white" joint="pZ"/>
    <motor name="yellow" joint="nZ"/>
  </actuator>
    """

    # Create the core with all cubelets
    core = RubiksCubeCore(name="core")

    # Add light
    light = Raw('<light pos="0 0 1"/>')

    # Compose scene with Mjcf
    scene = Mjcf(
        light,
        core,
        preamble=preamble,
        postamble=actuators,
        model="Cube 3x3x3"
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
            print("✓ Rubik's Cube scene loaded successfully!")
            print(f"  - Number of bodies: {model.nbody}")
            print(f"  - Number of geoms: {model.ngeom}")
            print(f"  - Number of actuators: {model.nu}")

            # Launch interactive viewer
            data = mujoco.MjData(model)
            print("Launching interactive viewer with Rubik's Cube...")
            print("Use the actuators to rotate the cube faces!")
            mujoco.viewer.launch(model, data)
        finally:
            Path(temp_path).unlink(missing_ok=True)

    except ImportError:
        print("MuJoCo not available, skipping load test")
        print("✓ XML generated and saved successfully!")
    except Exception as e:
        print(f"✗ Error loading model: {e}")
        raise