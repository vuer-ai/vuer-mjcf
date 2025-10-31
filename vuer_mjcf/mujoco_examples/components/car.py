from vuer_mjcf.schema import Body


class Car(Body):
    """
    A simple car with two rear wheels and a front wheel.

    Features:
    - Custom mesh chassis
    - Two motorized rear wheels (left and right)
    - Front steering wheel (sphere)
    - Tendon-based control for forward/turn motion
    - Tracking lights
    """

    _attributes = {
        "name": "car",
    }

    _preamble = """
    <asset>
        <mesh name="{name}_chasis" scale=".01 .006 .0015"
          vertex=" 9   2   0
                  -10  10  10
                   9  -2   0
                   10  3  -10
                   10 -3  -10
                  -8   10 -10
                  -10 -10  10
                  -8  -10 -10
                  -5   0   20"/>
    </asset>

    <default>
        <joint damping=".03" actuatorfrcrange="-0.5 0.5"/>
        <default class="{name}_wheel">
            <geom type="cylinder" size=".03 .01" rgba=".5 .5 1 1"/>
        </default>
        <default class="{name}_decor">
            <site type="box" rgba=".5 1 .5 1"/>
        </default>
    </default>
    """

    _children_raw = """
        <freejoint/>
        <light name="{name}_top_light" pos="0 0 2" mode="trackcom" diffuse=".4 .4 .4"/>
        <geom name="{name}_chasis" type="mesh" mesh="{name}_chasis"/>
        <geom name="{name}_front_wheel" pos=".08 0 -.015" type="sphere" size=".015" condim="1" priority="1"/>
        <light name="{name}_front_light" pos=".1 0 .02" dir="2 0 -1" diffuse="1 1 1"/>

        <body name="{name}_left_wheel" pos="-.07 .06 0" zaxis="0 1 0">
            <joint name="{name}_left"/>
            <geom class="{name}_wheel"/>
            <site class="{name}_decor" size=".006 .025 .012"/>
            <site class="{name}_decor" size=".025 .006 .012"/>
        </body>

        <body name="{name}_right_wheel" pos="-.07 -.06 0" zaxis="0 1 0">
            <joint name="{name}_right"/>
            <geom class="{name}_wheel"/>
            <site class="{name}_decor" size=".006 .025 .012"/>
            <site class="{name}_decor" size=".025 .006 .012"/>
        </body>
    """

    _postamble = """
    <tendon>
        <fixed name="{name}_forward">
            <joint joint="{name}_left" coef=".5"/>
            <joint joint="{name}_right" coef=".5"/>
        </fixed>
        <fixed name="{name}_turn">
            <joint joint="{name}_left" coef="-.5"/>
            <joint joint="{name}_right" coef=".5"/>
        </fixed>
    </tendon>

    <actuator>
        <motor name="{name}_forward" tendon="{name}_forward" ctrlrange="-1 1"/>
        <motor name="{name}_turn" tendon="{name}_turn" ctrlrange="-1 1"/>
    </actuator>

    <sensor>
        <jointactuatorfrc name="{name}_right" joint="{name}_right"/>
        <jointactuatorfrc name="{name}_left" joint="{name}_left"/>
    </sensor>
    """


if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    from vuer_mjcf.stage_sets.default_scene import DefaultStage
    from vuer_mjcf.utils.file import Prettify

    # Create a car instance
    car = Car(name="my_car", pos=[0, 0, 0.03])

    # Wrap in MuJoCo scene with ground plane
    scene = DefaultStage(car, model="test_car_scene")

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
            print("✓ Car model loaded successfully!")
            print(f"  - Number of bodies: {model.nbody}")
            print(f"  - Number of geoms: {model.ngeom}")
            print(f"  - Number of actuators: {model.nu}")

            # Launch interactive viewer
            data = mujoco.MjData(model)
            print("Launching interactive viewer...")
            print("Controls: Use actuators to drive forward/turn")
            mujoco.viewer.launch(model, data)
        finally:
            Path(temp_path).unlink(missing_ok=True)

    except ImportError:
        print("MuJoCo not available, skipping load test")
        print("✓ XML generated successfully!")
    except Exception as e:
        print(f"✗ Error loading model: {e}")
        raise
