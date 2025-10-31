import tempfile
from pathlib import Path

from vuer_mjcf.schema import Body

class TShape(Body):
    rgba = "1.0 0.1 0.1 1"

    # def __init__(self, *args, **kwargs):
    #     super().__init__(*args, **kwargs)
    #     self.vert_pos = self._pos +

    _attributes = {
        "name": "t-shape"
    }

    _preamble = """
    <asset>
      <material name="{name}-mat" rgba="{rgba}" shininess="0.5"/>
    </asset>
    """

    _children_raw = """
    <!-- Vertical bar of T -->
    <body name="{name}-body" pos="0 0 0">
        <geom name="{name}-vert" type="box" group="2" material="{name}-mat" contype="1" conaffinity="1" 
              size="0.02 0.10 0.02" pos="0 0 0.02" friction="50.0 0.005 0.0001" solref="0.005 0.01" solimp="0.95 0.99 0.001"/>
    
        <!-- Horizontal bar of T -->
        <geom name="{name}-horiz" type="box" group="2" material="{name}-mat" contype="1" conaffinity="1" 
              size="0.10 0.02 0.02" pos="0 0.08 0.02" friction="50.0 0.005 0.0001" solref="0.005 0.01" solimp="0.95 0.99 0.001"/>
              
                 <joint name="{name}-joint-slide_x" type="slide" axis="1 0 0" damping="10.0"/>
                 <joint name="{name}-joint-slide_y" type="slide" axis="0 1 0" damping="10.0"/>
                 <joint name="{name}-joint-hinge_z" type="hinge" axis="0 0 1" damping="0.5"/>
    
        <inertial pos="0 0.055 0.02" mass="1.0" diaginertia="0.0075 0.0074 0.0001"/>
        <site name="tee" pos="0.10 0.02 0.02" size="0.005" rgba="1 1 1 1"/> 
    </body>
    """

if __name__ == "__main__":
    from vuer_mjcf.objects.tshape import TShape
    from vuer_mjcf.stage_sets.default_scene import DefaultStage
    from vuer_mjcf.utils.file import Prettify

    # Create a TShape instance
    obj = TShape(name="test_tshape", pos=[0, 0, 1])

    # Wrap in MuJoCo scene
    scene = DefaultStage(obj, model="test_tshape_scene")

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
            print(f"✓ TShape model loaded successfully!")
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
