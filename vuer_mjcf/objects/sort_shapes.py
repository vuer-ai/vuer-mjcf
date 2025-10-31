import tempfile
from pathlib import Path

from vuer_mjcf.schema import Body


class HexBlock(Body):
    assets = "hex-block"
    prefix = "hx-block"

    _attributes = {
        "name": "hex-block",
    }
    _preamble = """
    <option timestep="0.002" integrator="implicitfast"/>

    <asset>
      <texture type="2d" name="{prefix}-warm_wood" file="{assets}/warm_wood_planks.png" />
      <material name="{prefix}-warm_wood" texture="{prefix}-warm_wood" specular="0.1" shininess="0.1" texrepeat="1 1"/>

      <mesh name="{prefix}-obj" file="{assets}/hex.obj" scale="{scale} {scale} {scale}"/>
    </asset>
    """

    _children_raw = """
        <joint type="free"/>
        <geom type="mesh" mesh="{prefix}-obj" material="{prefix}-warm_wood" />
        <site name="{name}" pos="0 0 0" size="0.02" rgba="1 1 1 0"/>
    """


class TriangleBlock(Body):
    assets = "triangle-block"
    prefix = "tr-block"

    _attributes = {
        "name": "triangle-block",
    }
    _preamble = """
    <option timestep="0.002" integrator="implicitfast"/>
    
    <asset>
      <texture type="2d" name="{prefix}-warm_wood" file="{assets}/warm_wood_planks.png" />
      <material name="{prefix}-warm_wood" texture="{prefix}-warm_wood" specular="0.1" shininess="0.1" texrepeat="1 1"/>
      
      <mesh name="{prefix}-obj" file="{assets}/triangle.obj" scale="{scale} {scale} {scale}"/>
    </asset>
    """

    _children_raw = """
        <joint type="free"/>
        <geom type="mesh" mesh="{prefix}-obj" material="{prefix}-warm_wood" />
        <site name="{name}" pos="0 0 0" size="0.02" rgba="1 1 1 0"/>
    """


class SquareBlock(Body):
    assets = "square-block"
    prefix = "sq-block"

    _attributes = {
        "name": "square-block",
    }
    _preamble = """
    <option timestep="0.002" integrator="implicitfast"/>
    
    <asset>
      <texture type="2d" name="{prefix}-warm_wood" file="{assets}/warm_wood_planks.png" />
      <material name="{prefix}-warm_wood" texture="{prefix}-warm_wood" specular="0.1" shininess="0.1" texrepeat="1 1"/>
      
      <mesh name="{prefix}-obj" file="{assets}/square.obj" scale="{scale} {scale} {scale}"/>
    </asset>
    """

    _children_raw = """
        <joint type="free"/>
        <geom type="mesh" mesh="{prefix}-obj" material="{prefix}-warm_wood" />
        <site name="{name}" pos="0 0 0" size="0.02" rgba="1 1 1 0"/>
    """


class CircleBlock(Body):
    assets = "circle-block"
    prefix = "cr-block"

    _attributes = {
        "name": "circle-block",
    }
    _preamble = """
    <option timestep="0.002" integrator="implicitfast"/>
    
    <asset>
      <texture type="2d" name="{prefix}-warm_wood" file="{assets}/warm_wood_planks.png" />
      <material name="{prefix}-warm_wood" texture="{prefix}-warm_wood" specular="0.1" shininess="0.1" texrepeat="1 1"/>
      
      <mesh name="{prefix}-obj" file="{assets}/circle.obj" scale="{scale} {scale} {scale}"/>
    </asset>
    """

    _children_raw = """
        <joint type="free"/>
        <geom type="mesh" mesh="{prefix}-obj" material="{prefix}-warm_wood" />
        <site name="{name}" pos="0 0 0" size="0.02" rgba="1 1 1 0"/>
    """


class LeBox(Body):
    assets = "insertion-box"
    prefix = "i-box"

    _attributes = {
        "name": "insertion-box",
    }
    _preamble = """
    <option timestep="0.002" integrator="implicitfast" sdf_iterations="20" sdf_initpoints="40"/>
    
    <asset>
      <texture type="2d" name="{prefix}-light_wood" file="{assets}/light_wood_planks.png" />
      <material name="{prefix}-light_wood" texture="{prefix}-light_wood" specular="0.1" shininess="0.1" texrepeat="1 1"/>
      
      <mesh name="{prefix}-obj" file="{assets}/block.obj" scale="{scale} {scale} {scale}"/>
    </asset>
    """

    _children_raw = """
    <joint type="free"/>
    <geom type="sdf" mesh="{prefix}-obj" material="{prefix}-light_wood" contype="1" conaffinity="1"/>
    <site name="{name}-hexblock" pos="0.03 -0.033 0.1" size="0.02" rgba="1 1 1 0"/>
    <site name="{name}-squareblock" pos="-0.03 0.03 0.1" size="0.02" rgba="1 1 1 0"/>
    <site name="{name}-circleblock" pos="0.031 0.031 0.1" size="0.02" rgba="1 1 1 0"/>
    <site name="{name}-triangleblock" pos="-0.03 -0.033 0.1" size="0.02" rgba="1 1 1 0"/>
    """


if __name__ == "__main__":
    from vuer_mjcf.objects.sort_shapes import HexBlock, LeBox, SquareBlock, TriangleBlock, CircleBlock
    from vuer_mjcf.stage_sets.default_scene import DefaultStage
    from vuer_mjcf.utils.file import Prettify

    ibox = LeBox(attributes={"name": "insertion-box"}, assets="sort_shape", pos=[0.5, 0, 0.82], scale=0.08)
    square_block = SquareBlock(attributes={"name": "square"}, assets="sort_shape", pos=[0.3, -0.0, 0.82], scale=0.07)
    triangle_block = TriangleBlock(attributes={"name": "triangle"}, assets="sort_shape", pos=[0.4, -0.0, 0.82], scale=0.07)
    hex_block = HexBlock(attributes={"name": "hex"}, assets="sort_shape", pos=[0.3, 0.1, 0.82], scale=0.07)
    circle_block = CircleBlock(attributes={"name": "circle"}, assets="sort_shape", pos=[0.4, 0.1, 0.82], scale=0.07)

    # Wrap in MuJoCo scene
    scene = DefaultStage(ibox,
                    square_block,
                    triangle_block,
                    hex_block,
                    circle_block,
         model="test_sort_shapes_scene")

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
            print(f"✓ HexBlock model loaded successfully!")
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
