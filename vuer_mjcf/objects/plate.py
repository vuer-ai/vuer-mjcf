import random
from vuer_mjcf.schema import Body

class ObjaverseMujocoPlate(Body):
    def __init__(
        self,
        name="object",
        assets="kitchen/plate",
        collision_count=32,
        visual_count=1,
        randomize_colors=True,
        scale=0.185,
        # Add these to match Body's signature
        pos=None,
        quat=None,
        attributes=None,
        **kwargs
    ):
        super().__init__(pos=pos, quat=quat, attributes=attributes or {}, **kwargs)
        self._attributes["name"] = name
        self.collision_count = collision_count

        # ------------------------------------------------------------------ #
        # 1) <asset> section – meshes, textures, materials
        # ------------------------------------------------------------------ #
        visual_meshes = ""
        for i in range(visual_count):
            visual_meshes += (
                f'<mesh name="{name}_vis_{i}" '
                f'file="{assets}/visual/model_normalized_{i}.obj" '
                f'scale="{scale} {scale} {scale}"/>\n'
            )

        collision_meshes = ""
        for i in range(collision_count):
            collision_meshes += (
                f'<mesh name="{name}_coll_{i}" '
                f'file="{assets}/collision/model_normalized_collision_{i}.obj" '
                f'scale="{scale} {scale} {scale}"/>\n'
            )

        # simple matte material (tweak as you like)
        material = f'<material name="{name}_mat" specular="0.5" shininess="0.0" rgba="0.1 0.5 0.03 1"/>'

        self._preamble = f"""
                <default>
                    <default class="{name}_visual">
                        <geom type="mesh" contype="0" conaffinity="0"
                              density="100" friction="2.0 0.3 0.1"
                              solref="0.001 1" solimp="0.998 0.998 0.001"/>
                    </default>
                    <default class="{name}_collision">
                        <geom type="mesh" group="3"
                              density="100" friction="2.0 0.3 0.1"
                              solref="0.001 1" solimp="0.998 0.998 0.001"/>
                    </default>
                </default>
                <asset>
                {visual_meshes}
                {collision_meshes}
                    <material name="{name}_mat" specular="0.5" shininess="0.0"
                              rgba="0.1 0.3 0.2 1.0"/>
                </asset>
                """.strip()

        # ------------------------------------------------------------------ #
        # 2) <body> – joint + geoms
        # ------------------------------------------------------------------ #
        # visual geoms
        visual_geoms = ""
        for i in range(visual_count):
            visual_geoms += (
                f'    <geom name="{name}_vis_{i}_mesh" mesh="{name}_vis_{i}" material="{name}_mat" '
                f'class="{name}_visual"/>\n'
            )

        # collision geoms
        collision_geoms = ""
        for i in range(collision_count):
            rgba = f"{random.random()} {random.random()} {random.random()} 1" if randomize_colors else "0.8 0.8 0.8 1"
            collision_geoms += (
                f'    <geom name="{name}_coll_{i}_mesh" mesh="{name}_coll_{i}" rgba="{rgba}" '
                f'class="{name}_collision"/>\n'
            )

        self._children_raw = f"""
                <joint name="{name}_joint" type="free"/>
                {visual_geoms}
                {collision_geoms.rstrip()}
                <site name="{name}" pos="0 0 0" size="0.01" rgba="1 1 1 0"/>
                """.strip()


if __name__ == "__main__":
    import tempfile
    from pathlib import Path
    from vuer_mjcf.stage_sets.default_scene import DefaultStage
    from vuer_mjcf.utils.file import Prettify

    # Create a ObjaverseMujoco instance
    obj = ObjaverseMujocoPlate(assets="kitchen/plate", pos=[0, -0.5, 0.95], name="plate", collision_count=32, randomize_colors=False)

    # Wrap in MuJoCo scene
    scene = DefaultStage(obj, model="test_plate_scene")

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
            print(f"✓ ObjaverseMujoco model loaded successfully!")
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
