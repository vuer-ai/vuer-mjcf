import tempfile
from pathlib import Path

from vuer_mjcf.schema import Body


class MjObjTextureless(Body):
    assets = "objects"
    free = True
    _files = ["model"]

    _attributes = {
        "name": "object",
    }

    @property
    def mesh_entries(self):
        return "\n".join(
            [
                f"""
                <mesh name="{{name}}-{fname}" file="{{assets}}/{fname}.obj"/>
                """
                for fname in self._files
            ]
        )

    @property
    def geom_entries(self):
        return "\n".join(
            [
                f"""
            <geom mesh="{{name}}-{fname}" class="{{name}}-visual"/>
            """
                for fname in self._files
            ]
        )

    @property
    def _preamble(self):
        return (
            """
            <default>
                <default class="{name}-visual">
                    <geom group="2" type="mesh" contype="0" conaffinity="0"/>
                </default>
                <default class="{name}-collision">
                    <geom group="3" type="mesh"/>
                </default>
            </default>
            <asset>
            """
            + self.mesh_entries
            + """</asset>"""
        )

    @property
    def _children_raw(self):
        if self.free:
            return (
                """
                <freejoint/>
                """
                + self.geom_entries
            )
        else:
            return self.geom_entries


class MjObj(Body):
    assets = "objects"
    free = True
    geom_quat = "0 0 0.7071 0.7071"
    geom_pos = "-1.2 0.8 1.5"
    collision = True

    _attributes = {
        "name": "object",
    }
    _preamble = """
    <default>
        <default class="{name}-visual">
            <geom group="2" type="mesh" contype="0" conaffinity="0"/>
        </default>
        <default class="{name}-collision">
            <geom group="3" type="mesh"/>
        </default>
    </default>

    <asset>
        <texture type="2d" name="{name}-texture" file="{assets}/texture.png"/>
        <material name="{name}-material" texture="{name}-texture" specular="0.5" shininess="0.5"/>
        <mesh name="{name}-model" file="{assets}/model.obj"/>
    </asset>
    """

    @property
    def collision_geom(self):
        if self.collision:
            return f"<geom mesh='{self.name}-model' class='{self.name}-collision'/>"
        else:
            return ""

    @property
    def _children_raw(self):
        if self.free:
            return """
                <freejoint/>
                <geom material="{name}-material" quat="{geom_quat}" pos="{geom_pos}" mesh="{name}-model" class="{name}-visual"/>
                {collision_geom}
                """
        else:
            return """
                <geom material="{name}-material" quat="{geom_quat}" pos="{geom_pos}" mesh="{name}-model" class="{name}-visual"/>
                {collision_geom}
                """


if __name__ == "__main__":
    from vuer_mjcf.objects.mj_obj import MjObj
    from vuer_mjcf.stage_sets.default_scene import DefaultStage
    from vuer_mjcf.utils.file import Prettify

    # Create a robot_room MjObj instance
    robot_room = MjObj(
        geom_quat="0 0 0.7071 0.7071",
        geom_pos="-1.2 0.8 1.5",
        assets="45_robot_room",
        free=False,
        collision=False,
    )

    # Wrap in MuJoCo scene
    scene = DefaultStage(robot_room, model="test_mj_obj_scene")

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
            print(f"✓ MjObj model loaded successfully!")
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
