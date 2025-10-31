import tempfile
from pathlib import Path

import random
from typing import List
from vuer_mjcf.schema import Body


class ObjMujocoObject(Body):
    """
    Generic Objaverse asset with *multiple* visual meshes and matching textures.

    Directory layout (all inside the folder given in ``assets``) ::

        <name>_0.obj                     # visual mesh 0
        <name>_1.obj                     # visual mesh 1
        ...
        <name>_collision_0.obj           # collision meshes
        <name>_collision_1.obj
        ...
        <texture0>.png                   # optional – one per visual mesh
        <texture1>.png                   #   (may be "")
        ...

    Parameters
    ----------
    name : str
        Base filename / MJCF prefix  (e.g. ``"medieval_tub"``).
    assets : str
        Folder containing all OBJ / PNG files.
    visual_count : int
        How many ``{name}_{i}.obj`` files exist.
    textures : list[str]
        One string per *visual* mesh (same order).  Empty string ⇒ no texture.
    collision_count : int
        Number of collision OBJs.
    scale : float
        Uniform scale factor.
    randomize_colors : bool
        Random RGBA on collision geoms (handy while debugging).
    """

    _additional_children_raw = ""
    prefix="object"

    def __init__(
        self,
        name: str,
        assets: str,
        visual_count: int,
        textures: List[str],
        collision_count: int,
        scale: float = 1.0,
        randomize_colors: bool = True,
        # passthrough to Body ------------------------------------------------
        pos=None,
        quat=None,
        free=True,
        attributes=None,
        **kwargs,
    ):
        if len(textures) != visual_count:
            raise ValueError("len(textures) must equal visual_count")

        super().__init__(pos=pos, quat=quat, attributes=attributes or {}, **kwargs)
        self._attributes["name"] = name

        # ------------------------------------------------------------------ #
        # 1)  <default> + <asset>
        # ------------------------------------------------------------------ #
        visual_mesh_lines = "\n".join(
            f'    <mesh name="{name}_vis_{i}" file="{assets}/{name}_{i}.obj" scale="{scale} {scale} {scale}"/>' for i in range(visual_count)
        )

        collision_mesh_lines = "\n".join(
            f'    <mesh name="{name}_coll_{i}" file="{assets}/{name}_collision_{i}.obj" scale="{scale} {scale} {scale}"/>'
            for i in range(collision_count)
        )

        # optional textures & materials (one per non-empty texture)
        tex_and_mat_lines = []
        for i, tex in enumerate(textures):
            if tex:
                tex_and_mat_lines.append(f'    <texture type="2d" name="{name}_tex_{i}" file="{assets}/{tex}.png"/>')
                tex_and_mat_lines.append(f'    <material name="{name}_mat_{i}" texture="{name}_tex_{i}" specular="0.5" shininess="0.25"/>')
        tex_and_mat_section = "\n".join(tex_and_mat_lines)

        self._preamble = f"""
        <default>
            <default class="{name}_visual">
                <geom type="mesh" contype="0" conaffinity="0" group="2"
                      density="100" friction="0.95 0.3 0.1"
                      solref="0.001 1"  solimp="0.998 0.998 0.001"/>
            </default>
            <default class="{name}_collision">
                <geom type="mesh" group="3"
                      density="100" friction="0.95 0.3 0.1"
                      solref="0.001 1"  solimp="0.998 0.998 0.001"/>
            </default>
        </default>

        <asset>
{tex_and_mat_section}
{visual_mesh_lines}
{collision_mesh_lines}
        </asset>
        """.strip()

        # ------------------------------------------------------------------ #
        # 2)  <body>
        # ------------------------------------------------------------------ #
        visual_geom_lines = []
        for i in range(visual_count):
            if textures[i]:
                visual_geom_lines.append(f'    <geom mesh="{name}_vis_{i}" material="{name}_mat_{i}" class="{name}_visual"/>')
            else:  # no material
                visual_geom_lines.append(f'    <geom mesh="{name}_vis_{i}" class="{name}_visual"/>')
        visual_geom_section = "\n".join(visual_geom_lines)

        # collision geoms
        collision_geom_lines = []
        for i in range(collision_count):
            rgba = f"{random.random()} {random.random()} {random.random()} 1" if randomize_colors else "0.8 0.8 0.8 1"
            collision_geom_lines.append(f'    <geom mesh="{name}_coll_{i}" rgba="{rgba}" class="{name}_collision"/>')
        collision_geom_section = "\n".join(collision_geom_lines)

        free_joint = f'        <joint name="{name}_joint" type="free"/>' if free else ""

        self._children_raw = f"""
        {free_joint}
        {visual_geom_section}
        {collision_geom_section}
        {self._additional_children_raw}
        """.strip()


if __name__ == "__main__":
    from vuer_mjcf.objects.decomposed_obj import ObjMujocoObject
    from vuer_mjcf.stage_sets.default_scene import DefaultStage
    from vuer_mjcf.utils.file import Prettify

    # Create basketball hoop
    basketball_hoop = ObjMujocoObject(
        name="basketball_hoop",
        assets="basketball_hoop",
        prefix="hoop",
        visual_count=1,
        pos=[-0.7, 0, 0.85],
        quat=[0.7071068, 0.7071068, 0, 0],
        scale=0.5,
        collision_count=12,
        textures=["DefaultMaterial_baseColor"],
        free=False,
        randomize_colors=True,
        _additional_children_raw="""
        <site name="{prefix}_corner1" pos="0.425 0.45 -0.09" size="0.01" rgba="1 1 0 0"/>
        <site name="{prefix}_corner2" pos="0.605 0.45 -0.09" size="0.01" rgba="1 1 0 0"/>
        <site name="{prefix}_corner3" pos="0.605 0.45  0.09" size="0.01" rgba="1 1 0 0"/>
        <site name="{prefix}_corner4" pos="0.425 0.45  0.09" size="0.01" rgba="1 1 0 0"/>
        """
    )

    # Create basketball
    basketball = ObjMujocoObject(
        name="basketball",
        assets="basketball",
        visual_count=1,
        pos=[0, 0, 0.85],
        scale=0.0215,
        collision_count=1,
        textures=["Basketball_size6_baseColor"],
        randomize_colors=True,
        _additional_children_raw="""
        <site name="{name}" pos="0 0.07 0" size="0.0215" rgba="1 1 0 0"/>
        """
    )

    # Wrap in MuJoCo scene
    scene = DefaultStage(basketball_hoop, basketball, model="test_decomposed_obj_scene")

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
            print(f"✓ ObjMujocoObject model loaded successfully!")
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
