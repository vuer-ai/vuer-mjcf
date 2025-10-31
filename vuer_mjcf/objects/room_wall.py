import tempfile
from pathlib import Path

from vuer_mjcf.schema import Body


class RoomWall(Body):
    """
    A single rectangular wall panel, with an optional thicker *backing* slab
    sitting 12 cm behind it.  
    ───────────────────────────────────────────────────────────────────
    • `panel_sz = (½-width, ½-height, ½-thickness)`  –  for the visible wall  
    • `backing_sz` defaults to panel_sz with `z` = backing_thickness  
    • visual geoms are mass-0, non-colliding (group 1)  
    • collision geoms are opaque red boxes (group 3) – change as you like
    """

    # textures / materials --------------------------------------------------
    assets = "kitchen/wall"          # folder with wall textures
    prefix = "wall"          # XML-name stem
    show_texture = True

    _preamble = """
    <asset>
        <texture type="2d" name="{prefix}_tex_paint" file="{assets}/paint.png"/>
        <material name="{prefix}_paint_mat" texture="{prefix}_tex_paint" shininess="0.15"/>
    </asset>"""

    # -----------------------------------------------------------------------
    def __init__(
        self,
        *,
        panel_sz: tuple[float, float, float],        # (½w, ½h, ½t)
        pos:       tuple[float, float, float],       # world-space position
        quat:      tuple[float, float, float, float],# orientation
        name:      str  = "room_wall",
        backing:   bool = True,                      # include backing slab?
        backing_thickness: float = 0.10,             # metres
        **kwargs,
    ):
        self.panel_sz = panel_sz
        self.backing  = backing

        # backing geom uses the same half-width & half-height but larger z
        hx, hy, hz = panel_sz
        self.backing_sz = f"{hx} {hy} {backing_thickness/2:.3f}"

        attributes = dict(name=name, pos=" ".join(map(str, pos)),
                          quat=" ".join(map(str, quat)))

        # feed through to Body-base
        self.show_texture = kwargs.get("show_texture", self.show_texture)
        if not self.show_texture:
            self._preamble = """
            <asset>
                <material name="{prefix}_paint_mat" rgba="0.7 0.7 0.7 1" shininess="0.15"/>
            </asset>
            """      # disable texture/material preamble
        super().__init__(attributes=attributes, **kwargs)

    # -----------------------------------------------------------------------
    @property
    def _children_raw(self) -> str:
        hx, hy, hz = self.panel_sz
        panel_sz_str = f"{hx} {hy} {hz}"

        backing_raw = (
            f"""
        <geom name="{{name}}_back_col" size="{self.backing_sz}"
              type="box" group="3" rgba="0.5 0 0 1"/>
        <geom name="{{name}}_back_vis" size="{self.backing_sz}"
              type="box" contype="0" conaffinity="0" group="0" mass="0"
              material="{{prefix}}_paint_mat"/>"""
            if self.backing else ""
        )

        return f"""
    <geom name="{{name}}_panel_col" size="{panel_sz_str}"
          type="box" group="3" rgba="0.5 0 0 1"/>
    <geom name="{{name}}_panel_vis" size="{panel_sz_str}"
          type="box" contype="0" conaffinity="0" group="0" mass="0"
          material="{{prefix}}_paint_mat"/>{backing_raw}
    """




if __name__ == "__main__":
    from vuer_mjcf.objects.room_wall import RoomWall
    from vuer_mjcf.stage_sets.default_scene import DefaultStage
    from vuer_mjcf.utils.file import Prettify

    # Create a RoomWall instance
    obj = RoomWall(
        panel_sz=(2.3, 1.50, 0.02),
        pos=(1.15, 0.02, 1.5),
        quat=(-0.707107, 0.707107, 0, 0),
        name="wall_room",
        backing=False,
    )

    # Wrap in MuJoCo scene
    scene = DefaultStage(obj, model="test_room_wall_scene")

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
            print(f"✓ RoomWall model loaded successfully!")
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
