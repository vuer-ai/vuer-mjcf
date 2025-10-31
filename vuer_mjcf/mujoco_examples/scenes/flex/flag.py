from pathlib import Path
from vuer_mjcf.schema import Mjcf, Raw
from vuer_mjcf.utils.file import Prettify


def make_schema(**options):
    """Generate flag flapping in the wind."""
    assets = str(Path(__file__).parent.parent.parent / "official_xmls" / "flex" / "asset")

    preamble = f"""
  <compiler meshdir="{assets}" texturedir="{assets}"/>
  <statistic center=".4 0 .8" extent="1.3"/>
  <option wind="5 5 0" density="10" solver="CG" tolerance="1e-6"/>
    """

    world = Raw('''
    <geom name="floor" type="plane" size="0 0 .1"/>
    <light diffuse=".6 .6 .6" specular="0.2 0.2 0.2" pos="0 0 4" dir="0 0 -1"/>
    <body name="pin" pos="0 0 1.5">
      <flexcomp type="grid" count="9 19 1" spacing=".05 .05 .05" mass="10"
                name="flag" radius="0.001">
        <edge equality="true" damping="0.001"/>
        <elasticity poisson="0" thickness="1e-2" young="3e6" elastic2d="none"/>
      </flexcomp>
    </body>
    ''')

    postamble = '''
  <equality>
    <connect body1="flag_0" anchor="0 0 0"/>
  </equality>
    '''

    return Mjcf(world, preamble=preamble, postamble=postamble, model="Flag")._xml | Prettify()


if __name__ == "__main__":
    import tempfile, mujoco, mujoco.viewer
    xml_str = make_schema()
    with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
        f.write(xml_str)
        model = mujoco.MjModel.from_xml_path(f.name)
        mujoco.viewer.launch(model, mujoco.MjData(model))
        Path(f.name).unlink()