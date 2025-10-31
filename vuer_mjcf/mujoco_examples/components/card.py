from vuer_mjcf.schema import Body


class Card(Body):
    _attributes = {
        "name": "queen_of_spades",
    }
    _children_raw = """
      <freejoint/>
      <geom class="card" material="{name}"/>
      <geom class="collision"/>
    """

    _preamble = """
    <asset>
        <mesh file="card.obj"/>
        <texture type="2d" file="{name}.png"/>
        <material name="{name}" texture="{name}"/>
      </asset>
    <default>
        <geom solref=".5e-4" solimp="0.9 0.99 1e-4" fluidcoef="0.5 0.25 0.5 2.0 1.0"/>
        <default class="card">
          <geom type="mesh" mesh="card" mass="1.84e-4" fluidshape="ellipsoid" contype="0" conaffinity="0"/>
        </default>
        <default class="collision">
          <geom type="box" mass="0" size="0.047 0.032 .00035" group="3" friction=".1"/>
        </default>
      </default>
    """

if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    from vuer_mjcf.stage_sets.default_scene import DefaultStage
    from vuer_mjcf.utils.file import Prettify

    # Create card instances - demonstrating different cards
    card1 = Card(name="ace_of_spades", pos=[-0.3, 0, 1])
    card2 = Card(name="queen_of_hearts", pos=[0, 0, 1])
    card3 = Card(name="king_of_clubs", pos=[0.3, 0, 1])
    card4 = Card(name="jack_of_diamonds", pos=[0.6, 0, 1])

    # Wrap in MuJoCo scene
    scene = DefaultStage(card1, card2, card3, card4, model="test_cards_scene", assets=str(Path(__file__).parent.parent / "official_xmls" / "cards" / "assets") )

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
            print("✓ Card model loaded successfully!")
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
