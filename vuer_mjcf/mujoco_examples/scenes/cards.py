from vuer_mjcf.schema import Mjcf, Body
from vuer_mjcf.utils.file import Prettify
from vuer_mjcf.mujoco_examples.components.card import Card


def make_schema(**options):
    """
    Generate the cards scene using modular Card components.

    Creates a stacked deck of 52 playing cards with realistic physics.
    Each card is slightly offset to create a falling/stacking effect.
    """

    # Define the full deck in order (matching cards.xml)
    card_names = [
        "2_of_clubs", "2_of_diamonds", "2_of_hearts", "2_of_spades",
        "3_of_clubs", "3_of_diamonds", "3_of_hearts", "3_of_spades",
        "4_of_clubs", "4_of_diamonds", "4_of_hearts", "4_of_spades",
        "5_of_clubs", "5_of_diamonds", "5_of_hearts", "5_of_spades",
        "6_of_clubs", "6_of_diamonds", "6_of_hearts", "6_of_spades",
        "7_of_clubs", "7_of_diamonds", "7_of_hearts", "7_of_spades",
        "8_of_clubs", "8_of_diamonds", "8_of_hearts", "8_of_spades",
        "9_of_clubs", "9_of_diamonds", "9_of_hearts", "9_of_spades",
        "10_of_clubs", "10_of_diamonds", "10_of_hearts", "10_of_spades",
        "jack_of_clubs", "jack_of_diamonds", "jack_of_hearts", "jack_of_spades",
        "queen_of_clubs", "queen_of_diamonds", "queen_of_hearts", "queen_of_spades",
        "king_of_clubs", "king_of_diamonds", "king_of_hearts", "king_of_spades",
        "ace_of_clubs", "ace_of_diamonds", "ace_of_hearts", "ace_of_spades",
    ]

    # Starting position and rotation
    base_pos = [0.000, -0.040, -0.000]
    base_euler = [-100, 180, 0]

    # Incremental offsets for stacking effect
    pos_increment = [0.005, 0.003, -0.005]
    euler_increment = [3, 0, 0]

    # Create all cards with incremental offsets
    cards = []
    for i, card_name in enumerate(card_names):
        pos = [
            base_pos[0] + i * pos_increment[0],
            base_pos[1] + i * pos_increment[1],
            base_pos[2] + i * pos_increment[2],
        ]
        euler = [
            base_euler[0] + i * euler_increment[0],
            base_euler[1] + i * euler_increment[1],
            base_euler[2] + i * euler_increment[2],
        ]

        card = Card(
            name=card_name,
            pos=pos,
            euler=" ".join(str(e) for e in euler)
        )
        cards.append(card)

    # Ground plane with grid material
    ground = Body(
        name="ground",
        _children_raw='<geom type="plane" size="5 5 .01" pos="0 0 -1" material="grid"/>'
    )

    # Support rod (from original XML)
    support = Body(
        name="support",
        _children_raw='<geom type="capsule" size="0.03" fromto=".1 -.1 -.3 .1 .1 -.3"/>'
    )

    # Lighting
    lights = Body(
        name="lights",
        _children_raw='''
        <light cutoff="30" pos="0 1 1" dir="0 -1 -2"/>
        <light cutoff="30" pos="0 -1 1" dir="0 1 -2"/>
        '''
    )

    # Camera
    camera = Body(
        name="camera",
        _children_raw='<camera name="cinematic" pos="0.119 -0.507 0.239" xyaxes="1 0 0 0 0.608 0.794" mode="trackcom" fovy="40"/>'
    )

    assets = str(Path(__file__).parent.parent / "official_xmls" / "cards" / "assets")

    # Compose scene with Mjcf
    scene = Mjcf(
        lights,
        camera,
        ground,
        support,
        *cards,
        model="cards",
        preamble=f"""
        <compiler assetdir="{assets}"/>
        <size memory="5M"/>
        <statistic extent="0.5" center="0 -.5 0"/>
        <visual>
          <map znear="0.1" zfar="10"/>
          <quality shadowsize="8192"/>
        </visual>
        <option density="1.225" viscosity="1.8e-5" timestep="1e-3" integrator="implicitfast"/>
        <asset>
          <texture name="grid" type="2d" builtin="checker" rgb1=".1 .2 .3" rgb2=".2 .3 .4" width="100" height="100"/>
          <material name="grid" texture="grid" texrepeat="8 8" texuniform="true" reflectance=".2"/>
        </asset>
        """,
    )

    return scene._xml | Prettify()


if __name__ == "__main__":
    import tempfile
    from pathlib import Path

    # Generate XML
    xml_str = make_schema()
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
            print("✓ Cards scene loaded successfully!")
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