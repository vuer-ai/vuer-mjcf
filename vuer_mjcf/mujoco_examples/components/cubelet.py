from vuer_mjcf.schema import Body


class Cubelet(Body):
    """
    A single cubelet piece of the Rubik's cube.

    Can be a face (1 color), edge (2 colors), or corner (3 colors).
    """

    material = "white"
    geom_pos = "0 0 0.019"
    joint_type = "ball"
    joint_axis = None

    _attributes = {
        "name": "cubelet",
    }

    _children_raw = """
      <joint name="{name}" type="{joint_type}"{axis_attr}/>
      <geom material="{material}" pos="{geom_pos}"/>
    """

    def _format_dict(self, omit: set = {}) -> dict:
        """Override to add conditional axis attribute."""
        props = super()._format_dict(omit)

        # Add axis attribute only if joint_axis is specified
        if self.joint_axis:
            props['axis_attr'] = f' axis="{self.joint_axis}"'
        else:
            props['axis_attr'] = ''

        return props


# Face cubelets (6 total - one per face)
class FacePX(Cubelet):
    material = "red"
    geom_pos = "0.019 0 0"
    joint_type = "hinge"
    joint_axis = "1 0 0"

class FaceNX(Cubelet):
    material = "orange"
    geom_pos = "-0.019 0 0"
    joint_type = "hinge"
    joint_axis = "-1 0 0"

class FacePY(Cubelet):
    material = "blue"
    geom_pos = "0 0.019 0"
    joint_type = "hinge"
    joint_axis = "0 1 0"

class FaceNY(Cubelet):
    material = "green"
    geom_pos = "0 -0.019 0"
    joint_type = "hinge"
    joint_axis = "0 -1 0"

class FacePZ(Cubelet):
    material = "white"
    geom_pos = "0 0 0.019"
    joint_type = "hinge"
    joint_axis = "0 0 1"

class FaceNZ(Cubelet):
    material = "yellow"
    geom_pos = "0 0 -0.019"
    joint_type = "hinge"
    joint_axis = "0 0 -1"


# Edge cubelets (12 total - two colors each)
class EdgePX_PY(Cubelet):
    material = "blue_red"
    geom_pos = "0.019 0.019 0"

class EdgeNY_PX(Cubelet):
    material = "green_red"
    geom_pos = "0.019 -0.019 0"

class EdgePX_PZ(Cubelet):
    material = "red_white"
    geom_pos = "0.019 0 0.019"

class EdgeNZ_PX(Cubelet):
    material = "red_yellow"
    geom_pos = "0.019 0 -0.019"

class EdgeNX_PY(Cubelet):
    material = "blue_orange"
    geom_pos = "-0.019 0.019 0"

class EdgeNX_NY(Cubelet):
    material = "green_orange"
    geom_pos = "-0.019 -0.019 0"

class EdgeNX_PZ(Cubelet):
    material = "orange_white"
    geom_pos = "-0.019 0 0.019"

class EdgeNX_NZ(Cubelet):
    material = "orange_yellow"
    geom_pos = "-0.019 0 -0.019"

class EdgePY_PZ(Cubelet):
    material = "blue_white"
    geom_pos = "0 0.019 0.019"

class EdgeNZ_PY(Cubelet):
    material = "blue_yellow"
    geom_pos = "0 0.019 -0.019"

class EdgeNY_PZ(Cubelet):
    material = "green_white"
    geom_pos = "0 -0.019 0.019"

class EdgeNY_NZ(Cubelet):
    material = "green_yellow"
    geom_pos = "0 -0.019 -0.019"


# Corner cubelets (8 total - three colors each)
class CornerPX_PY_PZ(Cubelet):
    material = "blue_red_white"
    geom_pos = "0.019 0.019 0.019"

class CornerNZ_PX_PY(Cubelet):
    material = "blue_red_yellow"
    geom_pos = "0.019 0.019 -0.019"

class CornerNY_PX_PZ(Cubelet):
    material = "green_red_white"
    geom_pos = "0.019 -0.019 0.019"

class CornerNY_NZ_PX(Cubelet):
    material = "green_red_yellow"
    geom_pos = "0.019 -0.019 -0.019"

class CornerNX_PY_PZ(Cubelet):
    material = "blue_orange_white"
    geom_pos = "-0.019 0.019 0.019"

class CornerNX_NZ_PY(Cubelet):
    material = "blue_orange_yellow"
    geom_pos = "-0.019 0.019 -0.019"

class CornerNX_NY_PZ(Cubelet):
    material = "green_orange_white"
    geom_pos = "-0.019 -0.019 0.019"

class CornerNX_NY_NZ(Cubelet):
    material = "green_orange_yellow"
    geom_pos = "-0.019 -0.019 -0.019"