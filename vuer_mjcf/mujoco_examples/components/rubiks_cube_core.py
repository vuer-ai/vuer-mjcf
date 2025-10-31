from vuer_mjcf.schema import Body
from vuer_mjcf.mujoco_examples.components.cubelet import (
    FacePX, FaceNX, FacePY, FaceNY, FacePZ, FaceNZ,
    EdgePX_PY, EdgeNY_PX, EdgePX_PZ, EdgeNZ_PX,
    EdgeNX_PY, EdgeNX_NY, EdgeNX_PZ, EdgeNX_NZ,
    EdgePY_PZ, EdgeNZ_PY, EdgeNY_PZ, EdgeNY_NZ,
    CornerPX_PY_PZ, CornerNZ_PX_PY, CornerNY_PX_PZ, CornerNY_NZ_PX,
    CornerNX_PY_PZ, CornerNX_NZ_PY, CornerNX_NY_PZ, CornerNX_NY_NZ,
)


class RubiksCubeCore(Body):
    """
    The core body of a 3x3x3 Rubik's cube with all 26 cubelets.

    Features:
    - Central core sphere (invisible, structural)
    - 6 face cubelets (single color)
    - 12 edge cubelets (two colors)
    - 8 corner cubelets (three colors)
    """

    _attributes = {
        "name": "core",
        "childclass": "cubelet",
    }

    def __init__(self, *args, **kwargs):
        # Create all 26 cubelets
        # 6 faces
        face_px = FacePX(name="pX")
        face_nx = FaceNX(name="nX")
        face_py = FacePY(name="pY")
        face_ny = FaceNY(name="nY")
        face_pz = FacePZ(name="pZ")
        face_nz = FaceNZ(name="nZ")

        # 12 edges
        edge_px_py = EdgePX_PY(name="pX_pY")
        edge_ny_px = EdgeNY_PX(name="nY_pX")
        edge_px_pz = EdgePX_PZ(name="pX_pZ")
        edge_nz_px = EdgeNZ_PX(name="nZ_pX")
        edge_nx_py = EdgeNX_PY(name="nX_pY")
        edge_nx_ny = EdgeNX_NY(name="nX_nY")
        edge_nx_pz = EdgeNX_PZ(name="nX_pZ")
        edge_nx_nz = EdgeNX_NZ(name="nX_nZ")
        edge_py_pz = EdgePY_PZ(name="pY_pZ")
        edge_nz_py = EdgeNZ_PY(name="nZ_pY")
        edge_ny_pz = EdgeNY_PZ(name="nY_pZ")
        edge_ny_nz = EdgeNY_NZ(name="nY_nZ")

        # 8 corners
        corner_px_py_pz = CornerPX_PY_PZ(name="pX_pY_pZ")
        corner_nz_px_py = CornerNZ_PX_PY(name="nZ_pX_pY")
        corner_ny_px_pz = CornerNY_PX_PZ(name="nY_pX_pZ")
        corner_ny_nz_px = CornerNY_NZ_PX(name="nY_nZ_pX")
        corner_nx_py_pz = CornerNX_PY_PZ(name="nX_pY_pZ")
        corner_nx_nz_py = CornerNX_NZ_PY(name="nX_nZ_pY")
        corner_nx_ny_pz = CornerNX_NY_PZ(name="nX_nY_pZ")
        corner_nx_ny_nz = CornerNX_NY_NZ(name="nX_nY_nZ")

        # Pass all cubelets as children
        super().__init__(
            face_px, face_nx, face_py, face_ny, face_pz, face_nz,
            edge_px_py, edge_ny_px, edge_px_pz, edge_nz_px,
            edge_nx_py, edge_nx_ny, edge_nx_pz, edge_nx_nz,
            edge_py_pz, edge_nz_py, edge_ny_pz, edge_ny_nz,
            corner_px_py_pz, corner_nz_px_py, corner_ny_px_pz, corner_ny_nz_px,
            corner_nx_py_pz, corner_nx_nz_py, corner_nx_ny_pz, corner_nx_ny_nz,
            *args,
            **kwargs
        )

    _children_raw = """
      <geom class="core"/>
    """