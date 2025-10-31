from pathlib import Path
import random
from copy import copy
import numpy as np

from vuer_mjcf.objects.bowl_0 import ObjaverseMujocoBowl
from vuer_mjcf.objects.cup_3 import ObjaverseMujocoCup
from vuer_mjcf.objects.mj_sdf import MjSDF
from vuer_mjcf.objects.mug import ObjaverseMujocoMug
from vuer_mjcf.objects.plate import ObjaverseMujoco
from vuer_mjcf.objects.spoon_7 import ObjaverseMujocoSpoon
from vuer_mjcf.robots.astribot import AstribotRobotiq2f85
from vuer_mjcf.robots.robotiq_2f85 import Robotiq2F85
from vuer_mjcf.objects.cabinet import KitchenCabinet
from vuer_mjcf.objects.cupcake import Cupcake
from vuer_mjcf.objects.dishwasher import KitchenDishwasher
from vuer_mjcf.objects.drawer_stack import DrawerStack
from vuer_mjcf.objects.granite_countertop import GraniteCountertop
from vuer_mjcf.objects.refrigerator import KitchenFridge
from vuer_mjcf.objects.room_wall import RoomWall
from vuer_mjcf.objects.sink_wide import KitchenSinkWide
from vuer_mjcf.schema import Body, Replicate
from vuer_mjcf.tasks import add_env
from vuer_mjcf.tasks.base.lucidxr_task import get_site, init_states
from vuer_mjcf.tasks.base.mocap_task import MocapTask
from vuer_mjcf.basic_components.rigs.camera_rig_lower_fov import make_camera_rig
from vuer_mjcf.tasks._floating_shadowhand import FloatingShadowHand
from vuer_mjcf.tasks.entrypoint import make_env
import vuer_mjcf.se3.se3_mujoco as m
from vuer_mjcf.objects.microwave_scaled import KitchenMicrowave
from vuer_mjcf.objects.vuer_mug import VuerMug
from vuer_mjcf.tasks._floating_robotiq import FloatingRobotiq2f85
import mink
x1, y1 = 0.2, -1
origin = m.Vector3(-0.7, -0.2, 0.2)

def _compute_look_at_rotation(
    head_pos: np.ndarray,
    target_pos: np.ndarray,
    world_up: np.ndarray = np.array([0.0, 0.0, 1.0]),
) -> mink.SE3:
    """Returns SE3 whose rotation points +X from head_pos toward target_pos."""
    look_direction = target_pos - head_pos
    x_axis = look_direction / (np.linalg.norm(look_direction) + 1e-12)

    y_axis = np.cross(world_up, x_axis)
    ny = np.linalg.norm(y_axis)
    if ny < 1e-6:
        y_axis = np.cross(x_axis, np.array([1.0, 0.0, 0.0]))
        ny = np.linalg.norm(y_axis)
        if ny < 1e-6:
            y_axis = np.cross(x_axis, np.array([0.0, 1.0, 0.0]))
            ny = np.linalg.norm(y_axis)
    y_axis /= max(ny, 1e-12)

    z_axis = np.cross(x_axis, y_axis)
    R = np.column_stack((x_axis, y_axis, z_axis))
    return mink.SE3.from_rotation(mink.SO3.from_matrix(R))


def make_schema(production=True, **kwargs):
    from vuer_mjcf.utils.file import Prettify

    quat = m.WXYZ(0.7071068, 0, 0, -0.7071068)

    front_wall = RoomWall(
        panel_sz=(1.1, 1.6, 0.02),
        pos=(0.035, 0.02, 0.89),
        quat=(-0.707107, 0.707107, 0, 0),
        name="wall_room",
        assets="kitchen/wall",
        backing=False,
    )

    side_wall = RoomWall(
        panel_sz=(0.02, 1.6, 1.385),
        pos=(-1.005, -1.385, 0.89),
        quat=(-0.707107, 0.707107, 0, 0),
        name="wall_room_side",
        assets="kitchen/wall",
        backing=False,
    )

    start_indicator = Body(
        pos=(-0.2, 0.1, 0.8),
        attributes=dict(name="start_indicator-1"),
        _children_raw="""
        <site name="start_indicator-1" pos="0 0.0 0.0" size="0.025"/>
        """,
    )

    table_A = GraniteCountertop(
        assets="kitchen/counter", size="1.06 0.325 0.015", attributes={"pos": "0.075 -0.325 0.905", "name": "table_A"}
    )
    table_B = GraniteCountertop(
        assets="kitchen/counter", size="0.325 1.06 0.015", attributes={"pos": "-0.66 -1.71 0.905", "name": "table_B"}
    )

    cameras = make_camera_rig(pos=[-1, 0.1, 1])

    microwave = KitchenMicrowave(pos=[0.035, -0.3, 1.09], quat=[1, 0, 0, 0])

    cabinet_B = KitchenCabinet(assets="kitchen/cabinet", attributes={"pos": "0.5 -0.20 2.0", "name": "cabinet_B"}, remove_joints=True)

    mug = VuerMug(
        pos=[0.45, -0.3, 0.8],
        quat=[0, 0, 0, 1],
        scale="0.0186 0.0284 0.0186",
        _attributes={
            "name": "mug",
            # 0.00933 0.0142 0.00933
        },
        remove_joints=True,
    )
    cupcake = Cupcake(
        pos=[0.4, 0.225, 0.82],
        scale="0.05 0.05 0.05",
    )
    dw = KitchenDishwasher(assets="kitchen/dishwasher", attributes={"pos": "-0.27 -0.075 0.00", "name": "dishwasher", "childclass": "dw"}, remove_joints=True)

    stack_D = DrawerStack(base_pos="0.3 -0.3 0.00", name="stack_D", assets="kitchen", remove_joints=True)
    stack_E = DrawerStack(base_pos="0.81 -0.3 0.00", name="stack_E", assets="kitchen", remove_joints=True)

    base_pos = np.array([-0.2, -0.1, 0])
    head_target = _compute_look_at_rotation(head_pos=base_pos + np.array([0.0, 0.0, 1.5]), target_pos=np.array([0.4, 0.225, 0.82]))

    head_quat_wxyz = head_target.wxyz_xyz[:4]  # wxyz
    # scene = FloatingShadowHand(
    scene = AstribotRobotiq2f85(
        Body(
            front_wall,
            side_wall,
            table_A,
            table_B,
            microwave,
            attributes=dict(name="room", pos=-1 * origin, quat=quat),
        ),
        # muffin,
        start_indicator,
        cupcake,
        Body(
            dw,
            stack_D,
            stack_E,
            # cabinet_A,
            cabinet_B,
            attributes=dict(name="furniture", pos=-1 * origin, quat=quat),
            remove_joints=True
        ),
        mug,
        *cameras.get_cameras(),
        camera_rig=cameras,
        # pos=[-0.2, 0.1, 0.8],
        head_quat=head_quat_wxyz.tolist(),
        mocap_pos="0.02 0.02 0.9",
        mocap_quat=[0.708643978617976, -0.024511212639366363, 0.7040855110834829, -0.03855514121194968],
        pos=base_pos.tolist(),
        # bimanual=False,
        # quat=[0, 0, 0, 1],
        # dual_gripper=True,
    )

    return scene._xml | Prettify()


class Fixed(MocapTask):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.set_sites()

    def set_sites(self):
        self.muffin_site = get_site(self.physics, "cupcake")
        self.microwave_site = get_site(self.physics, "mw_site")

    def get_reward(self, physics):
        microwave_door_hinge = physics.named.data.qpos["mw_hinge"]
        # success if the muffin is outside of the microwave and the door is closed.

        muffin_pos = physics.data.site_xpos[self.muffin_site.id][:2]
        mw_pos = physics.data.site_xpos[self.microwave_site.id][:2]

        dist = np.linalg.norm(muffin_pos - mw_pos)
        door_closed = abs(microwave_door_hinge) < 0.20
        reward = float((dist > 0.3) and door_closed)
        # print(f"Distance: {dist}, Reward: {reward}")
        return reward


class Random(Fixed):
    box_qpos_addr = 1
    d = 0.02
    xy_limits = [-0.08, 0.08], [-0.08, 0.08]
    xy_reject = None
    yaw_limits = (-np.pi, np.pi)  # ← add a range for yaw
    initial_pos = [0.4, 0.225, 0.82]

    xy_poses = init_states(xy_limits, d, xy_reject)
    print(f"Length: {len(xy_poses)}")
    pose_buffer = None

    @classmethod
    def random_state(
        cls,
        qpos=None,
        quat=None,
        addr=box_qpos_addr,
        index=None,
        mocap_pos=None,
        **kwargs,
    ):
        if index is None:
            if not cls.pose_buffer:
                cls.pose_buffer = copy(cls.xy_poses)
                random.shuffle(cls.pose_buffer)

            x, y = cls.pose_buffer.pop(0)
            print(f"Randomly selected pose: {x}, {y}")
        else:
            x, y = cls.xy_poses[index]

        new_qpos = qpos.copy()
        new_qpos[addr] = cls.initial_pos[0] + x
        new_qpos[addr + 1] = cls.initial_pos[1] + y
        new_qpos[addr + 2] = cls.initial_pos[2]

        # Optional Gaussian noise on mocap
        if mocap_pos is not None:
            mocap_pos = mocap_pos.copy()
            mocap_pos += np.random.normal(0.0, 0.005, mocap_pos.shape)

        return dict(qpos=new_qpos, quat=quat, mocap_pos=mocap_pos, **kwargs)


def register(strict=False):
    add_env(
        env_id="MicrowaveMuffinAstribot-v1",
        entrypoint=make_env,
        kwargs=dict(
            task=Fixed,
            xml_renderer=make_schema,
            camera_names=["third-person", "wrist"],
            workdir=Path(__file__).parent,
            mode="multiview",
        ),
        strict=strict,
    )

    add_env(
        env_id="MicrowaveMuffin-Random-v1",
        entrypoint=make_env,
        kwargs=dict(
            task=Random,
            xml_renderer=make_schema,
            camera_names=["front", "front_r", "wrist"],
            workdir=Path(__file__).parent,
            mode="multiview",
            keyframe_file="microwave_muffin.frame.yaml",
        ),
        strict=strict,
    )

    add_env(
        env_id="MicrowaveMuffin-Random-lucid-v1",
        entrypoint=make_env,
        kwargs=dict(
            task=Random,
            xml_renderer=make_schema,
            camera_names=["front", "front_r", "wrist"],
            workdir=Path(__file__).parent,
            mode="lucid",
            object_keys=["cupcake", "mw"],
        ),
        strict=strict,
    )


if __name__ == "__main__":
    from vuer_mjcf.utils.file import Save

    make_schema() | Save(__file__.replace(".py", ".mjcf.xml"))

    # from vuer_mjcf.tasks import make
    # env = make("MicrowaveMuffin-Random-v1")
    # env.unwrapped.env.physics.named.data.qpos
    # env.reset()
