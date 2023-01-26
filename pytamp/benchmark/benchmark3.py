import numpy as np

from pykin.robots.single_arm import SingleArm
from pykin.kinematics.transform import Transform
from pykin.utils.mesh_utils import get_object_mesh
from pykin.utils.transform_utils import get_h_mat
from pytamp.benchmark.benchmark import Benchmark


class Benchmark3(Benchmark):
    def __init__(
        self, robot_name="panda", geom="visual", is_pyplot=True, only_sim=False
    ):
        param = {"goal_object": "goal_can"}
        self.benchmark_config = {3: param}
        super().__init__(robot_name, geom, is_pyplot, self.benchmark_config, only_sim)
        self._load_robot()
        self._load_objects()
        self._load_scene()

    def _load_robot(self):
        self.robot = SingleArm(
            f_name=self.urdf_file,
            offset=Transform(rot=[0.0, 0.0, 0.0], pos=[0, 0, 0.913]),
            has_gripper=True,
            gripper_name=self.gripper_name,
        )

        if self.robot_name == "panda":
            self.robot.setup_link_name("panda_link_0", "right_hand")
            self.robot.init_qpos = np.array(
                [
                    0,
                    np.pi / 16.0,
                    0.00,
                    -np.pi / 2.0 - np.pi / 3.0,
                    0.00,
                    np.pi - 0.2,
                    -np.pi / 4,
                ]
            )

        if self.robot_name == "doosan":
            self.robot.setup_link_name("base_0", "right_hand")
            self.robot.init_qpos = np.array([0, 0, np.pi / 1.5, 0, np.pi / 3, 0])

    def _load_objects(self):
        self.table_mesh = get_object_mesh("ben_table.stl", [1.0, 1.5, 1.0])
        self.table_height = self.table_mesh.bounds[1][2] - self.table_mesh.bounds[0][2]
        self.table_pose = Transform(
            pos=np.array([1.0, -0.6, -self.table_mesh.bounds[0][2]])
        )

        self.tray_blue_mesh = get_object_mesh("ben_tray_blue.stl")
        self.tray_blue_mesh.apply_translation(-self.tray_blue_mesh.center_mass)

        #! clearbox 8, 16 is placing spot
        self.clearbox = get_object_mesh(f"clearbox.stl", scale=[1.4, 1.3, 1.5])
        self.clearbox_pose = Transform(
            pos=np.array(
                [0.7, 0.4, self.table_height + abs(self.clearbox.bounds[0][2])]
            ),
            rot=[0, 0, np.pi / 2],
        )

        clearbox_8_mesh = get_object_mesh(f"clearbox_8.stl", scale=[1.4, 1.3, 1.5])
        self.clearbox_8_height = (
            clearbox_8_mesh.bounds[1][2] - clearbox_8_mesh.bounds[0][2]
        )

        self.square_box_num = 3
        self.square_boxes = []
        self.square_box_poses = []
        self.square_box_colors = []
        for i in range(self.square_box_num):
            self.square_boxes.append(get_object_mesh("ben_cube.stl", [0.1, 0.1, 0.07]))
            self.square_boxes[i].apply_translation(-self.square_boxes[i].center_mass)

            squarebox_height = (
                self.square_boxes[i].bounds[1][2] - self.square_boxes[i].bounds[0][2]
            )
            square_box_pose = Transform(
                pos=np.array(
                    [
                        0.85,
                        0.4,
                        self.table_height
                        + self.clearbox_8_height
                        + abs(self.square_boxes[0].bounds[0][2])
                        + squarebox_height * i,
                    ]
                )
            )
            square_box_color = [0, 0.2 + i * 0.1, 0.4]

            self.square_box_poses.append(square_box_pose)
            self.square_box_colors.append(square_box_color)

        self.rect_box_num = 3
        self.rect_boxes = []
        self.rect_box_poses = []
        self.rect_box_colors = []
        for i in range(self.rect_box_num):
            self.rect_boxes.append(
                get_object_mesh("rect_box.stl", [0.002, 0.0005, 0.002])
            )
            self.rect_boxes[i].apply_transform(
                get_h_mat(orientation=np.array([np.pi / 2, 0, np.pi / 2]))
            )
            self.rect_boxes[i].apply_translation(-self.rect_boxes[i].center_mass)

            rectbox_height = (
                self.rect_boxes[0].bounds[1][2] - self.rect_boxes[0].bounds[0][2]
            )
            rect_box_pose = Transform(
                pos=np.array(
                    [
                        0.72,
                        0.3,
                        self.table_height
                        + self.clearbox_8_height
                        + abs(self.rect_boxes[i].bounds[0][2])
                        + rectbox_height * i,
                    ]
                )
            )

            rect_box_color = [0.4, 0.2 + i * 0.1, 0.4]
            self.rect_box_poses.append(rect_box_pose)
            self.rect_box_colors.append(rect_box_color)

        self.milk1 = get_object_mesh("milk.stl", [1.5, 1.5, 1.3])
        self.milk1.apply_translation(-self.milk1.center_mass)

        self.milk2 = get_object_mesh("milk.stl", [1.5, 1.5, 1.3])
        self.milk2.apply_translation(-self.milk2.center_mass)

        self.milk3 = get_object_mesh("milk.stl", [1.5, 1.5, 1.3])
        self.milk3.apply_translation(-self.milk3.center_mass)

        self.milk4 = get_object_mesh("milk.stl", [1.5, 1.5, 1.3])
        self.milk4.apply_translation(-self.milk4.center_mass)

        self.milk5 = get_object_mesh("milk.stl", [1.5, 1.5, 1.3])
        self.milk5.apply_translation(-self.milk4.center_mass)

        self.can = get_object_mesh("can.stl", [1.8, 1.8, 2.0])
        self.can.apply_translation(-self.can.center_mass)

        self.milk1_pose = Transform(
            pos=np.array(
                [
                    0.76,
                    0.5,
                    self.table_height
                    + self.clearbox_8_height
                    + abs(self.milk1.bounds[0][2]),
                ]
            ),
            rot=[0, 0, np.pi / 12],
        )
        self.milk2_pose = Transform(
            pos=np.array(
                [
                    0.62,
                    0.42,
                    self.table_height
                    + self.clearbox_8_height
                    + abs(self.milk2.bounds[0][2]),
                ]
            ),
            rot=[0, 0, np.pi / 20],
        )
        self.milk3_pose = Transform(
            pos=np.array(
                [
                    0.6,
                    0.5,
                    self.table_height
                    + self.clearbox_8_height
                    + abs(self.milk3.bounds[0][2]),
                ]
            ),
            rot=[0, 0, -np.pi / 2],
        )
        self.milk4_pose = Transform(
            pos=np.array(
                [
                    0.58,
                    0.32,
                    self.table_height
                    + self.clearbox_8_height
                    + abs(self.milk4.bounds[0][2]),
                ]
            )
        )
        self.milk5_pose = Transform(
            pos=np.array(
                [
                    0.52,
                    0.42,
                    self.table_height
                    + self.clearbox_8_height
                    + abs(self.milk5.bounds[0][2]),
                ]
            )
        )
        self.can_pose = Transform(
            pos=np.array(
                [
                    0.72,
                    0.4,
                    self.table_height
                    + self.clearbox_8_height
                    + abs(self.can.bounds[0][2]),
                ]
            )
        )
        self.tray_blue_pose = Transform(pos=np.array([0.6, 0, 0.8]))

    def _load_scene(self):
        self.benchmark_config = {3: None}
        self.scene_mngr.add_object(
            name="table",
            gtype="mesh",
            gparam=self.table_mesh,
            h_mat=self.table_pose.h_mat,
            color=[0.823, 0.71, 0.55],
        )

        for i in range(self.rect_box_num):
            rect_box_name = "rect_box" + str(i)
            self.scene_mngr.add_object(
                name=rect_box_name,
                gtype="mesh",
                gparam=self.rect_boxes[i],
                h_mat=self.rect_box_poses[i].h_mat,
                color=self.rect_box_colors[i],
            )

        for i in range(self.square_box_num):
            square_box_name = "square_box" + str(i)
            self.scene_mngr.add_object(
                name=square_box_name,
                gtype="mesh",
                gparam=self.square_boxes[i],
                h_mat=self.square_box_poses[i].h_mat,
                color=self.square_box_colors[i],
            )

        self.scene_mngr.add_object(
            name="milk1", gtype="mesh", gparam=self.milk1, h_mat=self.milk1_pose.h_mat
        )
        self.scene_mngr.add_object(
            name="milk2", gtype="mesh", gparam=self.milk2, h_mat=self.milk2_pose.h_mat
        )
        self.scene_mngr.add_object(
            name="milk3", gtype="mesh", gparam=self.milk3, h_mat=self.milk3_pose.h_mat
        )
        self.scene_mngr.add_object(
            name="milk4", gtype="mesh", gparam=self.milk4, h_mat=self.milk4_pose.h_mat
        )
        # self.scene_mngr.add_object(name="milk5", gtype="mesh", gparam=self.milk5, h_mat=self.milk5_pose.h_mat)
        self.scene_mngr.add_object(
            name="goal_can",
            gtype="mesh",
            gparam=self.can,
            h_mat=self.can_pose.h_mat,
            color=[1.0, 0.0, 0.0],
        )
        self.scene_mngr.add_object(
            name="clearbox",
            gtype="mesh",
            gparam=self.clearbox,
            h_mat=self.clearbox_pose.h_mat,
            color=[0.8, 0.8, 0.8],
        )
        self.scene_mngr.add_object(
            name="tray_blue",
            gtype="mesh",
            gparam=self.tray_blue_mesh,
            h_mat=self.tray_blue_pose.h_mat,
            color=[0, 0, 1.0],
        )
        self.scene_mngr.add_robot(self.robot)

        if not self.only_sim:
            for i in range(self.rect_box_num):
                rect_box_name = "rect_box" + str(i)
                if rect_box_name == "rect_box0":
                    self.scene_mngr.set_logical_state(rect_box_name, ("on", "table"))
                else:
                    prev_rect_box_name = "rect_box" + str(i - 1)
                    self.scene_mngr.set_logical_state(
                        rect_box_name, ("on", prev_rect_box_name)
                    )

            for i in range(self.square_box_num):
                square_box_name = "square_box" + str(i)
                if square_box_name == "square_box0":
                    self.scene_mngr.set_logical_state(square_box_name, ("on", "table"))
                else:
                    prev_square_box_name = "square_box" + str(i - 1)
                    self.scene_mngr.set_logical_state(
                        square_box_name, ("on", prev_square_box_name)
                    )

            self.scene_mngr.set_logical_state("goal_can", ("on", "clearbox"))
            self.scene_mngr.set_logical_state("milk1", ("on", "clearbox"))
            self.scene_mngr.set_logical_state("milk2", ("on", "clearbox"))
            self.scene_mngr.set_logical_state("milk3", ("on", "clearbox"))
            self.scene_mngr.set_logical_state("milk4", ("on", "clearbox"))
            # self.scene_mngr.set_logical_state("milk5", ("on", "clearbox"))
            self.scene_mngr.set_logical_state(
                "table", (self.scene_mngr.scene.logical_state.static, True)
            )
            self.scene_mngr.set_logical_state(
                "tray_blue", (self.scene_mngr.scene.logical_state.static, True)
            )
            self.scene_mngr.set_logical_state(
                "clearbox",
                (self.scene_mngr.scene.logical_state.static, True),
                ("on", "table"),
            )
            self.scene_mngr.set_logical_state(
                self.scene_mngr.gripper_name,
                (self.scene_mngr.scene.logical_state.holding, None),
            )
            self.scene_mngr.show_logical_states()
        self.scene_mngr.update_logical_states(is_init=True)
