import numpy as np
from collections import OrderedDict
from copy import deepcopy
from trimesh import Trimesh, proximity

from pykin.utils import mesh_utils as m_utils
from pykin.utils.log_utils import create_logger
from pytamp.action.activity import ActivityBase
from pytamp.scene.scene import Scene
from pytamp.utils.heuristic_utils import get_heuristic_release_eef_pose

logger = create_logger("PlaceAction", "debug")


class PlaceAction(ActivityBase):
    def __init__(
        self,
        scene_mngr,
        n_samples_held_obj=10,
        n_samples_support_obj=10,
        n_directions=1,
        retreat_distance=0.1,
        release_distance=0.01,
    ):
        super().__init__(scene_mngr)
        self.n_samples_held_obj = n_samples_held_obj
        self.n_samples_sup_obj = n_samples_support_obj

        if n_directions < 1:
            n_directions = 1
        self.n_directions = n_directions

        self.retreat_distance = retreat_distance
        self.release_distance = release_distance
        self.filter_logical_states = [scene_mngr.scene.logical_state.held]

    def get_possible_actions_level_1(self, scene: Scene = None) -> dict:
        self.deepcopy_scene(scene)
        held_obj = self.scene_mngr.scene.robot.gripper.attached_obj_name
        eef_pose = self.scene_mngr.scene.robot.gripper.grasp_pose
        self.scene_mngr.scene.objs[
            held_obj
        ].h_mat = self.scene_mngr.scene.robot.gripper.pick_obj_pose

        for sup_obj in deepcopy(self.scene_mngr.scene.objs):
            # print(f"place : {sup_obj}")
            if sup_obj == held_obj:
                continue

            if "ceiling" in sup_obj:
                continue

            # ? for benchmark 1
            if self.scene_mngr.scene.bench_num == 1:
                if "box" in sup_obj and "box" in held_obj:
                    sup_obj_num = ord(sup_obj.split("_")[0])
                    held_obj_num = ord(held_obj.split("_")[0])
                    if held_obj_num < sup_obj_num:
                        continue

                if sup_obj in self.scene_mngr.scene.prev_place_obj_name:
                    continue

            # ? for benchmark 2
            if self.scene_mngr.scene.bench_num == 2:
                if sup_obj not in ["shelf_9"]:
                    continue

            # ? for benchmark 3
            if self.scene_mngr.scene.bench_num == 3:
                if "can" not in held_obj:
                    if sup_obj not in ["table"]:
                        continue
                else:
                    if sup_obj not in ["tray_blue"]:
                        continue

            # ? for benchmark 4
            if self.scene_mngr.scene.bench_num == 4:
                support_objects = self.scene_mngr.scene.goal_objects + ["table"]
                if sup_obj not in support_objects:
                    continue

                if "hanoi_disk" in sup_obj and "hanoi_disk" in held_obj:
                    sup_obj_num = float(sup_obj.split("_")[-1])
                    held_obj_num = float(held_obj.split("_")[-1])
                    if held_obj_num < sup_obj_num:
                        continue

            is_same_prev_sup_obj = False
            for prev_place_obj_name in self.scene_mngr.scene.prev_place_obj_name:
                if sup_obj == prev_place_obj_name:
                    if self.scene_mngr.scene.bench_num == 2:
                        if sup_obj not in ["shelf_9"]:
                            is_same_prev_sup_obj = True
                    if self.scene_mngr.scene.bench_num == 4:
                        if sup_obj not in ["table"]:
                            is_same_prev_sup_obj = True
            if is_same_prev_sup_obj:
                continue

            if not any(
                logical_state in self.scene_mngr.scene.logical_states[sup_obj]
                for logical_state in self.filter_logical_states
            ):
                action_level_1 = self.get_action_level_1_for_single_object(
                    sup_obj, held_obj, eef_pose
                )
                if not action_level_1[self.info.RELEASE_POSES]:
                    continue

                yield action_level_1

    def get_action_level_1_for_single_object(
        self, sup_obj_name, held_obj_name, eef_pose=None, scene: Scene = None
    ):
        if scene is not None:
            # Bug !! pick action에서 object 위치를 바꿔 놓았는데 다시 원래대로 왜 돌려놓는걸까.. 아래 함수에서 h_mat불러서 안쓰고, pick_obj_pose로 대체하니까 잘 돌아감!
            # scene.objs[held_obj_name].h_mat = scene.robot.gripper.pick_obj_pose
            self.deepcopy_scene(scene)

        release_poses = list(
            self.get_all_release_poses_and_obj_pose(
                sup_obj_name, held_obj_name, eef_pose
            )
        )
        release_poses_not_collision = list(
            self.get_release_poses_not_collision(release_poses)
        )
        action_level_1 = self.get_action(
            held_obj_name, sup_obj_name, release_poses_not_collision
        )
        return action_level_1

    # Not Expand, only check possible action using ik
    #! It will be removed !!
    def get_possible_ik_solve_level_2(
        self, scene: Scene = None, release_poses: dict = {}
    ):
        self.deepcopy_scene(scene)

        ik_solve, release_poses_filtered = self.compute_ik_solve_for_robot(
            release_poses
        )
        return ik_solve, release_poses_filtered

    def get_possible_joint_path_level_2(
        self, scene: Scene = None, release_poses: dict = {}, init_thetas=None
    ):
        self.deepcopy_scene(scene)

        result_all_joint_path = []
        result_joint_path = OrderedDict()
        default_joint_path = []

        default_thetas = init_thetas
        if init_thetas is None:
            default_thetas = self.scene_mngr.scene.robot.init_qpos

        pre_release_pose = release_poses[self.move_data.MOVE_pre_release]
        release_pose = release_poses[self.move_data.MOVE_release]
        post_release_pose = release_poses[self.move_data.MOVE_post_release]
        success_joint_path = True

        self.scene_mngr.set_robot_eef_pose(default_thetas)
        self.scene_mngr.set_object_pose(
            scene.pick_obj_name, scene.pick_obj_default_pose
        )
        self.scene_mngr.attach_object_on_gripper(
            self.scene_mngr.scene.robot.gripper.attached_obj_name, True
        )

        pre_release_joint_path = self.get_rrt_star_path(
            default_thetas, pre_release_pose
        )
        self.cost = 0
        if pre_release_joint_path:
            self.cost += self.rrt_planner.goal_node_cost
            # pre_release_pose -> release_pose (cartesian)
            release_joint_path = self.get_cartesian_path(
                pre_release_joint_path[-1], release_pose
            )
            if release_joint_path:
                self.scene_mngr.detach_object_from_gripper()
                self.scene_mngr.add_object(
                    self.scene_mngr.scene.robot.gripper.attached_obj_name,
                    self.scene_mngr.init_objects[
                        self.scene_mngr.scene.robot.gripper.attached_obj_name
                    ].gtype,
                    self.scene_mngr.init_objects[
                        self.scene_mngr.scene.robot.gripper.attached_obj_name
                    ].gparam,
                    scene.robot.gripper.place_obj_pose,
                    self.scene_mngr.init_objects[
                        self.scene_mngr.scene.robot.gripper.attached_obj_name
                    ].color,
                )

                # release_pose -> post_release_pose (cartesian)
                post_release_joint_path = self.get_cartesian_path(
                    release_joint_path[-1], post_release_pose
                )
                if post_release_joint_path:
                    # post_release_pose -> default pose (rrt)
                    default_joint_path = self.get_rrt_star_path(
                        post_release_joint_path[-1], goal_q=default_thetas
                    )
                else:
                    success_joint_path = False
            else:
                success_joint_path = False
        else:
            success_joint_path = False

        if not success_joint_path:
            if self.scene_mngr.is_attached:
                self.scene_mngr.detach_object_from_gripper()
            try:
                self.scene_mngr.add_object(
                    self.scene_mngr.scene.robot.gripper.attached_obj_name,
                    self.scene_mngr.init_objects[
                        self.scene_mngr.scene.robot.gripper.attached_obj_name
                    ].gtype,
                    self.scene_mngr.init_objects[
                        self.scene_mngr.scene.robot.gripper.attached_obj_name
                    ].gparam,
                    scene.robot.gripper.place_obj_pose,
                    self.scene_mngr.init_objects[
                        self.scene_mngr.scene.robot.gripper.attached_obj_name
                    ].color,
                )
            except ValueError as e:
                print(e)
            return result_all_joint_path

        if default_joint_path:
            self.cost += self.rrt_planner.goal_node_cost
            result_joint_path.update(
                {self.move_data.MOVE_pre_release: pre_release_joint_path}
            )
            result_joint_path.update({self.move_data.MOVE_release: release_joint_path})
            result_joint_path.update(
                {self.move_data.MOVE_post_release: post_release_joint_path}
            )
            result_joint_path.update(
                {self.move_data.MOVE_default_release: default_joint_path}
            )
            result_all_joint_path.append(result_joint_path)

            return result_all_joint_path

    def get_action(self, held_obj_name, place_obj_name, poses):
        action = {}
        action[self.info.TYPE] = "place"
        action[self.info.HELD_OBJ_NAME] = held_obj_name
        action[self.info.PLACE_OBJ_NAME] = place_obj_name
        action[self.info.RELEASE_POSES] = poses
        return action

    def get_possible_transitions(self, scene: Scene = None, action: dict = {}):
        if not action:
            ValueError("Not found any action!!")

        held_obj_name = action[self.info.HELD_OBJ_NAME]
        place_obj_name = action[self.info.PLACE_OBJ_NAME]

        for release_poses, obj_pose_transformed in action[self.info.RELEASE_POSES]:
            next_scene = deepcopy(scene)

            ## Change transition
            next_scene.release_poses = release_poses
            next_scene.robot.gripper.place_obj_pose = obj_pose_transformed
            next_scene.robot.gripper.release_pose = release_poses[
                self.move_data.MOVE_release
            ]

            default_thetas = self.scene_mngr.scene.robot.init_qpos
            default_pose = self.scene_mngr.scene.robot.forward_kin(default_thetas)[
                self.scene_mngr.scene.robot.eef_name
            ].h_mat
            next_scene.robot.gripper.set_gripper_pose(default_pose)
            # next_scene.robot.gripper.set_gripper_pose(release_poses[self.move_data.MOVE_release])

            # Move pick object on support obj
            next_scene.objs[held_obj_name].h_mat = obj_pose_transformed
            self.scene_mngr.obj_collision_mngr.set_transform(
                held_obj_name, obj_pose_transformed
            )
            next_scene.cur_place_obj_name = place_obj_name

            ## Change Logical State
            # Clear logical_state of held obj
            next_scene.logical_states.get(held_obj_name).clear()

            # Chage logical_state holding : None
            next_scene.logical_states[next_scene.robot.gripper.name][
                next_scene.logical_state.holding
            ] = None

            # Add logical_state of held obj : {'on' : place_obj}
            next_scene.logical_states[held_obj_name][
                next_scene.logical_state.on
            ] = next_scene.objs[place_obj_name]

            if self.scene_mngr.scene.bench_num == 4:
                y_pose = obj_pose_transformed[1, 3]
                peg = self._get_peg(y_pose)
                next_scene.logical_states[held_obj_name][
                    next_scene.logical_state.hang
                ] = next_scene.objs[peg]
                next_scene.cur_peg_name = next_scene.objs[peg].name
            next_scene.update_logical_states()

            if self.scene_mngr.scene.bench_num == 1:
                # Check stability
                copied_scene = deepcopy(next_scene)
                held_obj = copied_scene.objs[held_obj_name]
                held_obj_mesh: Trimesh = deepcopy(held_obj.gparam)
                held_obj_mesh.apply_transform(obj_pose_transformed)
                com = held_obj_mesh.center_mass
                if not self._check_stability(next_scene, held_obj_name, com):
                    continue

            yield next_scene

    # Not consider collision
    def get_all_release_poses_and_obj_pose(
        self, support_obj_name, held_obj_name, eef_pose=None
    ):
        # gripper = self.scene_mngr.scene.robot.gripper
        transformed_eef_poses = list(
            self.get_transformed_eef_poses(support_obj_name, held_obj_name, eef_pose)
        )

        for eef_pose, obj_pose_transformed in transformed_eef_poses:
            if self.scene_mngr._scene.bench_num == 1:
                if not self._check_support(
                    support_obj_name, held_obj_name, obj_pose_transformed
                ):
                    continue
            if self.scene_mngr._scene.bench_num == 4:
                if support_obj_name == "table":
                    y_pose = obj_pose_transformed[1, 3]
                    peg = self._get_peg(y_pose)
                    if peg == self.scene_mngr.scene.prev_peg_name:
                        continue

            if self.scene_mngr.heuristic:
                heuristic_poses = get_heuristic_release_eef_pose(
                    obj_pose_transformed, eef_pose, self.n_directions
                )

                for eef_pose, obj_pose_transformed in heuristic_poses:
                    release_poses = self.get_all_release_poses(eef_pose)
                    yield release_poses, obj_pose_transformed
            else:
                release_poses = self.get_all_release_poses(eef_pose)
                yield release_poses, obj_pose_transformed

    def get_all_release_poses(self, eef_pose):
        release_pose = {}
        release_pose[self.move_data.MOVE_release] = eef_pose
        release_pose[self.move_data.MOVE_pre_release] = self.get_pre_release_pose(
            eef_pose
        )
        release_pose[self.move_data.MOVE_post_release] = self.get_post_release_pose(
            eef_pose
        )
        return release_pose

    def get_pre_release_pose(self, release_pose):
        pre_release_pose = np.eye(4)
        pre_release_pose[:3, :3] = release_pose[:3, :3]
        pre_release_pose[:3, 3] = release_pose[:3, 3] + np.array(
            [0, 0, self.retreat_distance]
        )
        return pre_release_pose

    def get_post_release_pose(self, release_pose):
        post_release_pose = np.eye(4)
        post_release_pose[:3, :3] = release_pose[:3, :3]
        if self.scene_mngr.scene.bench_num != 3:
            post_release_pose[:3, 3] = (
                release_pose[:3, 3] - self.retreat_distance * release_pose[:3, 2]
            )
        else:
            post_release_pose[:3, 3] = release_pose[:3, 3] + np.array(
                [0, 0, self.retreat_distance]
            )
        return post_release_pose

    # for level wise - 1 (Consider gripper collision)
    def get_release_poses_not_collision(self, release_poses, is_attached=True):
        if self.scene_mngr.scene.robot.has_gripper is None:
            raise ValueError("Robot doesn't have a gripper")

        for all_release_pose, obj_pose_transformed in release_poses:
            if is_attached:
                self.scene_mngr.attach_object_on_gripper(
                    self.scene_mngr.scene.robot.gripper.attached_obj_name, True
                )
            self.scene_mngr.close_gripper()
            for name, pose in all_release_pose.items():

                is_collision = False
                if name == self.move_data.MOVE_release:
                    self.scene_mngr.set_gripper_pose(pose)
                    for name in self.scene_mngr.scene.objs:
                        # self.scene_mngr.obj_collision_mngr.set_transform(name, self.scene_mngr.scene.objs[name].h_mat)
                        self.scene_mngr.obj_collision_mngr.set_transform(
                            name, self.scene_mngr.scene.robot.gripper.pick_obj_pose
                        )
                    if self._collide(is_only_gripper=True):
                        is_collision = True
                        break
                if name == self.move_data.MOVE_pre_release:
                    self.scene_mngr.set_gripper_pose(pose)
                    if self._collide(is_only_gripper=True):
                        is_collision = True
                        break
                if name == self.move_data.MOVE_post_release:
                    self.scene_mngr.set_gripper_pose(pose)
                    if self._collide(is_only_gripper=True):
                        is_collision = True
                        break

            if is_attached:
                self.scene_mngr.detach_object_from_gripper()
                self.scene_mngr.add_object(
                    self.scene_mngr.scene.robot.gripper.attached_obj_name,
                    self.scene_mngr.init_objects[
                        self.scene_mngr.scene.robot.gripper.attached_obj_name
                    ].gtype,
                    self.scene_mngr.init_objects[
                        self.scene_mngr.scene.robot.gripper.attached_obj_name
                    ].gparam,
                    self.scene_mngr.scene.robot.gripper.pick_obj_pose,
                    self.scene_mngr.init_objects[
                        self.scene_mngr.scene.robot.gripper.attached_obj_name
                    ].color,
                )
            self.scene_mngr.open_gripper()
            if not is_collision:
                yield all_release_pose, obj_pose_transformed

    def compute_ik_solve_for_robot(self, release_pose: dict, is_attached=True):
        ik_solve = {}
        release_pose_for_ik = {}

        if is_attached:
            self.scene_mngr.attach_object_on_gripper(
                self.scene_mngr.scene.robot.gripper.attached_obj_name, True
            )

        for name, pose in release_pose.items():
            if name == self.move_data.MOVE_release:
                thetas = self.scene_mngr.compute_ik(pose=pose, max_iter=100)
                self.scene_mngr.set_robot_eef_pose(thetas)
                release_pose_from_ik = self.scene_mngr.get_robot_eef_pose()
                if self._solve_ik(pose, release_pose_from_ik) and not self._collide(
                    is_only_gripper=False
                ):
                    ik_solve[name] = thetas
                    release_pose_for_ik[name] = pose
            if name == self.move_data.MOVE_pre_release:
                thetas = self.scene_mngr.compute_ik(pose=pose, max_iter=100)
                self.scene_mngr.set_robot_eef_pose(thetas)
                pre_release_pose_from_ik = self.scene_mngr.get_robot_eef_pose()
                if self._solve_ik(pose, pre_release_pose_from_ik) and not self._collide(
                    is_only_gripper=False
                ):
                    ik_solve[name] = thetas
                    release_pose_for_ik[name] = pose
            if name == self.move_data.MOVE_post_release:
                thetas = self.scene_mngr.compute_ik(pose=pose, max_iter=100)
                self.scene_mngr.set_robot_eef_pose(thetas)
                post_release_pose_from_ik = self.scene_mngr.get_robot_eef_pose()
                if self._solve_ik(
                    pose, post_release_pose_from_ik
                ) and not self._collide(is_only_gripper=False):
                    ik_solve[name] = thetas
                    release_pose_for_ik[name] = pose

        if is_attached:
            self.scene_mngr.detach_object_from_gripper()
            self.scene_mngr.add_object(
                self.scene_mngr.scene.robot.gripper.attached_obj_name,
                self.scene_mngr.init_objects[
                    self.scene_mngr.scene.robot.gripper.attached_obj_name
                ].gtype,
                self.scene_mngr.init_objects[
                    self.scene_mngr.scene.robot.gripper.attached_obj_name
                ].gparam,
                self.scene_mngr.scene.robot.gripper.pick_obj_pose,
                self.scene_mngr.init_objects[
                    self.scene_mngr.scene.robot.gripper.attached_obj_name
                ].color,
            )

        if len(ik_solve) == 3:
            return ik_solve, release_pose_for_ik
        return None, None

    def get_surface_points_for_support_obj(self, obj_name, alpha=0.2):
        support_obj = self.scene_mngr.scene.objs[obj_name]
        copied_mesh = deepcopy(support_obj.gparam)
        copied_mesh.apply_transform(support_obj.h_mat)
        center_point = copied_mesh.center_mass

        ### support object가 똑바로나 서있을 때 가능한 이야기고, 기울어 져있으면 다 좌표 다 틀림.
        # 만약 한변 길이 안다면
        len_x = abs(center_point[0] - copied_mesh.bounds[0][0])
        len_y = abs(center_point[1] - copied_mesh.bounds[0][1])

        min_x = center_point[0] - len_x * alpha
        max_x = center_point[0] + len_x * alpha
        min_y = center_point[1] - len_y * alpha
        max_y = center_point[1] + len_y * alpha
        margin = (min_x, max_x, min_y, max_y)
        ###
        weights = self._get_weights_for_support_obj(copied_mesh)

        n_sample_sup_obj = self.n_samples_sup_obj

        if obj_name == "table":
            if self.scene_mngr.scene.bench_num == 1:
                n_sample_sup_obj = 40
            if self.scene_mngr.scene.bench_num == 3:
                n_sample_sup_obj = 20

        if obj_name == "tray_red":
            n_sample_sup_obj = 0

        sample_points, normals = self.get_surface_points_from_mesh(
            copied_mesh, n_sample_sup_obj, weights
        )
        normals = np.tile(np.array([0.0, 0.0, 1.0]), (normals.shape[0], 1))
        # TODO heuristic
        if self.scene_mngr.scene.bench_num != 4:
            if obj_name not in ["shelf_8", "shelf_9", "shelf_15"]:
                center_upper_point = np.zeros(3)
                center_upper_point[0] = center_point[0] + np.random.uniform(
                    -0.002, 0.002
                )
                center_upper_point[1] = center_point[1] + np.random.uniform(
                    -0.002, 0.002
                )
                center_upper_point[2] = copied_mesh.bounds[1, 2]
                sample_points = np.append(
                    sample_points, np.array([center_upper_point]), axis=0
                )
                normals = np.append(normals, np.array([[0, 0, 1]]), axis=0)
            for point, normal_vector in zip(sample_points, normals):
                yield point, normal_vector, margin
        else:
            if "table" in obj_name:
                normals = np.tile(np.array([0, 0, 1]), reps=(3, 1))
                sample_points = np.array(
                    [
                        np.array(self.scene_mngr.get_object_pose(peg)[:3, 3])
                        for peg in self.scene_mngr.scene.pegs
                    ]
                )
                table_height = (
                    self.scene_mngr.scene.objs["table"].gparam.bounds[1][2]
                    - self.scene_mngr.scene.objs["table"].gparam.bounds[0][2]
                )
                sample_points[:3, 2] = table_height

                for point, normal_vector in zip(sample_points, normals):
                    yield point, normal_vector, margin

            if "hanoi_disk" in obj_name:
                margin = (0, 0, 0, 0)
                sample_points = np.array([np.array(support_obj.h_mat[:3, 3])])
                hanoi_disk_height = (
                    support_obj.h_mat[2, 3]
                    + (
                        support_obj.gparam.bounds[1][2]
                        - support_obj.gparam.bounds[0][2]
                    )
                    / 2
                )
                sample_points[:3, 2] = hanoi_disk_height
                normals = np.array([[0, 0, 1]])

                for point, normal_vector in zip(sample_points, normals):
                    yield point, normal_vector, margin

    def get_current_surface_points_from_support_obj(self, obj_name, alpha=0.2):
        support_obj = self.scene_mngr.scene.objs[obj_name]
        copied_mesh = deepcopy(support_obj.gparam)
        copied_mesh.apply_transform(support_obj.h_mat)
        center_point = copied_mesh.center_mass

        len_x = abs(center_point[0] - copied_mesh.bounds[0][0])
        len_y = abs(center_point[1] - copied_mesh.bounds[0][1])

        min_x = center_point[0] - len_x * alpha
        max_x = center_point[0] + len_x * alpha
        min_y = center_point[1] - len_y * alpha
        max_y = center_point[1] + len_y * alpha
        margin = (min_x, max_x, min_y, max_y)
        # Different from get_surface_points_for_held_obj
        weights = self._get_weights_for_want_normal(
            copied_mesh, support_obj.h_mat[:3, 2]
        )

        sample_points, normals = self.get_surface_points_from_mesh(
            copied_mesh, self.n_samples_sup_obj, weights
        )

        if "box" in obj_name:
            center_upper_point = self.compute_box_center_upper_point_from_box_pose(
                self.scene_mngr.scene.objs[obj_name].h_mat
            )
            print(
                "Support obj name : ",
                obj_name,
                "\nh_mat : \n",
                self.scene_mngr.scene.objs[obj_name].h_mat,
            )
        elif "tray" in obj_name:
            center_upper_point = np.zeros(3)
            center_upper_point[0] = center_point[0] + np.random.uniform(-0.002, 0.002)
            center_upper_point[1] = center_point[1] + np.random.uniform(-0.002, 0.002)
            center_upper_point[2] = copied_mesh.bounds[1, 2]
        else:
            center_upper_point, _ = self.get_surface_points_from_mesh(
                copied_mesh, 1, weights
            )
        sample_points = np.append(sample_points, np.array([center_upper_point]), axis=0)
        normals = np.append(normals, support_obj.h_mat[:3, 2], axis=0)
        for point, normal_vector in zip(sample_points, normals):
            yield point, normal_vector, margin

    @staticmethod
    def _get_weights_for_support_obj(obj_mesh):
        # heuristic
        weights = np.zeros(len(obj_mesh.faces))
        for idx, vertex in enumerate(obj_mesh.vertices[obj_mesh.faces]):
            weights[idx] = 0.0
            if np.all(vertex[:, 2] >= obj_mesh.bounds[1][2] * 0.999):
                weights[idx] = 1.0
        return weights

    def get_surface_points_for_held_obj(self, obj_name):
        copied_mesh = deepcopy(self.scene_mngr.init_objects[obj_name].gparam)
        copied_mesh.apply_translation(-copied_mesh.center_mass)
        copied_mesh.apply_transform(self.scene_mngr.scene.robot.gripper.pick_obj_pose)

        center_point = copied_mesh.center_mass

        center_lower_point = center_point
        center_lower_point[-1] = copied_mesh.bounds[0, 2]

        if self.scene_mngr.scene.bench_num == 2:
            if "bottle" in obj_name:
                sample_points = np.array([center_lower_point])
                normals = np.array([[0, 0, -1]])

                for point, normal_vector in zip(sample_points, normals):
                    yield point, normal_vector

        elif self.scene_mngr.scene.bench_num == 4:
            if "hanoi_disk" in obj_name:
                sample_points = np.array([center_lower_point])
                normals = np.array([[0, 0, -1]])

                for point, normal_vector in zip(sample_points, normals):
                    yield point, normal_vector
        else:

            weights = self._get_weights_for_held_obj(copied_mesh)

            # self.n_samples_held_obj=1
            sample_points, normals = self.get_surface_points_from_mesh(
                copied_mesh, self.n_samples_held_obj, weights
            )

            # heuristic
            sample_points = np.append(
                sample_points, np.array([center_lower_point]), axis=0
            )
            normals = np.append(normals, np.array([[0, 0, -1]]), axis=0)
            # print(sample_points, normals)
            for point, normal_vector in zip(sample_points, normals):
                yield point, normal_vector

    def compute_box_center_upper_point_from_box_pose(self, obj_pose=np.eye(4)):
        box_bounds = 0.05
        obj_lower_pose = np.eye(4)
        obj_lower_pose[:3, :3] = obj_pose[:3, :3]
        obj_lower_pose[:3, 3] = obj_pose[:3, 3] + np.dot(box_bounds, obj_pose[:3, 2])

    def compute_box_center_lower_point_from_box_pose(self, obj_pose=np.eye(4)):
        """
        Function that calculates center_lower_point considering box rotation

        box_bouds : half the length of a side of a square box
        """
        box_bounds = 0.05
        obj_lower_pose = np.eye(4)
        obj_lower_pose[:3, :3] = obj_pose[:3, :3]
        obj_lower_pose[:3, 3] = obj_pose[:3, 3] - np.dot(box_bounds, obj_pose[:3, 2])
        return obj_lower_pose

    def get_current_surface_points_for_held_obj(self, obj_name):

        copied_mesh = deepcopy(self.scene_mngr.init_objects[obj_name].gparam)
        copied_mesh.apply_translation(-copied_mesh.center_mass)
        # Different from get_surface_points_for_held_obj
        copied_mesh.apply_transform(self.scene_mngr.scene.objs[obj_name].h_mat)

        center_point = copied_mesh.center_mass
        center_lower_point_old = center_point

        # Different from get_surface_points_for_held_obj
        obj_center_lower_point = self.compute_box_center_lower_point_from_box_pose(
            self.scene_mngr.scene.objs[obj_name].h_mat
        )

        points = np.array([obj_center_lower_point[:3, 3]])
        normals = np.array([-obj_center_lower_point[:3, 2]])

        # weight from the face of the desired normal
        # weights = self._get_weights_for_want_normal(copied_mesh, normals[0])
        weights = self._get_weights_for_held_obj(copied_mesh)

        sample_points, sampled_normals = self.get_surface_points_from_mesh(
            copied_mesh, self.n_samples_held_obj, weights
        )

        normals = np.vstack([normals, sampled_normals])
        points = np.vstack([points, sample_points])

        for point, normal_vector in zip(points, normals):
            yield point, normal_vector

    @staticmethod
    def _get_weights_for_held_obj(obj_mesh):
        # heuristic
        weights = np.zeros(len(obj_mesh.faces))
        for idx, vertex in enumerate(obj_mesh.vertices[obj_mesh.faces]):
            weights[idx] = 0.4
            if np.all(vertex[:, 2] <= obj_mesh.bounds[0][2] * 1.02):

                weights[idx] = 0.6
        return weights

    @staticmethod
    def _get_weights_for_want_normal(obj_mesh, normal):
        error = np.square(np.subtract(obj_mesh.face_normals, normal)).sum(axis=1)
        weights = np.where(error < 1e-5, 1, 0)

        return weights

    def get_transformed_eef_poses(self, support_obj_name, held_obj_name, eef_pose=None):
        bench_num = self.scene_mngr.scene.bench_num
        alpha = 1

        if bench_num == 1:
            alpha = 0.9
        elif bench_num == 2:
            alpha = 1
        elif bench_num == 3:
            alpha = 0.9
        elif bench_num == 4:
            alpha = 0.3

        held_obj_pose = deepcopy(self.scene_mngr.scene.objs[held_obj_name].h_mat)
        # held_obj_pose = deepcopy(self.scene_mngr.scene.robot.gripper.pick_obj_pose)
        surface_points_for_sup_obj = list(
            self.get_surface_points_for_support_obj(support_obj_name, alpha=alpha)
        )

        # surface_points_for_sup_obj = list(self.get_current_surface_points_from_support_obj(support_obj_name, alpha=alpha))
        # surface_points_for_held_obj = list(self.get_surface_points_for_held_obj(held_obj_name))
        # chaned
        surface_points_for_held_obj = list(
            self.get_current_surface_points_for_held_obj(held_obj_name)
        )

        for (
            support_obj_point,
            support_obj_normal,
            (min_x, max_x, min_y, max_y),
        ) in surface_points_for_sup_obj:
            for held_obj_point, held_obj_normal in surface_points_for_held_obj:
                # print('normals',support_obj_normal, held_obj_normal)
                # rot_mat : held_obj_normal에서 얼마나 돌아야 -support_obj_normal 이 나오는지!
                rot_mat = m_utils.get_rotation_from_vectors(
                    held_obj_normal, -support_obj_normal
                )
                # print(rot_mat,'rotmat')
                # object 중심으로부터 sample_point까지 떨어진 xyz 좌표값
                held_obj_point_transformed = np.dot(
                    held_obj_point - held_obj_pose[:3, 3], rot_mat
                )
                # place 해야할 위치 얻어줌!! object가 기울어진 평면 위에 있을 경우 잘못 된 결과 나옴...
                (
                    held_obj_pose_transformed,
                    held_obj_point_transformed,
                ) = self._get_obj_pose_transformed(
                    held_obj_pose,
                    support_obj_point,
                    held_obj_point_transformed,
                    rot_mat,
                )

                # heuristic
                copied_mesh = deepcopy(
                    self.scene_mngr.init_objects[held_obj_name].gparam
                )
                copied_mesh.apply_transform(held_obj_pose_transformed)
                center_point = copied_mesh.center_mass

                if bench_num == 1:
                    if "table" in support_obj_name:
                        if not (min_x + 0.1 <= center_point[0] <= max_x - 0.3):
                            continue
                        if not (min_y + 0.1 <= center_point[1] <= max_y - 0.2):
                            continue
                if bench_num == 2:
                    center_point = (
                        copied_mesh.bounds[0]
                        + (copied_mesh.bounds[1] - copied_mesh.bounds[0]) / 2
                    )
                    if "shelf_8" in support_obj_name:
                        if not (min_x + 0.05 <= center_point[0] <= max_x - 0.2):
                            continue
                        if not (min_y + 0.1 <= center_point[1] <= max_y - 0.1):
                            continue
                    if "shelf_9" in support_obj_name:
                        if not (min_x + 0.02 <= center_point[0] <= max_x - 0.1):
                            continue
                        if not (min_y + 0.05 <= center_point[1] <= max_y - 0.7):
                            continue
                    if "shelf_15" in support_obj_name:
                        if not (min_x + 0.05 <= center_point[0] <= max_x - 0.2):
                            continue
                        if not (min_y + 0.2 <= center_point[1] <= max_y - 0.1):
                            continue
                if bench_num == 3:
                    if "table" in support_obj_name:
                        if not (min_x + 0.1 <= center_point[0] <= max_x - 0.2):
                            continue
                        if not (min_y <= center_point[1] <= max_y - 0.3):
                            continue

                if eef_pose is not None:
                    # ? 얘는 진짜 무슨 역할하는지 몰겠음 'ㅡ'
                    # T_obj_pose_and_obj_pose_transformed = np.dot(held_obj_pose, np.linalg.inv(held_obj_pose_rotated))
                    # T_obj_pose_and_obj_pose_transformed = np.eye(4)
                    # T_obj_pose_and_obj_pose_transformed[:3,:3] = deepcopy(held_obj_pose_transformed[:3,:3])
                    # object를 place하기 위한 eef 위치
                    eef_pose_transformed = self._get_eef_pose_transformed(
                        held_obj_pose_transformed
                    )

                    yield eef_pose_transformed, held_obj_pose_transformed
                else:
                    yield None, held_obj_pose_transformed

    @staticmethod
    def _get_obj_pose_transformed(
        held_obj_pose, sup_obj_point, held_obj_point_transformed, rot_mat
    ):
        # 뭐하는 변수지 rotation matrix로 내적 해주나마나 똑같은데? object 기울어진 채로  시작할때 사용되나 ;;
        obj_pose_rotated = np.eye(4)
        obj_pose_rotated[:3, :3] = np.dot(rot_mat, held_obj_pose[:3, :3])
        obj_pose_rotated[:3, 3] = held_obj_pose[:3, 3]

        # held object가 sub object 위에 place 될 위치 !!
        obj_pose_transformed = np.eye(4)
        # 궂이 계산 안하고 바로 support_obj_normal으로 대입해도 되는데 !!??!?
        obj_pose_transformed[:3, :3] = np.dot(
            held_obj_pose[:3, :3], np.linalg.inv(rot_mat)
        )
        # obj_pose_transformed[:3, :3] =np.dot(np.linalg.inv(rot_mat),held_obj_pose[:3, :3])
        # object가 place되어야 할 위치 !! 여기서 sampled point의 translatio을 고려해줌
        obj_pose_transformed[:2, 3] = sup_obj_point[:2] + held_obj_point_transformed[:2]
        obj_pose_transformed[2, 3] = sup_obj_point[2] - held_obj_point_transformed[2]
        return obj_pose_transformed, obj_pose_rotated

    def _get_eef_pose_transformed(self, held_obj_pose_transformed):
        g_T_o = self.scene_mngr.scene.robot.gripper.transform_bet_gripper_n_obj
        w_T_o = held_obj_pose_transformed
        # eef_pose_transformed = w_T_g
        eef_pose_transformed = np.dot(w_T_o, np.linalg.inv(g_T_o))
        return eef_pose_transformed

    # def _get_eef_pose_transformed(self, T, eef_pose, sup_obj_point, held_obj_point_transformed):
    #     eef_pose_transformed = np.dot(T, eef_pose)

    #     result_eef_pose_transformed = np.eye(4)
    #     result_eef_pose_transformed[:3, :3] = eef_pose_transformed[:3, :3]
    #     result_eef_pose_transformed[:3, 3] = eef_pose_transformed[:3, 3] + (sup_obj_point - held_obj_point_transformed) + np.array([0, 0, self.release_distance])
    #     return result_eef_pose_transformed

    @staticmethod
    def _check_stability(
        copied_scene: Scene, held_obj_name: str, com: np.ndarray
    ) -> bool:
        if copied_scene.logical_state.on in list(
            copied_scene.logical_states[held_obj_name].keys()
        ):
            support_obj = copied_scene.logical_states[held_obj_name][
                copied_scene.logical_state.on
            ]
            support_obj_mesh: Trimesh = deepcopy(support_obj.gparam)
            support_obj_mesh.apply_transform(support_obj.h_mat)

            closest_points, _, _ = proximity.closest_point(support_obj_mesh, [com])
            closest_point = closest_points[0]
            norm_vector = m_utils.normalize(closest_point - com)

            safe_com = norm_vector[2] == -1.0
            if safe_com:
                return PlaceAction._check_stability(copied_scene, support_obj.name, com)
            else:
                return False
        else:
            return True

    def _check_support(self, support_obj_name, held_obj_name, obj_pose):
        """
        Check support pose
        Args:
            support_obj_name (str) : support_object_name
            held_obj_name (str)  : held_obj_name
            obj_pose (np.array): pose of support object

        Returns:
            bool: If satisfy support pose, then true
                  Otherwise then false
        """
        held_obj = self.scene_mngr.scene.objs[held_obj_name]
        held_obj_mesh: Trimesh = deepcopy(held_obj.gparam)
        held_obj_mesh.apply_transform(obj_pose)

        place_obj = self.scene_mngr.scene.objs[support_obj_name]
        place_obj_mesh: Trimesh = deepcopy(place_obj.gparam)
        place_obj_mesh.apply_transform(place_obj.h_mat)

        locations, _, _ = place_obj_mesh.ray.intersects_location(
            ray_origins=[held_obj_mesh.center_mass], ray_directions=[[0, 0, -1]]
        )
        if len(locations) != 0:
            return True
        return False

    def _get_peg(self, y_pose):
        y_pose = np.round(y_pose, 1)
        if y_pose < 0.0:
            peg = self.scene_mngr.scene.pegs[2]
        elif y_pose > 0.0:
            peg = self.scene_mngr.scene.pegs[0]
        else:
            peg = self.scene_mngr.scene.pegs[1]
        return peg
