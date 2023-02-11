from abc import abstractclassmethod, ABCMeta
from dataclasses import dataclass
from copy import deepcopy

from pykin.utils.mesh_utils import surface_sampling
from pykin.utils import plot_utils as p_utils
from pytamp.planners.cartesian_planner import CartesianPlanner
from pytamp.planners.rrt_star_planner import RRTStarPlanner
from pytamp.planners.prm_star_planner import PRMStarPlanner

from pytamp.scene.scene_manager import SceneManager


@dataclass
class ActionInfo:
    TYPE = "type"
    PICK_OBJ_NAME = "pick_obj_name"
    HELD_OBJ_NAME = "held_obj_name"
    PLACE_OBJ_NAME = "place_obj_name"
    GRASP_POSES = "grasp_poses"
    TCP_POSES = "tcp_poses"
    RELEASE_POSES = "release_poses"
    LEVEL = "level"


@dataclass
class MoveData:
    """
    Grasp Status Enum class
    """

    MOVE_pre_grasp = "pre_grasp"
    MOVE_grasp = "grasp"
    MOVE_post_grasp = "post_grasp"
    MOVE_default_grasp = "default_grasp"

    MOVE_pre_release = "pre_release"
    MOVE_release = "release"
    MOVE_post_release = "post_release"
    MOVE_default_release = "default_release"


class ActivityBase(metaclass=ABCMeta):
    """
    Activity Base class

    Args:

    """

    def __init__(self, scene_mngr: SceneManager):
        self.scene_mngr = scene_mngr.deepcopy_scene(scene_mngr)
        self.info = ActionInfo
        self.move_data = MoveData

        if self.scene_mngr.scene.robot is not None:
            self.cartesian_planner = CartesianPlanner(
                dimension=self.scene_mngr.scene.robot.arm_dof
            )
            self.rrt_planner = RRTStarPlanner(
                delta_distance=0.05,
                epsilon=0.2,
                gamma_RRT_star=2.0,
                dimension=self.scene_mngr.scene.robot.arm_dof,
            )
            self.prm_planner = PRMStarPlanner(
                dimension=self.scene_mngr.scene.robot.arm_dof
            )

    def __repr__(self) -> str:
        return "pytamp.action.activity.{}()".format(type(self).__name__)

    @abstractclassmethod
    def get_possible_actions_level_1(self):
        raise NotImplementedError

    @abstractclassmethod
    def get_action_level_1_for_single_object(self):
        raise NotImplementedError

    @abstractclassmethod
    def get_possible_ik_solve_level_2(self):
        raise NotImplementedError

    @abstractclassmethod
    def get_possible_joint_path_level_2(self):
        raise NotImplementedError

    @abstractclassmethod
    def get_possible_transitions(self):
        raise NotImplementedError

    def get_surface_points_from_mesh(self, mesh, n_sampling=100, weights=None):
        contact_points, _, normals = surface_sampling(mesh, n_sampling, weights)
        return contact_points, normals

    def _collide(self, is_only_gripper: bool) -> bool:
        collide = False
        if is_only_gripper:
            collide = self.scene_mngr.collide_objs_and_gripper()
        else:
            collide = self.scene_mngr.collide_objs_and_robot()
        return collide

    def _solve_ik(self, pose1, pose2, eps=1e-2):
        pose_error = self.scene_mngr.scene.robot.get_pose_error(pose1, pose2)
        if pose_error < eps:
            return True
        return False

    def check_obj_in_scene(self, obj_name: str):
        assert obj_name in self.scene_mngr.scene.objs, "{} is not in scene".format(
            obj_name
        )

    def check_obj_support_static(self, obj_name) -> bool:
        # In get_possible_action_level_1, filtering is performed if the object to be picked supports another object..!
        # In this function, we can get possible action even support 1 object! but 2 object is not
        self.check_obj_in_scene(obj_name)
        static = self.scene_mngr.scene.logical_states[obj_name].get(
            self.scene_mngr.scene.logical_state.static
        )
        if static:
            return False

        sup = self.scene_mngr.scene.logical_states[obj_name].get(
            self.scene_mngr.scene.logical_state.support
        )
        if sup:
            if isinstance(sup, list):
                for obj in sup:
                    return not self.scene_mngr.scene.logical_states[obj.name].get(
                        self.scene_mngr.scene.logical_state.support
                    )
            else:
                return not self.scene_mngr.scene.logical_states[obj.name].get(
                    self.scene_mngr.scene.logical_state.support
                )
        return True

    def deepcopy_scene(self, scene=None):
        if scene is None:
            scene = self.scene_mngr.scene
        self.scene_mngr.scene = deepcopy(scene)

    def get_cartesian_path(self, cur_q, goal_pose, n_step=100, collision_check=False):
        self.cartesian_planner._n_step = n_step
        self.cartesian_planner.run(
            self.scene_mngr,
            cur_q,
            goal_pose,
            resolution=0.1,
            collision_check=collision_check,
        )
        return self.cartesian_planner.get_unique_joint_path()

    def get_rrt_star_path(
        self, cur_q, goal_pose=None, goal_q=None, max_iter=500, n_step=10
    ):
        self.rrt_planner.run(
            self.scene_mngr, cur_q, goal_pose, goal_q=goal_q, max_iter=max_iter
        )
        return self.rrt_planner.get_joint_path(n_step=n_step)

    def get_prm_star_path(
        self, cur_q, goal_pose=None, goal_q=None, max_iter=500, n_step=10
    ):
        self.prm_planner.run(
            self.scene_mngr, cur_q, goal_pose, goal_q=goal_q, max_iter=max_iter
        )
        return self.prm_planner.get_joint_path(n_step=n_step)

    def animation_path(
        self,
        pnp_all_joint_path,
        pick_all_objects,
        place_all_object_poses,
        visible_path=False,
        fig=None,
        ax=None,
        is_save=False,
        video_name="test",
        fps=60,
    ):

        pass

    def simulate_path(
        self,
        pnp_all_joint_path,
        pick_all_objects,
        place_all_object_poses,
        visible_path=False,
        fig=None,
        ax=None,
        is_save=False,
        video_name="test",
        fps=60,
        gif=False,
    ):
        # assert pnp_all_joint_path[0].any(), f"Cannot simulate joint path"

        self.scene_mngr.is_pyplot = True
        eef_poses = None
        for pnp_joint_all_path, pick_all_object, place_all_object_pose in zip(
            pnp_all_joint_path, pick_all_objects, place_all_object_poses
        ):
            result_joint = []
            eef_poses = []
            attach_idxes = []
            detach_idxes = []
            attach_idx = 0
            detach_idx = 0
            grasp_task_idx = 0
            post_grasp_task_idx = 0
            release_task_idx = 0
            post_release_task_idx = 0
            idx = 0

            for pnp_joint_path in pnp_joint_all_path:
                for _, (task, joint_path) in enumerate(pnp_joint_path.items()):
                    for _, joint in enumerate(joint_path):
                        idx += 1

                        if task == self.move_data.MOVE_grasp:
                            grasp_task_idx = idx
                        if task == self.move_data.MOVE_post_grasp:
                            post_grasp_task_idx = idx
                        if post_grasp_task_idx - grasp_task_idx == 1:
                            attach_idx = grasp_task_idx
                            attach_idxes.append(attach_idx)

                        if task == self.move_data.MOVE_release:
                            release_task_idx = idx
                        if task == self.move_data.MOVE_post_release:
                            post_release_task_idx = idx
                        if post_release_task_idx - release_task_idx == 1:
                            detach_idx = release_task_idx
                            detach_idxes.append(detach_idx)

                        result_joint.append(joint)
                        fk = self.scene_mngr.scene.robot.forward_kin(joint)
                        if visible_path:
                            eef_poses.append(
                                fk[self.scene_mngr.scene.robot.eef_name].pos
                            )

            if ax is None and fig is None:
                fig, ax = p_utils.init_3d_figure(name="Level wise 2")

            self.scene_mngr.animation(
                ax,
                fig,
                init_scene=self.scene_mngr.init_scene,
                joint_path=result_joint,
                eef_poses=eef_poses,
                visible_gripper=True,
                visible_text=False,
                alpha=1.0,
                interval=1,  # ms
                repeat=False,
                pick_object=pick_all_object,
                attach_idx=attach_idxes,
                detach_idx=detach_idxes,
                place_obj_pose=place_all_object_pose,
                is_save=is_save,
                video_name=video_name,
                fps=fps,
                gif=gif,
            )

    def show(self):
        self.scene_mngr.show()
