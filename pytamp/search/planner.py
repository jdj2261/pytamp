import numpy as np

from pytamp.action.pick import PickAction
from pytamp.action.place import PlaceAction
from pytamp.search.node_data import NodeData
from pytamp.scene.scene import Scene
from pytamp.scene.scene_manager import SceneManager

from pykin.utils import plot_utils as p_utils
from pykin.utils.kin_utils import ShellColors as sc


class Planner:
    def __init__(self, scene_mngr: SceneManager):
        self.node_data = NodeData
        self.scene_mngr = scene_mngr
        self.scene_mngr.is_debug_mode = False
        self.state = scene_mngr.scene
        bench_num = self.scene_mngr.scene.bench_num

        if bench_num == 1:
            self.pick_action = PickAction(scene_mngr, n_contacts=0, n_directions=3)
            self.place_action = PlaceAction(
                scene_mngr,
                n_samples_held_obj=0,
                n_samples_support_obj=0,
                n_directions=3,
                release_distance=0.02,
                retreat_distance=0.15,
            )
        elif bench_num == 2:
            self.pick_action = PickAction(
                scene_mngr,
                n_contacts=0,
                limit_angle_for_force_closure=0.02,
                n_directions=3,
            )
            self.place_action = PlaceAction(
                scene_mngr, n_samples_held_obj=0, n_samples_support_obj=30
            )
        elif bench_num == 3:
            self.pick_action = PickAction(
                scene_mngr, n_contacts=0, n_directions=3, retreat_distance=0.15
            )
            self.place_action = PlaceAction(
                scene_mngr,
                n_samples_held_obj=0,
                n_samples_support_obj=0,
                retreat_distance=0.2,
                n_directions=3,
            )
        elif bench_num == 4:
            self.pick_action = PickAction(
                scene_mngr, n_contacts=0, n_directions=0, retreat_distance=0.15
            )
            self.place_action = PlaceAction(
                scene_mngr,
                n_samples_held_obj=0,
                n_samples_support_obj=0,
                retreat_distance=0.2,
                n_directions=3,
            )

    def do_planning(self, planner):
        pnp_all_joint_path = []
        pick_all_objects = []
        place_all_object_poses = []

        pnp_path = []
        pick_objects = []
        place_object_poses = []
        init_thetas = []
        place_scene = None

        for action, obj_name in planner:
            if "pick" in action:
                pick_obj = obj_name
                print(f"{sc.COLOR_BROWN}Pick{sc.ENDC} {pick_obj}")
                pick_actions = self.pick_action.get_action_level_1_for_single_object(
                    self.scene_mngr.scene, obj_name
                )
                pick_scenes = list(
                    self.pick_action.get_possible_transitions(
                        self.scene_mngr.scene, pick_actions
                    )
                )
                pick_scene: Scene = np.random.choice(pick_scenes)
                if not init_thetas:
                    init_theta = self.pick_action.scene_mngr.scene.robot.init_qpos
                pick_joint_path = self.pick_action.get_possible_joint_path_level_2(
                    scene=pick_scene,
                    grasp_poses=pick_scene.grasp_poses,
                    init_thetas=init_theta,
                )
                if pick_joint_path:
                    success_pick = True
                    init_theta = pick_joint_path[-1][
                        self.pick_action.move_data.MOVE_default_grasp
                    ][-1]
                self.scene_mngr.scene = pick_scene
                # self.render_state("Pick", pick_scene)
            if "place" in action:
                place_obj = obj_name
                print(f"{sc.COLOR_BROWN}Place{sc.ENDC} {pick_obj} on {place_obj}")
                place_actions = self.place_action.get_action_level_1_for_single_object(
                    place_obj,
                    pick_obj,
                    self.scene_mngr.scene.robot.gripper.grasp_pose,
                    self.scene_mngr.scene,
                )
                place_scenes = list(
                    self.place_action.get_possible_transitions(
                        self.scene_mngr.scene, place_actions
                    )
                )
                place_scene: Scene = np.random.choice(place_scenes)
                place_joint_path = self.place_action.get_possible_joint_path_level_2(
                    scene=place_scene,
                    release_poses=place_scene.release_poses,
                    init_thetas=init_theta,
                )
                if place_joint_path:
                    pnp_path += pick_joint_path + place_joint_path
                    pick_objects.append(pick_scene.robot.gripper.attached_obj_name)
                    place_object_poses.append(
                        place_scene.objs[place_scene.pick_obj_name].h_mat
                    )
                    init_theta = place_joint_path[-1][
                        self.place_action.move_data.MOVE_default_release
                    ][-1]
                self.scene_mngr.scene = place_scene

        pnp_all_joint_path.append(pnp_path)
        pick_all_objects.append(pick_objects)
        place_all_object_poses.append(place_object_poses)
        # self.place_action.simulate_path(pnp_all_joint_path, pick_all_objects, place_all_object_poses, is_save=True, video_name="benchmark1")

        return pnp_all_joint_path, pick_all_objects, place_all_object_poses

    def render_state(self, title, state: Scene, close_gripper=None):
        ax = None
        if self.scene_mngr.is_pyplot is True:
            fig, ax = p_utils.init_3d_figure(name=title)

        if close_gripper is not None:
            if close_gripper:
                state.robot.close_gripper(0.01)
                if "milk" in state.pick_obj_name:
                    state.robot.close_gripper(0.04)
                if "can" in state.pick_obj_name:
                    state.robot.close_gripper(0.03)
                if "disk" in state.pick_obj_name:
                    state.robot.close_gripper(0.04)
        self.pick_action.scene_mngr.render_objects_and_gripper(ax, state)
        self.pick_action.show()

        if close_gripper is not None:
            if close_gripper:
                state.robot.open_gripper(0.01)
                if "milk" in state.pick_obj_name:
                    state.robot.open_gripper(0.04)
                if "can" in state.pick_obj_name:
                    state.robot.open_gripper(0.03)
                if "disk" in state.pick_obj_name:
                    state.robot.open_gripper(0.04)


if __name__ == "__main__":
    pass
