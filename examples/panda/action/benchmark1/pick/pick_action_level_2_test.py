from pytamp.benchmark import Benchmark1

# from pykin.utils import plot_utils as p_utils
from pytamp.action.pick import PickAction

benchmark1 = Benchmark1(robot_name="panda", geom="visual", is_pyplot=True)
pick = PickAction(benchmark1.scene_mngr, n_contacts=0, n_directions=0)

################# Action Test ##################
actions = list(pick.get_possible_actions_level_1())

pick_all_joint_path = []
pick_all_objects = []
pick_all_object_poses = []

pick_path = []
pick_objects = []
pick_object_poses = []

success_joint_path = False
for pick_action in actions:
    for idx, pick_scene in enumerate(
        pick.get_possible_transitions(pick.scene_mngr.scene, action=pick_action)
    ):
        pick_joint_path = pick.get_possible_joint_path_level_2(
            scene=pick_scene, grasp_poses=pick_scene.grasp_poses
        )
        if pick_joint_path:
            success_joint_path = True
            pick_path += pick_joint_path
            pick_objects.append(pick.scene_mngr.attached_obj_name)
            pick_object_poses.append(pick.scene_mngr.scene.robot.gripper.pick_obj_pose)
        if success_joint_path:
            break
    if success_joint_path:
        break

pick_all_joint_path.append(pick_path)
pick_all_objects.append(pick_objects)
pick_all_object_poses.append(pick_object_poses)

pick.simulate_path(pick_all_joint_path, pick_all_objects, pick_all_object_poses)
