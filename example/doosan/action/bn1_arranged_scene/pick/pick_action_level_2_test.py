from pytamp.benchmark import Benchmark1
from pykin.utils import plot_utils as p_utils
from pytamp.action.pick import PickAction

benchmark1 = Benchmark1(robot_name="doosan", geom="visual", is_pyplot=True)
pick = PickAction(benchmark1.scene_mngr, n_contacts=0, n_directions=0)

################# Action Test ##################
actions = list(pick.get_possible_actions_level_1())

pick_joint_all_path = []
pick_all_objects = []
pick_all_object_poses = []

success_joint_path = False
for pick_action in actions:
    for idx, pick_scene in enumerate(pick.get_possible_transitions(pick.scene_mngr.scene, action=pick_action)):
        pick_joint_path = pick.get_possible_joint_path_level_3(scene=pick_scene, grasp_poses=pick_scene.grasp_poses)
        if pick_joint_path:
            success_joint_path = True
            pick_joint_all_path.append(pick_joint_path)
            pick_all_objects.append(pick.scene_mngr.attached_obj_name)
            pick_all_object_poses.append(pick.scene_mngr.scene.robot.gripper.pick_obj_pose)
        if success_joint_path: 
            break
    if success_joint_path: 
        break

grasp_task_idx = 0
post_grasp_task_idx = 0
attach_idx = 0

for step, (all_joint_pathes, pick_object, pick_object_pose) in enumerate(zip(pick_joint_all_path, pick_all_objects, pick_all_object_poses)):
    for all_joint_path in all_joint_pathes:
        cnt = 0
        result_joint = []
        eef_poses = []
        fig, ax = p_utils.init_3d_figure( name="Level wise 3")
        for j, (task, joint_path) in enumerate(all_joint_path.items()):
            for k, joint in enumerate(joint_path):
                cnt += 1
                
                if task == "grasp":
                    grasp_task_idx = cnt
                if task == "post_grasp":
                    post_grasp_task_idx = cnt
                    
                if post_grasp_task_idx - grasp_task_idx == 1:
                    attach_idx = grasp_task_idx

                result_joint.append(joint)
                fk = pick.scene_mngr.scene.robot.forward_kin(joint)
                eef_poses.append(fk[pick.scene_mngr.scene.robot.eef_name].pos)

pick.scene_mngr.animation(
    ax,
    fig,
    joint_path=result_joint,
    eef_poses=eef_poses,
    visible_gripper=True,
    visible_text=True,
    alpha=1.0,
    interval=50,
    repeat=False,
    pick_object = [pick_all_objects[0]],
    attach_idx = [attach_idx],
    detach_idx = [])