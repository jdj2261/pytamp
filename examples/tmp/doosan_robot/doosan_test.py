from pykin.utils import plot_utils as p_utils
from pytamp.action.pick import PickAction
from pytamp.benchmark import Benchmark1

benchmark1 = Benchmark1(robot_name="doosan", geom="visual", is_pyplot=True, box_num=3)

pick = PickAction(benchmark1.scene_mngr, n_contacts=0, n_directions=0, retreat_distance=0.1)

################# Action Test ##################
fig, ax = p_utils.init_3d_figure(name="Heuristic")
for obj in ["A_box", "B_box", "C_box"]:
    pose = list(pick.get_grasp_pose_from_heuristic(obj_name=obj))
    for i in range(len(pose)):
        pick.scene_mngr.render_axis(ax, pose[i][pick.move_data.MOVE_grasp])
        pick.scene_mngr.set_gripper_pose(pose[i][pick.move_data.MOVE_grasp])
        pick.scene_mngr.render_axis(ax, pose=pick.scene_mngr.scene.robot.gripper.info["tcp"][3])
        pick.scene_mngr.render_gripper(ax)

pick.scene_mngr.render_objects(ax)
p_utils.plot_basis(ax)


# # ################# Action Test ##################
actions = list(pick.get_possible_actions_level_1())
fig, ax = p_utils.init_3d_figure(name="Level wise 1")
for pick_actions in actions:
    for all_grasp_pose in pick_actions[pick.info.GRASP_POSES]:
        pick.scene_mngr.render_axis(ax, all_grasp_pose[pick.move_data.MOVE_grasp])
        pick.scene_mngr.render.render_gripper(ax, benchmark1.robot, pose=all_grasp_pose[pick.move_data.MOVE_grasp])
        # pick.scene_mngr.render_axis(ax, all_grasp_pose[pick.move_data.MOVE_pre_grasp])
        # pick.scene_mngr.render_axis(ax, all_grasp_pose[pick.move_data.MOVE_post_grasp])
pick.scene_mngr.render_objects(ax)
p_utils.plot_basis(ax)
pick.show()

actions = list(pick.get_possible_actions_level_1())

pick_joint_all_path = []
pick_all_objects = []
pick_all_object_poses = []

success_joint_path = False
for pick_action in actions:
    for idx, pick_scene in enumerate(pick.get_possible_transitions(benchmark1.scene_mngr.scene, action=pick_action)):
        pick_joint_path = pick.get_possible_joint_path_level_2(scene=pick_scene, grasp_poses=pick_scene.grasp_poses)
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
        fig, ax = p_utils.init_3d_figure( name="Level wise 2")
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