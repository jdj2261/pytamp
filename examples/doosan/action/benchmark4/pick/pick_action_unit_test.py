from pykin.utils import plot_utils as p_utils
from pytamp.action.pick import PickAction
from pytamp.benchmark import Benchmark4

benchmark4 = Benchmark4(robot_name="doosan", geom="visual", is_pyplot=True, disk_num=5)
pick = PickAction(
    benchmark4.scene_mngr, n_contacts=3, n_directions=10, retreat_distance=0.1
)

###### All Contact Points #######
for obj in benchmark4.scene_mngr.scene.goal_objects:
    print(f"object: {obj}")
    fig, ax = p_utils.init_3d_figure(name="Get contact points")
    contact_points = pick.get_contact_points(obj_name=obj)
    pick.scene_mngr.render.render_points(ax, contact_points)
    pick.scene_mngr.render_objects(ax, alpha=0.5)
    p_utils.plot_basis(ax)

    ##### All Grasp Pose #######
    grasp_poses = list(pick.get_all_grasp_poses(obj))
    fig, ax = p_utils.init_3d_figure(name="Get all grasp pose")
    for grasp_pose in grasp_poses:
        pick.scene_mngr.render_axis(ax, grasp_pose[pick.move_data.MOVE_grasp])
    p_utils.plot_basis(ax)
    pick.scene_mngr.render_objects(ax)

    ##### Add Heuristic
    grasp_poses.extend(list(pick.get_grasp_pose_from_heuristic(obj)))

    # ###### Level wise - 1 #######
    fig, ax = p_utils.init_3d_figure(name="Level wise 1")
    grasp_poses_for_only_gripper = list(
        pick.get_all_grasp_poses_not_collision(grasp_poses)
    )
    for grasp_pose_for_only_gripper in grasp_poses_for_only_gripper:
        pick.scene_mngr.render_axis(
            ax, grasp_pose_for_only_gripper[pick.move_data.MOVE_grasp]
        )
        # pick.scene_mngr.render_axis(ax, grasp_pose_for_only_gripper[pick.move_data.MOVE_pre_grasp])
        # pick.scene_mngr.render_axis(ax, grasp_pose_for_only_gripper[pick.move_data.MOVE_post_grasp])
        pick.scene_mngr.render_gripper(
            ax, alpha=0.7, pose=grasp_pose_for_only_gripper[pick.move_data.MOVE_grasp]
        )
    pick.scene_mngr.render_objects(ax)
    p_utils.plot_basis(ax)
    pick.show()
