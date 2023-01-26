from pykin.utils import plot_utils as p_utils
from pytamp.action.pick import PickAction
from pytamp.action.place import PlaceAction
from pytamp.benchmark import Benchmark4

benchmark4 = Benchmark4(robot_name="doosan", geom="visual", is_pyplot=True, disk_num=1)
pick = PickAction(
    benchmark4.scene_mngr, n_contacts=0, n_directions=0, retreat_distance=0.1
)
place = PlaceAction(
    benchmark4.scene_mngr, n_samples_held_obj=0, n_samples_support_obj=10
)

################# Action Test ##################
actions = list(pick.get_possible_actions_level_1())

###### Surface sampling held and support obj#######
fig, ax = p_utils.init_3d_figure(figsize=(10, 6), dpi=120, name="Sampling Object")
surface_points_for_support_obj = list(place.get_surface_points_for_support_obj("table"))
for point, normal, _ in surface_points_for_support_obj:
    place.scene_mngr.render.render_point(ax, point)
surface_points_for_held_obj = list(
    place.get_surface_points_for_held_obj("hanoi_disk_0")
)
for point, normal in surface_points_for_held_obj:
    place.scene_mngr.render.render_point(ax, point)
p_utils.plot_basis(ax)
place.scene_mngr.render_objects(ax, alpha=0.2)

##### All Release Pose #######
fig, ax = p_utils.init_3d_figure(name="Get Release Pose")
eef_poses = list(pick.get_all_grasp_poses("hanoi_disk_0"))

# Add heuristic
eef_poses.extend(list(pick.get_grasp_pose_from_heuristic("hanoi_disk_0")))

all_release_poses = []
for eef_pose in eef_poses:
    release_poses = list(
        place.get_all_release_poses_and_obj_pose(
            "table", "hanoi_disk_0", eef_pose[pick.move_data.MOVE_grasp]
        )
    )
    pick.scene_mngr.render.render_gripper(
        ax, benchmark4.robot, pose=eef_pose[place.move_data.MOVE_grasp]
    )
    for release_pose, obj_pose in release_poses:
        all_release_poses.append((release_pose, obj_pose))
        place.scene_mngr.render.render_axis(
            ax, release_pose[place.move_data.MOVE_release]
        )
        place.scene_mngr.render.render_gripper(
            ax, benchmark4.robot, pose=release_pose[place.move_data.MOVE_release]
        )
        # pick.scene_mngr.render_axis(ax, release_pose[place.move_data.MOVE_pre_release])
        # pick.scene_mngr.render_axis(ax, release_pose[place.move_data.MOVE_post_release])
        place.scene_mngr.render.render_object(
            ax, place.scene_mngr.scene.objs["hanoi_disk_0"], obj_pose
        )
p_utils.plot_basis(ax)
place.scene_mngr.render_objects(ax)

# # ###### Level wise - 1 #######
fig, ax = p_utils.init_3d_figure(name="Level wise 1")
release_poses_for_only_gripper = list(
    place.get_release_poses_not_collision(all_release_poses, False)
)
for release_pose_for_only_gripper, obj_pose in release_poses_for_only_gripper:
    place.scene_mngr.render.render_axis(
        ax, release_pose_for_only_gripper[place.move_data.MOVE_release], scale=0.05
    )
    pick.scene_mngr.render.render_gripper(
        ax,
        benchmark4.robot,
        pose=release_pose_for_only_gripper[place.move_data.MOVE_release],
    )
    # place.scene_mngr.render_gripper(ax, pose=release_pose_for_only_gripper[place.move_data.MOVE_release])
    place.scene_mngr.render.render_object(
        ax, place.scene_mngr.scene.objs["hanoi_disk_0"], obj_pose
    )
p_utils.plot_basis(ax)
place.scene_mngr.render_objects(ax)
place.show()
