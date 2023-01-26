from pytamp.benchmark import Benchmark1
from pykin.utils import plot_utils as p_utils
from pytamp.action.pick import PickAction
from pytamp.action.place import PlaceAction

benchmark1 = Benchmark1(robot_name="doosan", geom="visual", is_pyplot=True, box_num=3)
pick = PickAction(benchmark1.scene_mngr, n_contacts=0, n_directions=0)
place = PlaceAction(
    benchmark1.scene_mngr, n_samples_held_obj=0, n_samples_support_obj=0
)

################# Action Test ##################
actions = list(pick.get_possible_actions_level_1())

###### Surface sampling held and support obj#######
fig, ax = p_utils.init_3d_figure(figsize=(10, 6), dpi=120, name="Sampling Object")
surface_points_for_support_obj = list(
    place.get_surface_points_for_support_obj("tray_red")
)
for point, normal, _ in surface_points_for_support_obj:
    place.scene_mngr.render.render_point(ax, point)
surface_points_for_held_obj = list(place.get_surface_points_for_held_obj("A_box"))
for point, normal in surface_points_for_held_obj:
    place.scene_mngr.render.render_point(ax, point)
p_utils.plot_basis(ax)
place.scene_mngr.render_objects(ax, alpha=0.5)

##### All Release Pose #######
fig, ax = p_utils.init_3d_figure(name="Get Release Pose")
eef_poses = list(pick.get_all_grasp_poses("A_box"))

# Add heuristic
eef_poses.extend(list(pick.get_grasp_pose_from_heuristic("A_box")))

all_release_poses = []
for eef_pose in eef_poses:
    release_poses = list(
        place.get_all_release_poses_and_obj_pose(
            "tray_red", "A_box", eef_pose[pick.move_data.MOVE_grasp]
        )
    )
    pick.scene_mngr.render.render_gripper(
        ax, place.scene_mngr.scene.robot, pose=eef_pose[place.move_data.MOVE_grasp]
    )
    for release_pose, obj_pose in release_poses:
        all_release_poses.append((release_pose, obj_pose))
        place.scene_mngr.render.render_axis(
            ax, release_pose[place.move_data.MOVE_release]
        )
        place.scene_mngr.render.render_gripper(
            ax,
            place.scene_mngr.scene.robot,
            pose=release_pose[place.move_data.MOVE_release],
        )
        # pick.scene_mngr.render_axis(ax, release_pose[place.move_data.MOVE_pre_release])
        # pick.scene_mngr.render_axis(ax, release_pose[place.move_data.MOVE_post_release])
        place.scene_mngr.render.render_object(
            ax, place.scene_mngr.scene.objs["A_box"], obj_pose
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
        place.scene_mngr.scene.robot,
        pose=release_pose_for_only_gripper[place.move_data.MOVE_release],
    )
    # place.scene_mngr.render_gripper(ax, pose=release_pose_for_only_gripper[place.move_data.MOVE_release])
    place.scene_mngr.render.render_object(
        ax, place.scene_mngr.scene.objs["A_box"], obj_pose
    )
p_utils.plot_basis(ax)
place.scene_mngr.render_objects(ax)
place.show()
