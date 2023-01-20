from pykin.utils import plot_utils as p_utils
from pytamp.action.pick import PickAction
from pytamp.action.place import PlaceAction
from pytamp.benchmark import Benchmark2

benchmark2 = Benchmark2(
    robot_name="doosan", geom="visual", is_pyplot=True, bottle_num=1
)
pick = PickAction(
    benchmark2.scene_mngr, n_contacts=0, n_directions=0, retreat_distance=0.1
)
place = PlaceAction(
    benchmark2.scene_mngr, n_samples_held_obj=0, n_samples_support_obj=10
)

################# Action Test ##################
fig, ax = p_utils.init_3d_figure(name="Level wise 1")
pick_action = pick.get_action_level_1_for_single_object(
    pick.scene_mngr.init_scene, "goal_bottle"
)

for grasp_pose in pick_action[pick.info.GRASP_POSES]:
    pick.scene_mngr.render_axis(ax, grasp_pose[pick.move_data.MOVE_grasp])

for pick_scene in pick.get_possible_transitions(
    pick.scene_mngr.init_scene, pick_action
):
    place_action = place.get_action_level_1_for_single_object(
        "shelf_9", "goal_bottle", pick_scene.robot.gripper.grasp_pose, scene=pick_scene
    )
    for release_pose, obj_pose in place_action[place.info.RELEASE_POSES]:
        place.scene_mngr.render.render_axis(
            ax, release_pose[place.move_data.MOVE_release]
        )
        place.scene_mngr.render.render_object(
            ax, place.scene_mngr.scene.objs["goal_bottle"], obj_pose
        )
        place.scene_mngr.render.render_gripper(
            ax, benchmark2.robot, pose=release_pose[place.move_data.MOVE_release]
        )
place.scene_mngr.render_objects(ax)
p_utils.plot_basis(ax)
place.show()
