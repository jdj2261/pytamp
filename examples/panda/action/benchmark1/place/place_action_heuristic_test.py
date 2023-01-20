from pytamp.benchmark import Benchmark1
from pykin.utils import plot_utils as p_utils
from pytamp.action.pick import PickAction
from pytamp.action.place import PlaceAction

benchmark1 = Benchmark1(robot_name="panda", geom="visual", is_pyplot=True)
pick = PickAction(benchmark1.scene_mngr, n_contacts=0, n_directions=0)
place = PlaceAction(
    benchmark1.scene_mngr, n_samples_held_obj=0, n_samples_support_obj=0
)

################# Action Test ##################
fig, ax = p_utils.init_3d_figure(name="Level wise 1")
pick_action = pick.get_action_level_1_for_single_object(
    pick.scene_mngr.init_scene, "A_box"
)

for grasp_pose in pick_action[pick.info.GRASP_POSES]:
    pick.scene_mngr.render_axis(ax, grasp_pose[pick.move_data.MOVE_grasp])

for pick_scene in pick.get_possible_transitions(
    pick.scene_mngr.init_scene, pick_action
):
    place_action = place.get_action_level_1_for_single_object(
        "tray_red", "A_box", pick_scene.robot.gripper.grasp_pose, scene=pick_scene
    )
    for release_pose, obj_pose in place_action[place.info.RELEASE_POSES]:
        place.scene_mngr.render.render_axis(
            ax, release_pose[place.move_data.MOVE_release]
        )
        place.scene_mngr.render.render_object(
            ax, place.scene_mngr.scene.objs["A_box"], obj_pose
        )
        place.scene_mngr.render.render_gripper(
            ax, benchmark1.robot, pose=release_pose[place.move_data.MOVE_release]
        )
place.scene_mngr.render_objects(ax)
p_utils.plot_basis(ax)
place.show()
