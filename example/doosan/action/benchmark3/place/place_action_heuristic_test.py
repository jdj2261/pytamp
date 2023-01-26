from pykin.utils import plot_utils as p_utils
from pytamp.action.pick import PickAction
from pytamp.action.place import PlaceAction
from pytamp.benchmark import Benchmark3

benchmark3 = Benchmark3(robot_name="doosan", geom="visual", is_pyplot=True)
pick = PickAction(
    benchmark3.scene_mngr, n_contacts=3, n_directions=5, retreat_distance=0.1
)
place = PlaceAction(
    benchmark3.scene_mngr, n_samples_held_obj=0, n_samples_support_obj=1
)

fig, ax = p_utils.init_3d_figure(name="Benchmark")
benchmark3.scene_mngr.render_objects(ax)
benchmark3.scene_mngr.show()

pick.scene_mngr.is_pyplot = False
################# Action Test ##################

for pick_obj in benchmark3.scene_mngr.scene.castle:
    fig, ax = p_utils.init_3d_figure(name="Level wise 1")
    for place_obj in ["table"]:
        pick_action = pick.get_action_level_1_for_single_object(
            pick.scene_mngr.init_scene, pick_obj
        )
        for grasp_pose in pick_action[pick.info.GRASP_POSES]:
            pick.scene_mngr.render_axis(ax, grasp_pose[pick.move_data.MOVE_grasp])
        for pick_scene in pick.get_possible_transitions(
            pick.scene_mngr.init_scene, pick_action
        ):
            place_action = place.get_action_level_1_for_single_object(
                place_obj,
                pick_obj,
                pick_scene.robot.gripper.grasp_pose,
                scene=pick_scene,
            )
            for release_pose, obj_pose in place_action[place.info.RELEASE_POSES]:
                print(place.scene_mngr.scene.objs[pick_obj])
                pick.scene_mngr.render_axis(
                    ax, release_pose[place.move_data.MOVE_release]
                )
                pick.scene_mngr.render_object(
                    ax, place.scene_mngr.scene.objs[pick_obj], obj_pose
                )
                pick.scene_mngr.set_gripper_pose(
                    release_pose[place.move_data.MOVE_release]
                )
                pick.scene_mngr.render_gripper(ax)
            # pick.scene_mngr.open_gripper(0.01)

    pick.scene_mngr.render_scene(ax, alpha=0.8)
    p_utils.plot_basis(ax)
    pick.show()
