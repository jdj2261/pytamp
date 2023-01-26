from pykin.utils import plot_utils as p_utils
from pytamp.action.pick import PickAction
from pytamp.action.place import PlaceAction
from pytamp.benchmark import Benchmark4

benchmark4 = Benchmark4(robot_name="doosan", geom="visual", is_pyplot=True, disk_num=3)
pick = PickAction(
    benchmark4.scene_mngr, n_contacts=0, n_directions=0, retreat_distance=0.1
)
place = PlaceAction(
    benchmark4.scene_mngr,
    n_samples_held_obj=0,
    n_samples_support_obj=10,
    n_directions=5,
)
fig, ax = p_utils.init_3d_figure(name="Benchmark")
benchmark4.scene_mngr.render_objects(ax)
benchmark4.scene_mngr.show()

place.scene_mngr.is_pyplot = False
################# Action Test ##################
pick_actions = list(pick.get_possible_actions_level_1())
fig, ax = p_utils.init_3d_figure(name="Level wise 1")
for pick_action in pick_actions:
    for pick_scene in pick.get_possible_transitions(
        pick.scene_mngr.scene, action=pick_action
    ):
        place_actions = list(place.get_possible_actions_level_1(pick_scene))
        for place_action in place_actions:
            for all_release_pose, obj_pose in place_action[place.info.RELEASE_POSES]:
                print(
                    place.scene_mngr.scene.robot.gripper.attached_obj_name,
                    pick_scene.cur_place_obj_name,
                )
                place.scene_mngr.set_gripper_pose(
                    all_release_pose[place.move_data.MOVE_release]
                )
                place.scene_mngr.set_object_pose(
                    place.scene_mngr.scene.robot.gripper.attached_obj_name, obj_pose
                )
                place.scene_mngr.render_axis(
                    ax, all_release_pose[place.move_data.MOVE_release]
                )
                place.scene_mngr.render_gripper(ax)
                place.scene_mngr.render_object(
                    ax,
                    place.scene_mngr.scene.objs[
                        place.scene_mngr.scene.robot.gripper.attached_obj_name
                    ],
                    obj_pose,
                )
                place.scene_mngr.render_objects(ax)
                p_utils.plot_basis(ax)
                place.show()
