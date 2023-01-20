from pykin.utils import plot_utils as p_utils
from pytamp.action.pick import PickAction
from pytamp.action.place import PlaceAction
from pytamp.benchmark import Benchmark3

benchmark3 = Benchmark3(robot_name="doosan", geom="collision", is_pyplot=True)
pick = PickAction(
    benchmark3.scene_mngr, n_contacts=3, n_directions=5, retreat_distance=0.1
)
place = PlaceAction(
    benchmark3.scene_mngr, n_samples_held_obj=0, n_samples_support_obj=20
)

################# Action Test ##################
pick_actions = list(pick.get_possible_actions_level_1())

for pick_action in pick_actions:
    fig, ax = p_utils.init_3d_figure(name="Level wise 1")
    for pick_scene in pick.get_possible_transitions(
        pick.scene_mngr.scene, action=pick_action
    ):
        place_actions = list(place.get_possible_actions_level_1(pick_scene))
        for place_action in place_actions:
            for all_release_pose, obj_pose in place_action[place.info.RELEASE_POSES]:
                print(
                    place.scene_mngr.scene.objs[
                        place.scene_mngr.scene.robot.gripper.attached_obj_name
                    ]
                )
                place.scene_mngr.render.render_object(
                    ax,
                    place.scene_mngr.scene.objs[
                        place.scene_mngr.scene.robot.gripper.attached_obj_name
                    ],
                    obj_pose,
                )
                place.scene_mngr.render.render_axis(
                    ax, all_release_pose[place.move_data.MOVE_release]
                )
                # place.scene_mngr.render.render_gripper(ax, benchmark3.robot, pose=all_release_pose[place.move_data.MOVE_release], alpha=0.1)
    place.scene_mngr.render_objects(ax, alpha=0.1)
    p_utils.plot_basis(ax, pick.scene_mngr.scene.robot)
    pick.show()
