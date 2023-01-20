from pytamp.benchmark import Benchmark1
from pykin.utils import plot_utils as p_utils
from pytamp.action.pick import PickAction
from pytamp.action.place import PlaceAction

benchmark1 = Benchmark1(robot_name="panda", geom="visual", is_pyplot=True, box_num=3)
pick = PickAction(benchmark1.scene_mngr, n_contacts=1, n_directions=1)
place = PlaceAction(
    benchmark1.scene_mngr, n_samples_held_obj=1, n_samples_support_obj=1
)

################# Action Test ##################
fig, ax = p_utils.init_3d_figure(name="Level wise 1")
pick_actions = list(pick.get_possible_actions_level_1())
for pick_action in pick_actions:
    for pick_scene in pick.get_possible_transitions(
        pick.scene_mngr.scene, action=pick_action
    ):
        place_actions = list(place.get_possible_actions_level_1(pick_scene))
        for place_action in place_actions:
            for all_release_pose, obj_pose in place_action[place.info.RELEASE_POSES]:
                place.scene_mngr.render.render_axis(
                    ax, all_release_pose[place.move_data.MOVE_release]
                )
                place.scene_mngr.render.render_gripper(
                    ax,
                    benchmark1.robot,
                    pose=all_release_pose[place.move_data.MOVE_release],
                )
place.scene_mngr.render_objects(ax)
p_utils.plot_basis(ax)
pick.show()
