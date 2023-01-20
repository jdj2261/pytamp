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

pick_actions = list(pick.get_possible_actions_level_1())

for pick_action in pick_actions:
    for pick_scene in pick.get_possible_transitions(
        pick.scene_mngr.scene, action=pick_action
    ):
        place_actions = list(place.get_possible_actions_level_1(pick_scene))
        for place_action in place_actions:
            for place_scene in place.get_possible_transitions(
                scene=pick_scene, action=place_action
            ):
                fig, ax = p_utils.init_3d_figure(name="all possible pick transitions")
                place.scene_mngr.render_gripper(
                    ax, pick_scene, alpha=0.9, only_visible_axis=False
                )
                pick_scene.show_logical_states()
                place.scene_mngr.render_objects(ax, pick_scene)
                ax.view_init(20, 150, "z")
                place.scene_mngr.show()

                fig, ax = p_utils.init_3d_figure(name="all possible place transitions")
                place.scene_mngr.render_gripper(
                    ax, place_scene, alpha=0.9, only_visible_axis=False
                )
                place_scene.show_logical_states()
                place.scene_mngr.render_objects(ax, place_scene)
                ax.view_init(20, 150, "z")
                place.scene_mngr.show()
