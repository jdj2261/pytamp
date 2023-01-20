from pytamp.benchmark import Benchmark1
from pykin.utils import plot_utils as p_utils
from pytamp.action.pick import PickAction

benchmark1 = Benchmark1(robot_name="panda", geom="visual", is_pyplot=True)
pick = PickAction(benchmark1.scene_mngr, n_contacts=0, n_directions=0)

################## Transitions Test Action 1 ##################
actions = list(pick.get_possible_actions_level_1())
for action in actions:
    for idx, pick_scene in enumerate(
        pick.get_possible_transitions(pick.scene_mngr.scene, action=action)
    ):
        fig, ax = p_utils.init_3d_figure(name="all possible transitions")
        pick.scene_mngr.render_gripper(
            ax, pick_scene, alpha=0.9, only_visible_axis=False
        )
        pick.scene_mngr.render_objects(ax, pick_scene)
        pick_scene.show_logical_states()
        pick.scene_mngr.show()
