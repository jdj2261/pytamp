from pytamp.benchmark import Benchmark1
from pykin.utils import plot_utils as p_utils
from pytamp.action.pick import PickAction

benchmark1 = Benchmark1(robot_name="panda", geom="visual", is_pyplot=True)
pick = PickAction(benchmark1.scene_mngr, n_contacts=0, n_directions=0)

################# Action Test ##################
actions = list(pick.get_possible_actions_level_1())
fig, ax = p_utils.init_3d_figure(name="Level wise 1")
for pick_actions in actions:
    for all_grasp_pose in pick_actions[pick.info.GRASP_POSES]:
        pick.scene_mngr.render_axis(ax, all_grasp_pose[pick.move_data.MOVE_grasp])
        pick.scene_mngr.render_axis(ax, all_grasp_pose[pick.move_data.MOVE_pre_grasp])
        pick.scene_mngr.render_axis(ax, all_grasp_pose[pick.move_data.MOVE_post_grasp])
pick.scene_mngr.render_objects(ax)
p_utils.plot_basis(ax)
pick.show()
