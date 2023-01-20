from pykin.utils import plot_utils as p_utils
from pytamp.action.pick import PickAction
from pytamp.benchmark import Benchmark2

benchmark2 = Benchmark2(robot_name="doosan", geom="visual", is_pyplot=False)
pick = PickAction(
    benchmark2.scene_mngr, n_contacts=0, n_directions=1, retreat_distance=0.1
)

################# Action Test ##################
fig, ax = p_utils.init_3d_figure(name="Heuristic")

pose = list(pick.get_grasp_pose_from_heuristic(obj_name="goal_bottle"))
for i in range(len(pose)):
    pick.scene_mngr.render_axis(ax, pose[i][pick.move_data.MOVE_grasp])
    # pick.scene_mngr.set_gripper_pose(pose[i][pick.move_data.MOVE_grasp])
    # pick.scene_mngr.render_axis(ax, pose=pick.scene_mngr.scene.robot.gripper.info["tcp"][3])
    pick.scene_mngr.render_axis(ax, pick.scene_mngr.scene.objs["goal_bottle"].h_mat)

    # pick.scene_mngr.render_gripper(ax)

pick.scene_mngr.render_objects(ax)
p_utils.plot_basis(ax)
pick.show()
