from pykin.utils import plot_utils as p_utils
from pytamp.action.pick import PickAction
from pytamp.benchmark import Benchmark1

benchmark1 = Benchmark1(
    robot_name="panda", geom="collision", is_pyplot=False, box_num=4
)

pick = PickAction(
    benchmark1.scene_mngr, n_contacts=0, n_directions=0, retreat_distance=0.1
)

################# Action Test ##################
fig, ax = p_utils.init_3d_figure(name="Heuristic")
for obj in benchmark1.scene_mngr.scene.goal_objects:
    pose = list(pick.get_grasp_pose_from_heuristic(obj_name=obj))
    for i in range(len(pose)):
        pick.scene_mngr.render_axis(ax, pose[i][pick.move_data.MOVE_grasp])
        pick.scene_mngr.set_gripper_pose(pose[i][pick.move_data.MOVE_grasp])
        pick.scene_mngr.close_gripper()
        pick.scene_mngr.render_axis(
            ax, pose=pick.scene_mngr.scene.robot.gripper.info["tcp"][3]
        )
        pick.scene_mngr.render_gripper(ax)
        pick.scene_mngr.open_gripper()

pick.scene_mngr.render_objects(ax)
pick.show()
