from pykin.utils import plot_utils as p_utils
from pytamp.action.pick import PickAction
from pytamp.benchmark import Benchmark3

benchmark3 = Benchmark3(robot_name="doosan", geom="collision", is_pyplot=True)
pick = PickAction(
    benchmark3.scene_mngr, n_contacts=0, n_directions=3, retreat_distance=0.1
)


################# Action Test ##################
for obj_name in ["milk1", "milk2"]:
    pose = list(pick.get_grasp_pose_from_heuristic(obj_name=obj_name))
    for i in range(len(pose)):
        fig, ax = p_utils.init_3d_figure(name="Heuristic")
        pick.scene_mngr.render_axis(ax, pose[i][pick.move_data.MOVE_grasp])
        pick.scene_mngr.set_gripper_pose(pose[i][pick.move_data.MOVE_grasp])
        pick.scene_mngr.close_gripper()
        pick.scene_mngr.render_axis(
            ax, pose=pick.scene_mngr.scene.robot.gripper.info["tcp"][3]
        )
        pick.scene_mngr.render_gripper(ax)
        pick.scene_mngr.open_gripper()

        pick.scene_mngr.render_objects(ax)
        p_utils.plot_basis(ax)
        pick.show()
