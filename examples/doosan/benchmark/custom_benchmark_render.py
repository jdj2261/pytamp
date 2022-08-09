import numpy as np

from pytamp.benchmark import Benchmark
from pytamp.action.pick import PickAction
from pytamp.utils.heuristic_utils import get_custom_tcp_pose

from pykin.robots.single_arm import SingleArm
from pykin.kinematics.transform import Transform
from pykin.utils.mesh_utils import get_object_mesh



# Custom Benchmark Setting
custom_benchmark = Benchmark(robot_name="doosan", geom="visual", is_pyplot=True)
custom_benchmark.robot = SingleArm(f_name=custom_benchmark.urdf_file, 
                               offset=Transform(rot=[0.0, 0.0, 0.0], pos=[0, 0, 0.913]),
                               has_gripper=True, 
                               gripper_name=custom_benchmark.gripper_name)

custom_benchmark.robot.setup_link_name("base_0", "right_hand")
custom_benchmark.robot.init_qpos = np.array([ 0, 0, np.pi/1.5, 0, np.pi/3, 0])
custom_benchmark.scene_mngr.add_robot(custom_benchmark.robot, custom_benchmark.robot.init_qpos)

table_mesh = get_object_mesh('ben_table.stl')
tray_red_mesh = get_object_mesh('ben_tray_red.stl')
box_mesh = get_object_mesh('ben_cube.stl', 0.06)

print(box_mesh.bounds)

table_pose = Transform(pos=np.array([1.1, -0.4, -0.03]))
tray_red_pose = Transform(pos=np.array([0.6, -0.5-0.3, 0.8]))
A_box_pose = Transform(pos=np.array([0.6, 0, table_mesh.bounds[1][2]]))

custom_benchmark.scene_mngr.add_object(name="table", gtype="mesh", gparam=table_mesh, h_mat=table_pose.h_mat, color=[0.39, 0.263, 0.129])
custom_benchmark.scene_mngr.add_object(name="A_box", gtype="mesh", gparam=box_mesh, h_mat=A_box_pose.h_mat, color=[1.0, 1.0, 0.])

pick_obj = "A_box"

# Pick Action
pick = PickAction(custom_benchmark.scene_mngr, n_contacts=0, n_directions=10, retreat_distance=0.1)

tcp_pose = get_custom_tcp_pose(pick.scene_mngr, pick_obj)

grasp_poses = {}
grasp_poses[pick.move_data.MOVE_grasp] = pick.scene_mngr.scene.robot.gripper.compute_eef_pose_from_tcp_pose(tcp_pose)
grasp_poses[pick.move_data.MOVE_pre_grasp] = pick.get_pre_grasp_pose(grasp_poses[pick.move_data.MOVE_grasp])
grasp_poses[pick.move_data.MOVE_post_grasp] = pick.get_post_grasp_pose(grasp_poses[pick.move_data.MOVE_grasp])

pre_grasp_pose = grasp_poses[pick.move_data.MOVE_pre_grasp]
grasp_pose = grasp_poses[pick.move_data.MOVE_grasp]
post_grasp_pose = grasp_poses[pick.move_data.MOVE_post_grasp]

# pre_grasp_joint_path = pick.get_rrt_star_path(pick.scene_mngr.scene.robot.init_qpos, pre_grasp_pose)

pre_grasp_thetas = pick.scene_mngr.scene.robot.inverse_kin(pick.scene_mngr.scene.robot.init_qpos, pre_grasp_pose)
grasp_joint_path = pick.get_cartesian_path(pre_grasp_thetas, grasp_pose)


result = np.round(np.array(grasp_joint_path), 6).reshape(-1).tolist()
print(result)
print(type(result))


