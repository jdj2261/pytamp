import numpy as np
from trimesh import Trimesh

from pytamp.scene.scene_manager import SceneManager
from pykin.utils import transform_utils as t_utils
from pykin.utils import mesh_utils as m_utils

def get_heuristic_tcp_pose(scene_mngr:SceneManager, 
                           object_name:str,
                           object_mesh:Trimesh,
                           n_directions:int):
    
    bench_num = scene_mngr.scene.bench_num
    assert 1 <= bench_num <=4, f"Check again benchmark number.. the current number is {bench_num}. "
    
    if bench_num == 1:
        if "box" in object_name:
            obj_pose = np.eye(4)
            obj_pose[:3, :3] = scene_mngr.scene.objs[object_name].h_mat[:3, :3]
            obj_pose[:3, 3] = object_mesh.center_mass + [0, 0, -0.005]
            for theta in np.linspace(np.pi+np.pi/24, np.pi-np.pi/24, n_directions):
                r_mat_y = t_utils.get_matrix_from_rpy(rpy=[0, -theta, 0])
                tcp_pose = np.eye(4)
                tcp_pose[:3, :3] = r_mat_y
                tcp_pose = np.dot(obj_pose, tcp_pose)
                yield tcp_pose

    if bench_num == 2:
        if "bottle" in object_name:
            center_point = object_mesh.bounds[0] + (object_mesh.bounds[1] - object_mesh.bounds[0])/2
            obj_pose = np.eye(4)
            obj_pose[:3, :3] = scene_mngr.scene.objs[object_name].h_mat[:3, :3]
            obj_pose[:3, 3] = center_point + [0, 0, 0.005]

            if "goal_bottle" not in object_name:
                for theta in np.linspace(-np.pi+np.pi/(2.2), -np.pi/(2.2), n_directions):
                    r_mat_y = t_utils.get_matrix_from_rpy(rpy=[0, -theta, 0])
                    for theta2 in np.linspace(-np.pi/3, np.pi/3, 5):
                        r_mat_z = t_utils.get_matrix_from_rpy(rpy=[0, 0, theta2])
                        tcp_pose = np.eye(4)
                        tcp_pose[:3, :3] = np.dot(r_mat_z, r_mat_y)
                        tcp_pose = np.dot(obj_pose, tcp_pose)
                        yield tcp_pose
            else:
                for theta in np.linspace(-np.pi+np.pi/(2.2), -np.pi/(2.2), n_directions):
                    r_mat_y = t_utils.get_matrix_from_rpy(rpy=[0, -theta, 0])
                    tcp_pose = np.eye(4)
                    tcp_pose[:3, :3] = r_mat_y
                    tcp_pose = np.dot(obj_pose, tcp_pose)
                    yield tcp_pose

    if bench_num == 3:
        if object_name in scene_mngr.scene.castle:
            center_point = object_mesh.bounds[0] + (object_mesh.bounds[1] - object_mesh.bounds[0])/2        
            if object_name == "rect_bottom_box" or object_name == "half_cylinder_box" or object_name == "rect_top_box":
                for theta in np.linspace(np.pi - np.pi/24, np.pi + np.pi/24, n_directions):
                    r_mat_y = t_utils.get_matrix_from_rpy(rpy=[0, theta, 0])
                    tcp_pose = np.eye(4)
                    tcp_pose[:3, :3] = r_mat_y
                    tcp_pose[:3,3] = center_point + [0, 0, 0.005]
                    
                    r_mat_z = t_utils.get_matrix_from_rpy(rpy=[0, 0, np.pi/2])
                    h_mat_z = np.eye(4)
                    h_mat_z[:3, :3] = r_mat_z
                    tcp_pose[:3, :3] = np.dot(h_mat_z[:3, :3], tcp_pose[:3, :3])
                    
                    yield tcp_pose
            else:
                obj_pose = np.eye(4)
                obj_pose[:3, :3] = scene_mngr.scene.objs[object_name].h_mat[:3, :3]
                obj_pose[:3, 3] = center_point
                for theta in np.linspace(np.pi, np.pi + np.pi/24, n_directions):
                    r_mat_y = t_utils.get_matrix_from_rpy(rpy=[0, theta, 0])
                    for theta2 in np.linspace(np.pi/2 - np.pi/12, np.pi/2 + np.pi/12, n_directions):
                        r_mat_z = t_utils.get_matrix_from_rpy(rpy=[0, 0, theta2])
                        tcp_pose = np.eye(4)
                        tcp_pose[:3, :3] = np.dot(r_mat_z, r_mat_y)
                        tcp_pose = np.dot(obj_pose, tcp_pose)
                        yield tcp_pose

    if bench_num == 4:
        if "hanoi_disk" in object_name:
            split_num = float(object_name.split('_')[-1])
            obj_pose = np.eye(4)
            obj_pose[:3, :3] = scene_mngr.scene.objs[object_name].h_mat[:3, :3]
            obj_pose[:3, 3] = object_mesh.center_mass + [0, 0, -0.005]
            heuristic_pose = np.dot(obj_pose, t_utils.get_h_mat(position=np.array([0.08+0.01*int(5 - split_num), 0, 0])))
            for theta in np.linspace(np.pi-np.pi/24, np.pi-np.pi/12, n_directions):
                r_mat_y = t_utils.get_matrix_from_rpy(rpy=[0, -theta, 0])
                tcp_pose = np.eye(4)
                tcp_pose[:3, :3] = r_mat_y
                tcp_pose = np.dot(heuristic_pose, tcp_pose)
                yield tcp_pose

def get_heuristic_release_eef_pose(obj_pose_transformed,
                                   eef_pose,
                                   n_directions=10):
    for theta in np.linspace(0, np.pi, n_directions):
        T = m_utils.get_relative_transform(obj_pose_transformed, eef_pose)
        obj_r_mat_z = t_utils.get_matrix_from_rpy(rpy=[0, 0, theta])
        obj_h_mat_z = np.eye(4)
        obj_h_mat_z[:3, :3] = obj_r_mat_z
        obj_pose_transformed = np.dot(obj_pose_transformed, obj_h_mat_z)
        eef_pose = np.dot(obj_pose_transformed, T)
        yield eef_pose, obj_pose_transformed