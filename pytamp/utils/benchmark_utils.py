import numpy as np
from trimesh import Trimesh

from pytamp.scene.scene_manager import SceneManager
from pykin.utils import transform_utils as t_utils

def get_heuristic_tcp_pose(scene_mngr:SceneManager, 
                           object_name:str,
                           object_mesh:Trimesh):
    
    bench_num = scene_mngr.scene.bench_num
    assert 1 <= bench_num <=4, f"Check again benchmark number.. the current number is {bench_num}. "
    
    if bench_num == 1:
        if "box" in object_name:
            obj_pose = np.eye(4)
            obj_pose[:3, :3] = scene_mngr.scene.objs[object_name].h_mat[:3, :3]
            
            obj_pose[1,:3] = abs(obj_pose[1,:3])
            obj_pose[0,:3] = np.cross(obj_pose[1,:3], obj_pose[2,:3])
            obj_pose[:3, 3] = object_mesh.center_mass + [0, 0, 0.01]
        
            for theta in np.linspace(np.pi+np.pi/24, np.pi-np.pi/24, 5):
                tcp_pose = np.eye(4)
                tcp_pose[:3,0] = [np.cos(theta), 0, np.sin(theta)]
                tcp_pose[:3,1] = [0, 1, 0]
                tcp_pose[:3,2] = [-np.sin(theta), 0, np.cos(theta)]
                tcp_pose = np.dot(obj_pose, tcp_pose)
                yield tcp_pose

    if bench_num == 2:
        if "bottle" in object_name:
            center_point = object_mesh.bounds[0] + (object_mesh.bounds[1] - object_mesh.bounds[0])/2
            for theta in np.linspace(-np.pi+np.pi/(2.2), -np.pi/(2.2), 3):
                tcp_pose = np.eye(4)
                tcp_pose[:3,0] = [np.cos(theta), 0, np.sin(theta)]
                tcp_pose[:3,1] = [0, 1, 0]
                tcp_pose[:3,2] = [-np.sin(theta), 0, np.cos(theta)]
                tcp_pose[:3,3] = center_point + [0, 0, 0.005]
                yield tcp_pose

    if bench_num == 3:
        pass

    if bench_num == 4:
        if "hanoi_disk" in object_name:
            obj_pose = np.eye(4)
            obj_pose[:3, :3] = scene_mngr.scene.objs[object_name].h_mat[:3, :3]
            obj_pose[:3, 3] = object_mesh.center_mass
            heuristic_pose = np.dot(obj_pose, t_utils.get_h_mat(position=np.array([0.060, 0, 0])))
            for theta in np.linspace(np.pi, np.pi-np.pi/24, 1):
                tcp_pose = np.eye(4)
                tcp_pose[:3,0] = [np.cos(theta), 0, np.sin(theta)]
                tcp_pose[:3,1] = [0, 1, 0]
                tcp_pose[:3,2] = [-np.sin(theta), 0, np.cos(theta)]
                tcp_pose = np.dot(heuristic_pose, tcp_pose)
                yield tcp_pose
