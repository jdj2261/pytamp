import numpy as np

from pykin.kinematics.transform import Transform
from pykin.utils.mesh_utils import get_object_mesh, get_mesh_bounds
from pytamp.benchmark.benchmark import Benchmark

class Benchmark4(Benchmark):
    def __init__(
        self, 
        robot_name="panda", 
        disk_num=6,
        geom="visual", 
        is_pyplot=True
    ):
        self.disk_num = disk_num
        self.benchmark_config = {4 : None}
        super().__init__(robot_name, geom, is_pyplot, self.benchmark_config)
        
        # panda arm is 7 axis robot arm 
        if robot_name == "panda":
            self.robot.init_qpos = np.array([ 0, 0, np.pi/1.5,0, 0, np.pi/3,  0])
        if robot_name == "doosan":
            self.robot.init_qpos = np.array([ 0, 0, np.pi/1.5, 0, np.pi/3,  0])
        
        self._load_objects()
        self._load_scene()

    def _load_objects(self):
        self.table_mesh = get_object_mesh('ben_table.stl')
        self.cylinder_mesh = get_object_mesh('hanoi_cylinder.stl', scale=[0.9, 0.9, 1.0])
        self.disk_mesh = get_object_mesh('hanoi_disk.stl', scale=[1.2, 1.2, 1.0])

        self.cylinder_mesh_bound = get_mesh_bounds(mesh=self.cylinder_mesh)
        self.disk_mesh_bound = get_mesh_bounds(mesh=self.disk_mesh)
        self.disk_heigh = self.disk_mesh_bound[1][2] - self.disk_mesh_bound[0][2]
        self.table_height = self.table_mesh.bounds[1][2] - self.table_mesh.bounds[0][2]

        self.table_pose = Transform(pos=np.array([1.0, -0.4, -0.03]))
        self.cylinder1_pose = Transform(pos=np.array([0.6, -0.25, self.table_height + self.cylinder_mesh_bound[1][2]]))
        self.cylinder2_pose = Transform(pos=np.array([0.6, 0, self.table_height + self.cylinder_mesh_bound[1][2]]))
        self.cylinder3_pose = Transform(pos=np.array([0.6, 0.25, self.table_height + self.cylinder_mesh_bound[1][2]]))

        self.disk_pose = [ Transform() for _ in range(self.disk_num)]
        self.disk_object = [ 0 for _ in range(self.disk_num)]

    def _load_scene(self):
        theta = np.linspace(-np.pi, np.pi, self.disk_num)
        for i in range(self.disk_num):
            disk_pos = np.array([0.59, 0.25, self.table_height + self.disk_mesh_bound[1][2] + self.disk_heigh *i ])
            disk_ori = Transform._to_quaternion([0, 0, theta[i]])
            disk_ori = Transform._to_quaternion([0, 0, np.pi])
            self.disk_pose[i] = Transform(pos=self.disk_mesh.center_mass + disk_pos, rot=disk_ori)
            disk_name = "hanoi_disk_" + str(i)
            hanoi_mesh = get_object_mesh(f'hanoi_disk.stl', scale=[1.5-0.1*i, 1.5-0.1*i, 1])
            self.scene_mngr.add_object(name=disk_name, gtype="mesh", gparam=hanoi_mesh, h_mat=self.disk_pose[i].h_mat, color=[0., 1., 0.])

        self.scene_mngr.add_object(name="cylinder_1", gtype="mesh", gparam=self.cylinder_mesh, h_mat=self.cylinder1_pose.h_mat, color=[0., 0., 1.])
        self.scene_mngr.add_object(name="cylinder_2", gtype="mesh", gparam=self.cylinder_mesh, h_mat=self.cylinder2_pose.h_mat, color=[0., 0., 1.])
        self.scene_mngr.add_object(name="cylinder_3", gtype="mesh", gparam=self.cylinder_mesh, h_mat=self.cylinder3_pose.h_mat, color=[0., 0., 1.])
        self.scene_mngr.add_object(name="table", gtype="mesh", gparam=self.table_mesh, h_mat=self.table_pose.h_mat, color=[0.39, 0.263, 0.129])
        self.scene_mngr.add_robot(self.robot)

        self.scene_mngr.set_logical_state("cylinder_1", ("on", "table"), ("static", True)) 
        self.scene_mngr.set_logical_state("cylinder_2", ("on", "table"), ("static", True)) 
        self.scene_mngr.set_logical_state("cylinder_3", ("on", "table"), ("static", True)) 
        
        self.scene_mngr.set_logical_state("hanoi_disk_0", ("on", "table"))
        self.scene_mngr.set_logical_state("hanoi_disk_1", ("on", "hanoi_disk_0"))
        self.scene_mngr.set_logical_state("hanoi_disk_2", ("on", "hanoi_disk_1"))
        self.scene_mngr.set_logical_state("hanoi_disk_3", ("on", "hanoi_disk_2"))
        self.scene_mngr.set_logical_state("hanoi_disk_4", ("on", "hanoi_disk_3"))
        self.scene_mngr.set_logical_state("hanoi_disk_5", ("on", "hanoi_disk_4"))

        self.scene_mngr.set_logical_state("table", (self.scene_mngr.scene.logical_state.static, True), (self.scene_mngr.scene.logical_state.holding, None))
        self.scene_mngr.set_logical_state(self.scene_mngr.gripper_name, (self.scene_mngr.scene.logical_state.holding, None))
        self.scene_mngr.update_logical_states(is_init=True)
        self.scene_mngr.show_logical_states()