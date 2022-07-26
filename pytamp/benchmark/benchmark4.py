from matplotlib.pyplot import sca
import numpy as np

from pykin.robots.single_arm import SingleArm
from pykin.kinematics.transform import Transform
from pykin.utils.mesh_utils import get_object_mesh, get_mesh_bounds
from pytamp.benchmark.benchmark import Benchmark

class Benchmark4(Benchmark):
    def __init__(
        self, 
        robot_name="panda", 
        disk_num=5,
        geom="visual", 
        is_pyplot=True
    ):
        assert disk_num <= 5, f"The number of disks must be 5 or less."
        self.disk_num = disk_num
        param = {"disk_num" : disk_num}
        self.benchmark_config = {4 : param}
        super().__init__(robot_name, geom, is_pyplot, self.benchmark_config)
        
        self._load_robot()
        self._load_objects()
        self._load_scene()

    def _load_robot(self):
        self.robot = SingleArm(f_name=self.urdf_file, offset=Transform(rot=[0.0, 0.0, 0.0], pos=[0, 0, 0.913]), has_gripper=True, gripper_name=self.gripper_name)
        
        if self.robot_name == "panda":
            self.robot.setup_link_name("panda_link_0", "right_hand")
            self.robot.init_qpos = np.array([0, np.pi / 16.0, 0.00, -np.pi / 2.0 - np.pi / 3.0, 0.00, np.pi - 0.2, -np.pi/4])
            
        if self.robot_name == "doosan":
            self.robot.setup_link_name("base_0", "right_hand")
            self.robot.init_qpos = np.array([ 0, 0, np.pi/1.5, 0, np.pi/3, 0])
        
    def _load_objects(self):
        self.table_mesh = get_object_mesh('ben_table.stl', [1.0, 1.5, 1.0])
        self.cylinder_mesh = get_object_mesh('hanoi_cylinder.stl', scale=[1.0, 1.0, 1.0])
        self.disk_mesh = get_object_mesh('hanoi_disk.stl', scale=[1, 1, 2.0])

        self.cylinder_mesh_bound = get_mesh_bounds(mesh=self.cylinder_mesh)
        self.disk_mesh_bound = get_mesh_bounds(mesh=self.disk_mesh)
        self.disk_heigh = self.disk_mesh_bound[1][2] - self.disk_mesh_bound[0][2]
        self.table_height = self.table_mesh.bounds[1][2] - self.table_mesh.bounds[0][2]

        self.table_pose = Transform(pos=np.array([1.0, -0.6, -self.table_mesh.bounds[0][2]]))
        self.cylinder1_pose = Transform(pos=np.array([0.7, -0.20, self.table_height + self.cylinder_mesh_bound[1][2]]))
        self.cylinder2_pose = Transform(pos=np.array([0.7, 0, self.table_height + self.cylinder_mesh_bound[1][2]]))
        self.cylinder3_pose = Transform(pos=np.array([0.7, 0.20, self.table_height + self.cylinder_mesh_bound[1][2]]))

        self.disk_pose = [ Transform() for _ in range(self.disk_num)]
        self.disk_object = [ 0 for _ in range(self.disk_num)]

    def _load_scene(self):
        self.scene_mngr.add_object(name="cylinder_1", gtype="mesh", gparam=self.cylinder_mesh, h_mat=self.cylinder1_pose.h_mat, color=[0., 0., 1.])
        self.scene_mngr.add_object(name="cylinder_2", gtype="mesh", gparam=self.cylinder_mesh, h_mat=self.cylinder2_pose.h_mat, color=[0., 0., 1.])
        self.scene_mngr.add_object(name="cylinder_3", gtype="mesh", gparam=self.cylinder_mesh, h_mat=self.cylinder3_pose.h_mat, color=[0., 0., 1.])
        self.scene_mngr.add_object(name="table", gtype="mesh", gparam=self.table_mesh, h_mat=self.table_pose.h_mat, color=[0.39, 0.263, 0.129])
        self.scene_mngr.add_robot(self.robot)

        self.scene_mngr.set_logical_state("cylinder_1", ("on", "table"), ("static", True)) 
        self.scene_mngr.set_logical_state("cylinder_2", ("on", "table"), ("static", True)) 
        self.scene_mngr.set_logical_state("cylinder_3", ("on", "table"), ("static", True)) 
    
        # theta = np.linspace(-np.pi, np.pi, disk_num)
        for i in range(self.disk_num):
            disk_pos = np.array([0.69, 0.2, self.table_height + self.disk_mesh_bound[1][2] + self.disk_heigh *i ])
            self.disk_pose[i] = Transform(pos=self.disk_mesh.center_mass + disk_pos)
            disk_name = "hanoi_disk_" + str(i)
            print(disk_name)
            hanoi_mesh = get_object_mesh(f'hanoi_disk.stl', scale=[1.5-0.1*i, 1.5-0.1*i, 2.0])
            self.scene_mngr.add_object(name=disk_name, gtype="mesh", gparam=hanoi_mesh, h_mat=self.disk_pose[i].h_mat, color=[0., 1., 0.])
        
        for i in range(self.disk_num):
            disk_name = "hanoi_disk_" + str(i)
            if disk_name == "hanoi_disk_0":
                self.scene_mngr.set_logical_state(disk_name, ("on", "table"))
            else:
                prev_disk_name = "hanoi_disk_" + str(i-1)
                self.scene_mngr.set_logical_state(disk_name, ("on", prev_disk_name))

        self.scene_mngr.set_logical_state("table", (self.scene_mngr.scene.logical_state.static, True), (self.scene_mngr.scene.logical_state.holding, None))
        self.scene_mngr.set_logical_state(self.scene_mngr.gripper_name, (self.scene_mngr.scene.logical_state.holding, None))
        self.scene_mngr.update_logical_states(is_init=True)
        self.scene_mngr.show_logical_states()