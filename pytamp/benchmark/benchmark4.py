import numpy as np

from pykin.kinematics.transform import Transform
from pykin.robots.single_arm import SingleArm
from pykin.utils.mesh_utils import get_object_mesh, get_mesh_bounds
from pytamp.scene.scene_manager import SceneManager
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
        self.geom = geom
        self.is_pyplot = is_pyplot
        super().__init__(robot_name)
        
        self.load_robot()
        self.load_objects()
        self.load_scene()

    def load_robot(self):
        file_path = 'urdf/' + self.robot_name + '/' + self.robot_name + '.urdf'
        self.robot = SingleArm(f_name=file_path, offset=Transform(rot=[0.0, 0.0, 0.0], pos=[0, 0, 0.913]), has_gripper=True)
        self.robot.setup_link_name("panda_link_0", "panda_right_hand")
        self.robot.init_qpos = np.array([0, np.pi / 16.0, 0.00, -np.pi / 2.0 - np.pi / 3.0, 0.00, np.pi - 0.2, -np.pi/4])

    def load_objects(self):
        self.table_mesh = get_object_mesh('ben_table.stl')
        self.cylinder_mesh = get_object_mesh('hanoi_cylinder.stl', scale=[0.3, 0.3, 1.0])
        self.disk_mesh = get_object_mesh('hanoi_disk.stl')

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

    def load_scene(self):
        self.benchmark_config = {4 : None}
        self.scene_mngr = SceneManager(self.geom, is_pyplot=self.is_pyplot, benchmark=self.benchmark_config)

        theta = np.linspace(-np.pi, np.pi, self.disk_num)
        for i in range(self.disk_num):
            disk_pos = np.array([0.6, 0.25, self.table_height + self.disk_mesh_bound[1][2] + self.disk_heigh *i ])
            self.disk_pose[i] = Transform(pos=self.disk_mesh.center_mass + disk_pos)
            disk_name = "hanoi_disk_" + str(i)
            hanoi_mesh = get_object_mesh(f'hanoi_disk.stl')
            self.scene_mngr.add_object(name=disk_name, gtype="mesh", gparam=hanoi_mesh, h_mat=self.disk_pose[i].h_mat, color=[0., 1., 0.])

        self.scene_mngr.add_object(name="cylinder_1", gtype="mesh", gparam=self.cylinder_mesh, h_mat=self.cylinder1_pose.h_mat, color=[1, 0., 0.])
        self.scene_mngr.add_object(name="cylinder_2", gtype="mesh", gparam=self.cylinder_mesh, h_mat=self.cylinder2_pose.h_mat, color=[1, 0., 0.])
        self.scene_mngr.add_object(name="cylinder_3", gtype="mesh", gparam=self.cylinder_mesh, h_mat=self.cylinder3_pose.h_mat, color=[1, 0., 0.])
        self.scene_mngr.add_object(name="table", gtype="mesh", gparam=self.table_mesh, h_mat=self.table_pose.h_mat, color=[0.39, 0.263, 0.129])
        self.scene_mngr.add_robot(self.robot)

        self.scene_mngr.scene.logical_states["cylinder_1"] = {self.scene_mngr.scene.logical_state.on : self.scene_mngr.scene.objs["table"]}
        self.scene_mngr.scene.logical_states["cylinder_2"] = {self.scene_mngr.scene.logical_state.on : self.scene_mngr.scene.objs["table"]}
        self.scene_mngr.scene.logical_states["cylinder_3"] = {self.scene_mngr.scene.logical_state.on : self.scene_mngr.scene.objs["table"]}
        self.scene_mngr.scene.logical_states["cylinder_1"] = {self.scene_mngr.scene.logical_state.static : True}
        self.scene_mngr.scene.logical_states["cylinder_2"] = {self.scene_mngr.scene.logical_state.static : True}
        self.scene_mngr.scene.logical_states["cylinder_3"] = {self.scene_mngr.scene.logical_state.static : True}

        self.scene_mngr.scene.logical_states["hanoi_disk_0"] = {self.scene_mngr.scene.logical_state.on : self.scene_mngr.scene.objs["table"]}
        self.scene_mngr.scene.logical_states["hanoi_disk_1"] = {self.scene_mngr.scene.logical_state.on : self.scene_mngr.scene.objs["hanoi_disk_0"]}
        self.scene_mngr.scene.logical_states["hanoi_disk_2"] = {self.scene_mngr.scene.logical_state.on : self.scene_mngr.scene.objs["hanoi_disk_1"]}
        self.scene_mngr.scene.logical_states["hanoi_disk_3"] = {self.scene_mngr.scene.logical_state.on : self.scene_mngr.scene.objs["hanoi_disk_2"]}
        self.scene_mngr.scene.logical_states["hanoi_disk_4"] = {self.scene_mngr.scene.logical_state.on : self.scene_mngr.scene.objs["hanoi_disk_3"]}
        self.scene_mngr.scene.logical_states["hanoi_disk_5"] = {self.scene_mngr.scene.logical_state.on : self.scene_mngr.scene.objs["hanoi_disk_4"]}

        self.scene_mngr.scene.logical_states["table"] = {self.scene_mngr.scene.logical_state.static : True}
        self.scene_mngr.scene.logical_states[self.scene_mngr.gripper_name] = {self.scene_mngr.scene.logical_state.holding : None}
        self.scene_mngr.update_logical_states(is_init=True)
        self.scene_mngr.show_logical_states()