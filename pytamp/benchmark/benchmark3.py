import numpy as np

from pykin.kinematics.transform import Transform
from pykin.robots.single_arm import SingleArm
from pykin.utils.mesh_utils import get_object_mesh
from pytamp.scene.scene_manager import SceneManager
from pytamp.benchmark.benchmark import Benchmark

class Benchmark3(Benchmark):
    def __init__(
        self, 
        robot_name="panda", 
        geom="visual", 
        is_pyplot=True
    ):
        self.geom = geom
        self.is_pyplot = is_pyplot
        super().__init__(robot_name)
        
        self.load_robot()
        self.load_objects()
        self.load_scene()

    def load_robot(self):
        file_path = 'urdf/' + self.robot_name + '/' + self.robot_name + '.urdf'
        self.robot = SingleArm(f_name=file_path, offset=Transform(rot=[0.0, 0.0, 0.0], pos=[0, 0, 0.913]), has_gripper=True)
        
        if self.robot_name == "panda":
            self.robot.setup_link_name("panda_link_0", "right_hand")
            self.robot.init_qpos = np.array([0, np.pi / 16.0, 0.00, -np.pi / 2.0 - np.pi / 3.0, 0.00, np.pi - 0.2, -np.pi/4])

        # TODO
        if self.robot_name == "doosan":
            pass

    def load_objects(self):
        self.table_mesh = get_object_mesh('ben_table.stl')
        self.table_height = self.table_mesh.bounds[1][2] - self.table_mesh.bounds[0][2]
        self.table_pose = Transform(pos=np.array([1.0, -0.4, -0.03]))
        self.clearbox1_pose = Transform(pos=np.array([0.6, 0.25, self.table_height + 0.0607473]))
        self.clearbox2_pose = Transform(pos=np.array([0.6, -0.25, self.table_height + 0.0607473]))

    def load_scene(self):
        self.benchmark_config = {3 : None}
        self.scene_mngr = SceneManager(self.geom, is_pyplot=self.is_pyplot, benchmark=self.benchmark_config)

        for i in range(20):
            clearbox_1_name = 'clearbox_1_' + str(i)
            clearbox_1_mesh = get_object_mesh(f'clearbox_{i}' + '.stl', scale=0.9)
            self.scene_mngr.add_object(name=clearbox_1_name, gtype="mesh", h_mat=self.clearbox1_pose.h_mat, gparam=clearbox_1_mesh, color=[0.8 + i*0.01, 0.8 + i*0.01, 0.8 + i*0.01])
            self.scene_mngr.scene.logical_states[clearbox_1_name] = {self.scene_mngr.scene.logical_state.static : True}
            
            clearbox_2_name = 'clearbox_2_' + str(i)
            clearbox_2_mesh = get_object_mesh(f'clearbox_{i}' + '.stl', scale=0.9)
            self.scene_mngr.add_object(name=clearbox_2_name, gtype="mesh", h_mat=self.clearbox2_pose.h_mat, gparam=clearbox_2_mesh, color=[0.8 + i*0.01, 0.8 + i*0.01, 0.8 + i*0.01])
            self.scene_mngr.scene.logical_states[clearbox_2_name] = {self.scene_mngr.scene.logical_state.static : True}

        self.scene_mngr.add_object(name="table", gtype="mesh", gparam=self.table_mesh, h_mat=self.table_pose.h_mat, color=[0.39, 0.263, 0.129])
        self.scene_mngr.add_robot(self.robot)
        
        self.scene_mngr.scene.logical_states["table"] = {self.scene_mngr.scene.logical_state.static : True}
        self.scene_mngr.scene.logical_states[self.scene_mngr.gripper_name] = {self.scene_mngr.scene.logical_state.holding : None}
        self.scene_mngr.update_logical_states(is_init=True)
        self.scene_mngr.show_logical_states()