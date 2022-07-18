import numpy as np

from abc import abstractclassmethod, ABCMeta
from pykin.kinematics.transform import Transform
from pykin.robots.single_arm import SingleArm

class Benchmark(metaclass=ABCMeta):
    def __init__(self, robot_name):
        self.robot_name = robot_name
        self._load_robot()
        
    def _load_robot(self):
        urdf_file_name = self.robot_name
        gripper_name = "panda"
        if self.robot_name == "doosan":
            urdf_file_name = "doosan_with_robotiq140"
            gripper_name = "robotiq140"
        file_path = 'urdf/' + self.robot_name + '/' + urdf_file_name + '.urdf'
        self.robot = SingleArm(f_name=file_path, offset=Transform(rot=[0.0, 0.0, 0.0], pos=[0, 0, 0.913]), has_gripper=True, gripper_name=gripper_name)
        
        if self.robot_name == "panda":
            self.robot.setup_link_name("panda_link_0", "right_hand")
            self.robot.init_qpos = np.array([0, np.pi / 16.0, 0.00, -np.pi / 2.0 - np.pi / 3.0, 0.00, np.pi - 0.2, -np.pi/4])
            
        if self.robot_name == "doosan":
            self.robot.setup_link_name("base_0", "right_hand")
            

    @abstractclassmethod
    def _load_objects(self):
        raise NotImplementedError

    @abstractclassmethod
    def _load_scene(self):
        raise NotImplementedError