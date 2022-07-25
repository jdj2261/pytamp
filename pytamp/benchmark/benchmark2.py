import numpy as np

from pykin.robots.single_arm import SingleArm
from pykin.kinematics.transform import Transform
from pykin.utils.mesh_utils import get_object_mesh
from pytamp.benchmark.benchmark import Benchmark

"""
shelf mesh num
13, 8, 0
15,  9
17, 16, 2
"""

class Benchmark2(Benchmark):
    def __init__(
        self, 
        robot_name="panda", 
        geom="visual", 
        is_pyplot=True,
        bottle_num=6
    ):
        assert bottle_num <= 6, f"The number of bottles must be 6 or less."
        self.bottle_num = bottle_num
        param = {'bottle_num' : self.bottle_num, 'goal_object' : 'goal_bottle'}
        self.benchmark_config = {2 : param}
        super().__init__(robot_name, geom, is_pyplot, self.benchmark_config)
        
        self._load_robot()
        self._load_objects()
        self._load_scene()

    def _load_robot(self):
        self.robot = SingleArm(f_name=self.urdf_file, offset=Transform(rot=[0.0, 0.0, 0.0], pos=[0.3, 0, 0.913]), has_gripper=True, gripper_name=self.gripper_name)
        
        if self.robot_name == "panda":
            self.robot.setup_link_name("panda_link_0", "right_hand")
            self.robot.init_qpos = np.array([0, np.pi / 16.0, 0.00, -np.pi / 2.0 - np.pi / 3.0, 0.00, np.pi - 0.2, -np.pi/4])
            
        if self.robot_name == "doosan":
            self.robot.setup_link_name("base_0", "right_hand")
            self.robot.init_qpos = np.array([0, -np.pi/3, np.pi/1.5, 0, np.pi/3, np.pi/2])
        
    def _load_objects(self):
        self.shelf_pose = Transform(pos=np.array([0.9, 0, 1.41725156]),rot=np.array([0, 0, np.pi/2]))
        self.bin_pose = Transform(pos=np.array([0.0, 1.0, 0.3864222]))
        
        self.bottle_meshes = [get_object_mesh('bottle.stl') for _ in range(self.bottle_num)]
        self.goal_bottle_pose = Transform(pos=np.array([1.0, 0, 1.29]))

        self.bottle_poses = []
        bottle_pose1 = Transform(pos=np.array([0.95, 0.15, 1.29]))
        bottle_pose2 = Transform(pos=np.array([0.95, -0.15,1.29]))
        bottle_pose3 = Transform(pos=np.array([0.85, 0.15, 1.29]))
        bottle_pose4 = Transform(pos=np.array([0.85, 0, 1.29]))
        bottle_pose5 = Transform(pos=np.array([0.85, -0.15, 1.29]))
        
        self.bottle_poses.extend([bottle_pose1,
                                  bottle_pose2,
                                  bottle_pose3,
                                  bottle_pose4,
                                  bottle_pose5])
    def _load_scene(self):
        for i in range(20):
            shelf_name = 'shelf_' + str(i)
            self.shelf_mesh = get_object_mesh(shelf_name + '.stl', scale=0.9)
            self.scene_mngr.add_object(name=shelf_name, gtype="mesh", h_mat=self.shelf_pose.h_mat, gparam=self.shelf_mesh, color=[0.39, 0.263, 0.129])

        for i in range(20):
            bin_name = 'bin_' + str(i)
            bin_mesh = get_object_mesh(bin_name + '.stl', scale=0.9)
            self.scene_mngr.add_object(name=bin_name, gtype="mesh", h_mat=self.bin_pose.h_mat, gparam=bin_mesh, color=[0.8 + i*0.01, 0.8 + i*0.01, 0.8 + i*0.01])

        for i in range(self.bottle_num-1):
            bottle_name = "bottle_" + str(i+1)
            self.scene_mngr.add_object(name=bottle_name, gtype="mesh", h_mat=self.bottle_poses[i].h_mat, gparam=self.bottle_meshes[i+1], color=[0., 1., 0.])
            self.scene_mngr.set_logical_state(bottle_name, ("on", "shelf_9"))

        self.scene_mngr.add_object(name="goal_bottle", gtype="mesh", h_mat=self.goal_bottle_pose.h_mat, gparam=self.bottle_meshes[0], color=[1., 0., 0.])
        self.scene_mngr.set_logical_state("goal_bottle", ("on", "shelf_9"))
        self.scene_mngr.add_robot(self.robot, self.robot.init_qpos)

        for i in range(20):
            self.scene_mngr.set_logical_state(f"shelf_"+str(i), (self.scene_mngr.scene.logical_state.static, True))
            self.scene_mngr.set_logical_state(f"bin_"+str(i), (self.scene_mngr.scene.logical_state.static, True))
        
        self.scene_mngr.set_logical_state(self.scene_mngr.gripper_name, (self.scene_mngr.scene.logical_state.holding, None))
        self.scene_mngr.update_logical_states(is_init=True)
        self.scene_mngr.show_logical_states()