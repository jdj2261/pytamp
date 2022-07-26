import numpy as np

from pykin.robots.single_arm import SingleArm
from pykin.kinematics.transform import Transform
from pykin.utils.mesh_utils import get_object_mesh
from pytamp.benchmark.benchmark import Benchmark

class Benchmark1(Benchmark):
    def __init__(
        self, 
        robot_name="panda", 
        box_num=6,
        geom="visual", 
        is_pyplot=True
    ):
        assert box_num <= 6, f"The number of boxes must be 6 or less."
        self.box_num = box_num
        self.param = {'stack_num' : self.box_num, 'goal_object':'tray_red'}
        self.benchmark_config = {1 : self.param}
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
        self.table_mesh = get_object_mesh('ben_table.stl')
        self.ceiling_mesh = get_object_mesh('ben_table_ceiling.stl')
        self.tray_red_mesh = get_object_mesh('ben_tray_red.stl')
        # self.tray_blue_mesh = get_object_mesh('ben_tray_blue.stl')

        self.box_poses = []
        A_box_pose = Transform(pos=np.array([0.5, -0.2, self.table_mesh.bounds[1][2]]))
        B_box_pose = Transform(pos=np.array([0.5, 0., self.table_mesh.bounds[1][2]]))
        C_box_pose = Transform(pos=np.array([0.5, 0.2, self.table_mesh.bounds[1][2]]))
        D_box_pose = Transform(pos=np.array([0.7, -0.2, self.table_mesh.bounds[1][2]]))
        E_box_pose = Transform(pos=np.array([0.7, 0., self.table_mesh.bounds[1][2]]))
        F_box_pose = Transform(pos=np.array([0.7, 0.2, self.table_mesh.bounds[1][2]]))
        self.box_poses.extend([A_box_pose, 
                              B_box_pose, 
                              C_box_pose, 
                              D_box_pose,
                              E_box_pose,
                              F_box_pose])

        self.box_colors = []
        A_box_color=np.array([1.0, 0.0, 0.0])
        B_box_color=np.array([0.0, 1.0, 0.0])
        C_box_color=np.array([0.0, 0.0, 1.0])
        D_box_color=np.array([1.0, 1.0, 0.0])
        E_box_color=np.array([0.0, 1.0, 1.0])
        F_box_color=np.array([1.0, 0.0, 1.0])
        self.box_colors.extend([A_box_color, 
                              B_box_color, 
                              C_box_color, 
                              D_box_color,
                              E_box_color,
                              F_box_color])

        self.table_pose = Transform(pos=np.array([1.0, -0.4, -0.03]))
        self.ceiling_pose = Transform(pos=np.array([1.0, -0.4, 1.5]))
        self.tray_red_pose = Transform(pos=np.array([0.6, -0.5-0.3, 0.8]))
        # self.tray_blue_pose = Transform(pos=np.array([0.6, 0.5, 0.8]))

    def _load_scene(self):
        self.scene_mngr.add_object(name="table", gtype="mesh", gparam=self.table_mesh, h_mat=self.table_pose.h_mat, color=[0.39, 0.263, 0.129])
        for i in range(self.box_num):
            box_name = self.scene_mngr.scene.alphabet_list[i] + '_box'
            box_mesh = get_object_mesh('ben_cube.stl', 0.06)
            self.scene_mngr.add_object(name=box_name, gtype="mesh", gparam=box_mesh, h_mat=self.box_poses[i].h_mat, color=self.box_colors[i])
            self.scene_mngr.set_logical_state(box_name, ("on", "table"))

        # self.scene_mngr.add_object(name="ceiling", gtype="mesh", gparam=self.ceiling_mesh, h_mat=self.ceiling_pose.h_mat, color=[0.39, 0.263, 0.129])
        self.scene_mngr.add_object(name="tray_red", gtype="mesh", gparam=self.tray_red_mesh, h_mat=self.tray_red_pose.h_mat, color=[1.0, 0, 0])
        # self.scene_mngr.add_object(name="tray_blue", gtype="mesh", gparam=self.tray_blue_mesh, h_mat=self.tray_blue_pose.h_mat, color=[0, 0, 1.0])
        self.scene_mngr.add_robot(self.robot, self.robot.init_qpos)
        
        # self.scene_mngr.set_logical_state("ceiling", (self.scene_mngr.scene.logical_state.static, True))
        self.scene_mngr.set_logical_state("tray_red", (self.scene_mngr.scene.logical_state.static, True))
        # self.scene_mngr.set_logical_state("tray_blue", (self.scene_mngr.scene.logical_state.static, True))
        self.scene_mngr.set_logical_state("table", (self.scene_mngr.scene.logical_state.static, True))
        self.scene_mngr.set_logical_state(self.scene_mngr.gripper_name, (self.scene_mngr.scene.logical_state.holding, None))
        self.scene_mngr.update_logical_states(is_init=True)
        self.scene_mngr.show_logical_states()
