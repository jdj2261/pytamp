import numpy as np

from pykin.kinematics.transform import Transform
from pykin.utils.mesh_utils import get_object_mesh
from pytamp.benchmark.benchmark import Benchmark

class Benchmark3(Benchmark):
    def __init__(
        self, 
        robot_name="panda", 
        geom="visual", 
        is_pyplot=True
    ):
        self.benchmark_config = {3 : None}
        super().__init__(robot_name, geom, is_pyplot, self.benchmark_config)
        
        self.robot.init_qpos = np.array([ 0, 0, np.pi/1.5, 0, np.pi/3,  0])
        self._load_objects()
        self._load_scene()

    def _load_objects(self):
        self.table_mesh = get_object_mesh('ben_table.stl')
        self.table_height = self.table_mesh.bounds[1][2] - self.table_mesh.bounds[0][2]
        self.table_pose = Transform(pos=np.array([1.0, -0.4, -0.03]))
        
        clearbox_8_mesh = get_object_mesh(f'clearbox_8.stl', scale=[1.7, 1.7, 1.7],)
        self.clearbox1_pose = Transform(pos=np.array([0.7, 0.2, self.table_height + abs(clearbox_8_mesh.bounds[0][2])]), rot=[0, 0, np.pi/2])

        self.arch_box = get_object_mesh('arch_box.stl', [0.001, 0.001, 0.001])
        self.arch_box.apply_translation(-self.arch_box.center_mass)
        
        self.can = get_object_mesh('can.stl')
        self.can.apply_translation(-self.can.center_mass)
        
        self.rect_box = get_object_mesh('rect_box.stl', [0.001, 0.001, 0.001])
        self.rect_box.apply_translation(-self.rect_box.center_mass)
        
        self.half_cylinder_box = get_object_mesh('half_cylinder_box.stl', [0.001, 0.001, 0.001])
        self.half_cylinder_box.apply_translation(-self.half_cylinder_box.center_mass)
        
        self.square_box = get_object_mesh('square_box.stl', [0.001, 0.001, 0.001])
        self.square_box.apply_translation(-self.square_box.center_mass)

        self.arch_box_pose = Transform(pos=np.array([0.6, -0.15, self.table_height + self.arch_box.bounds[1][2]]))
        self.can_pose = Transform(np.array([0.6, -0.3, self.table_height + self.can.bounds[1][2]]))
        self.rect_box_pose = Transform(np.array([0.5, -0.3, self.table_height + self.rect_box.bounds[1][2]]))
        self.half_cylinder_box_pose = Transform(np.array([0.7, -0.3, self.table_height + self.half_cylinder_box.bounds[1][2]]))
        self.square_box_pose = Transform(np.array([0.7, -0.2, self.table_height + self.square_box.bounds[1][2]]))
        
    def _load_scene(self):
        self.benchmark_config = {3 : None}
        for i in range(20):
            clearbox_1_name = 'clearbox_1_' + str(i)
            clearbox_1_mesh = get_object_mesh(f'clearbox_{i}' + '.stl', scale=[1.7, 1.7, 1.7])
            self.scene_mngr.add_object(name=clearbox_1_name, gtype="mesh", h_mat=self.clearbox1_pose.h_mat, gparam=clearbox_1_mesh, color=[0.8 + i*0.01, 0.8 + i*0.01, 0.8 + i*0.01])
            self.scene_mngr.set_logical_state(clearbox_1_name, (self.scene_mngr.scene.logical_state.static, True))
            
        self.scene_mngr.add_object(name="table", gtype="mesh", gparam=self.table_mesh, h_mat=self.table_pose.h_mat, color=[0.39, 0.263, 0.129])
        self.scene_mngr.add_object(name="arch_box", gtype="mesh", gparam=self.arch_box, h_mat=self.arch_box_pose.h_mat)
        self.scene_mngr.add_object(name="can", gtype="mesh", gparam=self.can, h_mat=self.can_pose.h_mat)
        self.scene_mngr.add_object(name="rect_box", gtype="mesh", gparam=self.rect_box, h_mat=self.rect_box_pose.h_mat)
        self.scene_mngr.add_object(name="half_cylinder_box", gtype="mesh", gparam=self.half_cylinder_box, h_mat=self.half_cylinder_box_pose.h_mat)
        self.scene_mngr.add_object(name="square_box", gtype="mesh", gparam=self.square_box, h_mat=self.square_box_pose.h_mat)
        self.scene_mngr.add_robot(self.robot)
        
        self.scene_mngr.set_logical_state("arch_box", ("on", "table"))
        self.scene_mngr.set_logical_state("can", ("on", "table"))
        self.scene_mngr.set_logical_state("rect_box", ("on", "table"))
        self.scene_mngr.set_logical_state("half_cylinder_box", ("on", "table"))
        self.scene_mngr.set_logical_state("square_box", ("on", "table"))
        self.scene_mngr.set_logical_state("table", (self.scene_mngr.scene.logical_state.static, True))
        self.scene_mngr.set_logical_state(self.scene_mngr.gripper_name, (self.scene_mngr.scene.logical_state.holding, None))
        
        self.scene_mngr.update_logical_states(is_init=True)
        self.scene_mngr.show_logical_states()