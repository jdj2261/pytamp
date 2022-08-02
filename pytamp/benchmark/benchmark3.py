import numpy as np

from pykin.robots.single_arm import SingleArm
from pykin.kinematics.transform import Transform
from pykin.utils.mesh_utils import get_object_mesh
from pykin.utils.transform_utils import get_h_mat
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
        self.table_height = self.table_mesh.bounds[1][2] - self.table_mesh.bounds[0][2]
        self.table_pose = Transform(pos=np.array([1.0, -0.6, -self.table_mesh.bounds[0][2]]))
        
        #! clearbox 8, 16 is placing spot
        clearbox_8_mesh = get_object_mesh(f'clearbox_8.stl', scale=[1.4, 1.3, 1.5])
        self.clearbox1_pose = Transform(pos=np.array([0.7, 0.4, self.table_height + abs(clearbox_8_mesh.bounds[0][2])]), rot=[0, 0, np.pi/2])

        self.rect_bottom_box = get_object_mesh('rect_box.stl', [0.002, 0.0025, 0.0005])
        self.rect_bottom_box.apply_translation(-self.rect_bottom_box.center_mass)

        self.rect_bottom_right_box = get_object_mesh('rect_box.stl', [0.002, 0.001, 0.0005])
        self.rect_bottom_right_box.apply_transform(get_h_mat(orientation=np.array([np.pi/2, 0, 0])))
        self.rect_bottom_right_box.apply_translation(-self.rect_bottom_right_box.center_mass)

        self.rect_bottom_center_box = get_object_mesh('rect_box.stl', [0.002, 0.001, 0.0005])
        self.rect_bottom_center_box.apply_transform(get_h_mat(orientation=np.array([np.pi/2, 0, 0])))
        self.rect_bottom_center_box.apply_translation(-self.rect_bottom_center_box.center_mass)

        self.rect_bottom_left_box = get_object_mesh('rect_box.stl', [0.002, 0.001, 0.0005])
        self.rect_bottom_left_box.apply_transform(get_h_mat(orientation=np.array([np.pi/2, 0, 0])))
        self.rect_bottom_left_box.apply_translation(-self.rect_bottom_left_box.center_mass)

        self.rect_top_box = get_object_mesh('rect_box.stl', [0.002, 0.002, 0.0005])
        self.rect_top_box.apply_translation(-self.rect_top_box.center_mass)

        self.rect_top_right_box = get_object_mesh('rect_box.stl', [0.002, 0.001, 0.0005])
        self.rect_top_right_box.apply_transform(get_h_mat(orientation=np.array([np.pi/2, 0, 0])))
        self.rect_top_right_box.apply_translation(-self.rect_top_right_box.center_mass)

        self.rect_top_left_box = get_object_mesh('rect_box.stl', [0.002, 0.001, 0.0005])
        self.rect_top_left_box.apply_transform(get_h_mat(orientation=np.array([np.pi/2, 0, 0])))
        self.rect_top_left_box.apply_translation(-self.rect_top_left_box.center_mass)

        self.half_cylinder_box = get_object_mesh('half_cylinder_box.stl', [0.002, 0.002, 0.002])
        self.half_cylinder_box.apply_transform(get_h_mat(orientation=[0, -np.pi/2, 0]))
        self.half_cylinder_box.apply_translation(-self.half_cylinder_box.center_mass)

        self.rect_bottom_box_pose = Transform(np.array([0.65, -0.3, self.table_height + self.rect_bottom_box.bounds[1][2]]))
        self.rect_bottom_right_box_pose = Transform(pos=np.array([0.65, -0.2, self.table_height + self.rect_bottom_right_box.bounds[1][2]+ self.rect_bottom_box.bounds[1][2]*2]))
        self.rect_bottom_center_box_pose = Transform(pos=np.array([0.65, -0.3, self.table_height + self.rect_bottom_center_box.bounds[1][2]+ self.rect_bottom_box.bounds[1][2]*2]))
        self.rect_bottom_left_box_pose = Transform(pos=np.array([0.65, -0.4, self.table_height + self.rect_bottom_left_box.bounds[1][2]+ self.rect_bottom_box.bounds[1][2]*2]))
        self.rect_top_box_pose = Transform(np.array([0.65, -0.3, self.table_height + self.rect_bottom_box.bounds[1][2]*2 + self.rect_bottom_center_box.bounds[1][2]*2]))
        self.rect_top_right_box_pose = Transform(pos=np.array([0.65, -0.25, self.table_height + self.rect_bottom_box.bounds[1][2]*3 + self.rect_bottom_center_box.bounds[1][2]*3]))
        self.rect_top_left_box_pose = Transform(pos=np.array([0.65, -0.35, self.table_height + self.rect_bottom_box.bounds[1][2]*3 + self.rect_bottom_center_box.bounds[1][2]*3]))
        self.half_cylinder_box_pose = Transform(pos=np.array([0.65, -0.3, self.table_height + self.rect_bottom_box.bounds[1][2]*4 + self.rect_bottom_center_box.bounds[1][2]*4]))

    def _load_scene(self):
        self.benchmark_config = {3 : None}
        self.scene_mngr.add_object(name="table", gtype="mesh", gparam=self.table_mesh, h_mat=self.table_pose.h_mat, color=[0.39, 0.263, 0.129])
        self.scene_mngr.add_object(name="rect_bottom_box", gtype="mesh", gparam=self.rect_bottom_box, h_mat=self.rect_bottom_box_pose.h_mat)
        self.scene_mngr.add_object(name="rect_bottom_right_box", gtype="mesh", gparam=self.rect_bottom_right_box, h_mat=self.rect_bottom_right_box_pose.h_mat)
        self.scene_mngr.add_object(name="rect_bottom_center_box", gtype="mesh", gparam=self.rect_bottom_center_box, h_mat=self.rect_bottom_center_box_pose.h_mat)
        self.scene_mngr.add_object(name="rect_bottom_left_box", gtype="mesh", gparam=self.rect_bottom_left_box, h_mat=self.rect_bottom_left_box_pose.h_mat)
        self.scene_mngr.add_object(name="rect_top_box", gtype="mesh", gparam=self.rect_top_box, h_mat=self.rect_top_box_pose.h_mat)
        self.scene_mngr.add_object(name="rect_top_right_box", gtype="mesh", gparam=self.rect_top_right_box, h_mat=self.rect_top_right_box_pose.h_mat)
        self.scene_mngr.add_object(name="rect_top_left_box", gtype="mesh", gparam=self.rect_top_left_box, h_mat=self.rect_top_left_box_pose.h_mat)
        self.scene_mngr.add_object(name="half_cylinder_box", gtype="mesh", gparam=self.half_cylinder_box, h_mat=self.half_cylinder_box_pose.h_mat)
        self.scene_mngr.add_robot(self.robot)
        
        for i in range(20):
            clearbox_1_name = 'clearbox_1_' + str(i)
            clearbox_1_mesh = get_object_mesh(f'clearbox_{i}' + '.stl', scale=[1.4, 1.3, 1.5])
            self.scene_mngr.add_object(name=clearbox_1_name, gtype="mesh", h_mat=self.clearbox1_pose.h_mat, gparam=clearbox_1_mesh, color=[0.8 + i*0.01, 0.8 + i*0.01, 0.8 + i*0.01])
            self.scene_mngr.set_logical_state(clearbox_1_name, (self.scene_mngr.scene.logical_state.static, True), ("on", "table"))
            
        self.scene_mngr.set_logical_state("rect_bottom_box", ("on", "table"))
        self.scene_mngr.set_logical_state("rect_bottom_right_box", ("on", "rect_bottom_box"))
        self.scene_mngr.set_logical_state("rect_bottom_center_box", ("on", "rect_bottom_box"))
        self.scene_mngr.set_logical_state("rect_bottom_left_box", ("on", "rect_bottom_box"))
        self.scene_mngr.set_logical_state("rect_top_box", ("on", "rect_bottom_right_box"), ("on", "rect_bottom_center_box"), ("on", "rect_bottom_left_box"))
        self.scene_mngr.set_logical_state("rect_top_right_box", ("on", "rect_top_box"))
        self.scene_mngr.set_logical_state("rect_top_left_box", ("on", "rect_top_box"))
        self.scene_mngr.set_logical_state("half_cylinder_box", ("on", "rect_top_right_box"), ("on", "rect_top_left_box"))
        self.scene_mngr.set_logical_state("table", (self.scene_mngr.scene.logical_state.static, True))
        self.scene_mngr.set_logical_state(self.scene_mngr.gripper_name, (self.scene_mngr.scene.logical_state.holding, None))
        
        self.scene_mngr.update_logical_states(is_init=True)
        self.scene_mngr.show_logical_states()