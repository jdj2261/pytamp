import numpy as np

from pykin.kinematics.transform import Transform
from pykin.utils.mesh_utils import get_object_mesh
from pytamp.scene.scene_manager import SceneManager
from pytamp.benchmark.benchmark import Benchmark

class Benchmark2(Benchmark):
    def __init__(
        self, 
        robot_name="panda", 
        geom="visual", 
        is_pyplot=True
    ):
        self.geom = geom
        self.is_pyplot = is_pyplot
        super().__init__(robot_name)

        self._load_objects()
        self._load_scene()

    def _load_objects(self):
        self.shelf_pose = Transform(pos=np.array([0.9, 0, 1.41725156]),rot=np.array([0, 0, np.pi/2]))
        self.bin_pose = Transform(pos=np.array([0.0, 1.0, 0.3864222]))
        self.bottle_meshes = []
        for i in range(6):
            self.bottle_meshes.append(get_object_mesh('bottle.stl'))
        self.bottle_pose1 = Transform(pos=np.array([1.0, 0, 1.29]))
        self.bottle_pose2 = Transform(pos=np.array([0.95, 0.05, 1.29]))
        self.bottle_pose3 = Transform(pos=np.array([0.95, -0.05,1.29]))
        self.bottle_pose4 = Transform(pos=np.array([0.90, 0.1, 1.29]))
        self.bottle_pose5 = Transform(pos=np.array([0.90, 0, 1.29]))
        self.bottle_pose6 = Transform(pos=np.array([0.90, -0.1, 1.29]))
    
    def _load_scene(self):
        self.benchmark_config = {2 : None}
        self.scene_mngr = SceneManager(self.geom, is_pyplot=self.is_pyplot, benchmark=self.benchmark_config)

        for i in range(20):
            shelf_name = 'shelf_' + str(i)
            self.shelf_mesh = get_object_mesh(shelf_name + '.stl', scale=0.9)
            self.scene_mngr.add_object(name=shelf_name, gtype="mesh", h_mat=self.shelf_pose.h_mat, gparam=self.shelf_mesh, color=[0.39, 0.263, 0.129])

        for i in range(20):
            bin_name = 'bin_' + str(i)
            bin_mesh = get_object_mesh(bin_name + '.stl', scale=0.9)
            self.scene_mngr.add_object(name=bin_name, gtype="mesh", h_mat=self.bin_pose.h_mat, gparam=bin_mesh, color=[0.8 + i*0.01, 0.8 + i*0.01, 0.8 + i*0.01])

        self.scene_mngr.add_object(name="bottle_1", gtype="mesh", h_mat=self.bottle_pose1.h_mat, gparam=self.bottle_meshes[0], color=[1., 0., 0.])
        self.scene_mngr.add_object(name="bottle_2", gtype="mesh", h_mat=self.bottle_pose2.h_mat, gparam=self.bottle_meshes[1], color=[0., 1., 0.])
        self.scene_mngr.add_object(name="bottle_3", gtype="mesh", h_mat=self.bottle_pose3.h_mat, gparam=self.bottle_meshes[2], color=[0., 1., 0.])
        self.scene_mngr.add_object(name="bottle_4", gtype="mesh", h_mat=self.bottle_pose4.h_mat, gparam=self.bottle_meshes[3], color=[0., 1., 0.])
        self.scene_mngr.add_object(name="bottle_5", gtype="mesh", h_mat=self.bottle_pose5.h_mat, gparam=self.bottle_meshes[4], color=[0., 1., 0.])
        self.scene_mngr.add_object(name="bottle_6", gtype="mesh", h_mat=self.bottle_pose6.h_mat, gparam=self.bottle_meshes[5], color=[0., 1., 0.])
        self.scene_mngr.add_robot(self.robot, self.robot.init_qpos)

        self.scene_mngr.set_logical_state("bottle_1", ("on", "shelf_9"))
        self.scene_mngr.set_logical_state("bottle_2", ("on", "shelf_9"))
        self.scene_mngr.set_logical_state("bottle_3", ("on", "shelf_9"))
        self.scene_mngr.set_logical_state("bottle_4", ("on", "shelf_9"))
        self.scene_mngr.set_logical_state("bottle_5", ("on", "shelf_9"))
        self.scene_mngr.set_logical_state("bottle_6", ("on", "shelf_9"))

        for i in range(20):
            self.scene_mngr.set_logical_state(f"shelf_"+str(i), (self.scene_mngr.scene.logical_state.static, True))
            self.scene_mngr.set_logical_state(f"bin_"+str(i), (self.scene_mngr.scene.logical_state.static, True))
        self.scene_mngr.set_logical_state(self.scene_mngr.gripper_name, (self.scene_mngr.scene.logical_state.holding, None))
        self.scene_mngr.update_logical_states(is_init=True)
        self.scene_mngr.show_logical_states()