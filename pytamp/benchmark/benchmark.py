from pytamp.scene.scene_manager import SceneManager

class Benchmark:
    def __init__(
        self, 
        robot_name,
        geom="collision",
        is_pyplot=True,
        benchmark_config=None
    ):
        self.robot_name = robot_name
        self.geom = geom
        self.is_pyplot = is_pyplot
        self.benchmark_config = benchmark_config

        urdf_file_name = self.robot_name
        self.gripper_name = "panda"
        if self.robot_name == "doosan":
            urdf_file_name = "doosan_with_robotiq140"
            self.gripper_name = "robotiq140"
        self.urdf_file = 'urdf/' + self.robot_name + '/' + urdf_file_name + '.urdf'

        self.scene_mngr = SceneManager(self.geom, is_pyplot=self.is_pyplot, benchmark=benchmark_config, debug_mode=True)
    
    def _load_robot(self):
        pass

    def _load_objects(self):
        pass
        
    def _load_scene(self):
        pass
        