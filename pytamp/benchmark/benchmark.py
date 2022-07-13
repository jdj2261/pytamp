from abc import abstractclassmethod, ABCMeta

class Benchmark(metaclass=ABCMeta):
    def __init__(self, robot_name):
        self.robot_name = robot_name
        
    @abstractclassmethod
    def load_robot(self):
        raise NotImplementedError
    
    @abstractclassmethod
    def load_objects(self):
        raise NotImplementedError

    @abstractclassmethod
    def load_scene(self):
        raise NotImplementedError