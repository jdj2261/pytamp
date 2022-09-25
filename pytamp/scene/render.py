import numpy as np
import trimesh
from abc import abstractclassmethod, ABCMeta

from pykin.utils import plot_utils as p_utils
from pykin.utils.kin_utils import apply_robot_to_scene, apply_objects_to_scene, apply_gripper_to_scene

class SceneRender(metaclass=ABCMeta):

    @abstractclassmethod
    def render_scene():
        raise NotImplementedError

    @abstractclassmethod
    def render_objects_and_gripper():
        raise NotImplementedError
    
    @abstractclassmethod
    def render_objects():
        raise NotImplementedError

    @abstractclassmethod
    def render_gripper():
        raise NotImplementedError

    @abstractclassmethod
    def show():
        raise NotImplementedError

class RenderTriMesh(SceneRender):

    def __init__(self):
        self.trimesh_scene = trimesh.Scene()
    
    def render_scene(self, objs, robot, geom="collision"):
        self.render_objects(objs)
        self.render_robot(robot, geom)

    def render_objects_and_gripper(self, objs, robot, geom="collision"):
        self.render_objects(objs)
        self.render_gripper(robot, geom)

    def render_objects(self, objs):
        self.trimesh_scene = apply_objects_to_scene(trimesh_scene=self.trimesh_scene, objs=objs)

    def render_object(self, obj, pose=None):
        o_type = obj.gtype
        mesh = obj.gparam
        if o_type == "mesh":
            mesh = obj.gparam
            mesh.visual.face_colors = obj.color
            self.trimesh_scene.add_geometry(mesh, transform=pose)

    def render_robot(self, robot, geom):
        self.trimesh_scene = apply_robot_to_scene(trimesh_scene=self.trimesh_scene, robot=robot, geom=geom)
        
    def render_gripper(self, robot, geom):
        self.trimesh_scene = apply_gripper_to_scene(trimesh_scene=self.trimesh_scene, robot=robot, geom=geom)

    def render_axis(
        self,
        pose,
    ):
        axis = trimesh.creation.axis(origin_size=0.01, transform=pose)
        self.trimesh_scene.add_geometry(axis)

    def render_point(self, ax=None, point=np.zeros(3), radius=0.001, color=[1.0, 0.0, 0.]):
        pose = np.eye(4)
        pose[:3, 3] = point
        sphere_mesh = trimesh.creation.icosphere(radius=radius)
        sphere_mesh.visual.face_colors = color
        self.trimesh_scene.add_geometry(sphere_mesh, transform=pose)

    def show(self):
        # self.trimesh_scene.set_camera(np.array([-np.pi/2 - np.pi/6, np.pi, np.pi/6 + np.pi/2]), 3, resolution=(640, 480))
        self.trimesh_scene.set_camera(np.array([-np.pi/2 - np.pi/3, np.pi, -np.pi/6]), 2.5, resolution=(640, 480))
        self.trimesh_scene.show('gl')
        self.trimesh_scene = None

class RenderPyPlot(SceneRender):

    @staticmethod
    def render_scene(ax, objs, robot, alpha, robot_color, geom, only_visible_geom, visible_text):
        RenderPyPlot.render_objects(ax, objs, alpha)
        RenderPyPlot.render_robot(ax, robot, alpha, robot_color, geom, only_visible_geom, visible_text)

    @staticmethod
    def render_objects_and_gripper(ax, objs, robot, alpha, robot_color, visible_tcp):
        RenderPyPlot.render_objects(ax, objs, alpha)
        RenderPyPlot.render_gripper(ax, robot, alpha, robot_color, visible_tcp)

    @staticmethod
    def render_objects(ax, objs, alpha=1.0):
        p_utils.plot_objects(ax, objs, alpha)

    @staticmethod
    def render_object(ax, obj, pose=None, alpha=1.0):
        p_utils.plot_object(ax, obj, pose, alpha)

    @staticmethod
    def render_robot(ax, robot, alpha, robot_color=None, geom="collision", only_visible_geom=True, visible_text=True, visible_gripper=False):
        p_utils.plot_robot(
            ax, 
            robot, 
            alpha=alpha, 
            color=robot_color,
            geom=geom,
            only_visible_geom=only_visible_geom,
            visible_text=visible_text)
        if visible_gripper:
            RenderPyPlot.render_gripper(ax, robot, alpha, robot_color)

    @staticmethod
    def render_gripper(ax, robot, alpha=0.9, robot_color=None, visible_tcp=True, pose=None, only_visible_axis=False):
        p_utils.plot_basis(ax, robot) 

        if pose is not None:
            robot.gripper.set_gripper_pose(pose)
        gripper_info =  robot.gripper.info

        if visible_tcp:
            ax.scatter(
                robot.gripper.info["tcp"][3][0,3], 
                robot.gripper.info["tcp"][3][1,3], 
                robot.gripper.info["tcp"][3][2,3], s=5, c='r')

        if only_visible_axis:
            for link, info in gripper_info.items():
                if link == "right_gripper":
                    RenderPyPlot.render_axis(ax, info[3])
            return

        for link, info in gripper_info.items():
            if link == "collision_pad":
                continue
            p_utils.plot_geom_from_info(ax, robot, link, "collision", info, alpha, robot_color)

    @staticmethod
    def render_axis(
        ax,
        pose,
        axis=[1, 1, 1],
        scale=0.05
    ):
        p_utils.render_axis(ax, pose, axis, scale)

    @staticmethod
    def render_points(ax, points, s=5, c='r'):
        if isinstance(points, list):
            points = np.array(points).reshape(-1,3)
        for point in points:
            ax.scatter(point[0], point[1], point[2], s=s, c=c)

    @staticmethod
    def render_point(ax, point, s=5, c='r'):
        ax.scatter(point[0], point[1], point[2], s=s, c=c)

    @staticmethod
    def render_trajectory(ax, path, size=1, color='r'):
        p_utils.plot_trajectories(ax, path, size, color)

    @staticmethod
    def show():
        p_utils.show_figure()