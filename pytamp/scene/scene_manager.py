import numpy as np
import trimesh, io
import matplotlib.animation as animation
from collections import OrderedDict
from copy import deepcopy
from PIL import Image

from pykin.utils import plot_utils as p_utils
from pykin.robots.single_arm import SingleArm
from pykin.collision.collision_manager import CollisionManager
from pykin.utils.mesh_utils import get_relative_transform
from pykin.utils.kin_utils import apply_robot_to_scene
from pytamp.scene.scene import Scene
from pytamp.scene.object import Object
from pytamp.scene.render import RenderPyPlot, RenderTriMesh


class SceneManager:
    def __init__(
        self,
        geom="collision",
        is_pyplot=True,
        scene: Scene = None,
        benchmark: dict = {1: {"stack_num": 3, "goal_object": "goal_box"}},
        debug_mode=True,
    ):
        # Element for Scene
        self.geom = geom
        self._scene = scene
        self.is_debug_mode = debug_mode
        self._heuristic = True
        if scene is None:
            self._scene = Scene(benchmark)

        self.init_objects = OrderedDict()
        self.init_logical_states = OrderedDict()

        self.attached_obj_name = None
        self.save_grasp_pose = {}

        # Collision Manager
        self.obj_collision_mngr = CollisionManager()
        self.robot_collision_mngr = None
        self.gripper_collision_mngr = None

        # Render
        self._is_pyplot = is_pyplot
        if is_pyplot:
            self.render = RenderPyPlot()
        else:
            self.render = RenderTriMesh()

        # Attach / Detach
        self.is_attached = False

    def __repr__(self):
        return "pytamp.scene.scene.{}()".format(type(self).__name__)

    def add_object(self, name, gtype, gparam, h_mat=None, color=[0.0, 1.0, 0.0]):
        if name in self._scene.objs:
            raise ValueError("Duplicate name: object {} already exists".format(name))

        if h_mat is None:
            h_mat = np.eye(4, dtype=np.float32)

        self._scene.objs[name] = Object(name, gtype, gparam, h_mat, color)
        self.obj_collision_mngr.add_object(name, gtype, gparam, h_mat)

        self.init_objects[name] = deepcopy(self._scene.objs[name])

    def add_robot(self, robot: SingleArm, thetas=[]):
        if self._scene.robot is not None:
            raise ValueError("robot {} already exists".format(robot.robot_name))
        self._scene.robot = robot

        if np.array(thetas).size != 0:
            self._scene.robot.set_transform(thetas)
        else:
            self._scene.robot.set_transform(robot.init_qpos)

        self.robot_collision_mngr = CollisionManager(is_robot=True)
        self.robot_collision_mngr.setup_robot_collision(robot, geom=self.geom)

        if self._scene.robot.has_gripper:
            self.gripper_collision_mngr = CollisionManager()
            self.gripper_collision_mngr.setup_gripper_collision(robot)

    def remove_object(self, name):
        if name not in self._scene.objs:
            raise ValueError("object {} needs to be added first".format(name))

        self._scene.objs.pop(name, None)
        self.obj_collision_mngr.remove_object(name)

    def attach_object_on_gripper(self, name, has_transform_bet_gripper_n_obj=False):
        if self._scene.robot is None:
            raise ValueError("Robot needs to be added first")

        if name not in self._scene.objs:
            raise ValueError("object {} needs to be added first".format(name))

        self.is_attached = True
        self._scene.robot.gripper.is_attached = self.is_attached
        self.attached_obj_name = self._scene.objs[name].name
        self._scene.robot.gripper.attached_obj_name = self._scene.objs[name].name

        self.obj_collision_mngr.remove_object(name)

        if has_transform_bet_gripper_n_obj:
            self._transform_bet_gripper_n_obj = (
                self._scene.robot.gripper.transform_bet_gripper_n_obj
            )
        else:
            eef_pose = self.get_gripper_pose()
            self._transform_bet_gripper_n_obj = get_relative_transform(
                eef_pose, self._scene.objs[name].h_mat
            )

        self.robot_collision_mngr.add_object(
            self._scene.objs[name].name,
            self._scene.objs[name].gtype,
            self._scene.objs[name].gparam,
            self._scene.objs[name].h_mat,
        )
        self._scene.robot.info["collision"][name] = [
            self._scene.objs[name].name,
            self._scene.objs[name].gtype,
            self._scene.objs[name].gparam,
            self._scene.objs[name].h_mat,
        ]
        self._scene.robot.info["visual"][name] = [
            self._scene.objs[name].name,
            self._scene.objs[name].gtype,
            self._scene.objs[name].gparam,
            self._scene.objs[name].h_mat,
        ]

        self.gripper_collision_mngr.add_object(
            self._scene.objs[name].name,
            self._scene.objs[name].gtype,
            self._scene.objs[name].gparam,
            self._scene.objs[name].h_mat,
        )

        self._scene.robot.gripper.info[name] = [
            self._scene.objs[name].name,
            self._scene.objs[name].gtype,
            self._scene.objs[name].gparam,
            self._scene.objs[name].h_mat,
            self._scene.objs[name].color,
        ]
        self._scene.objs.pop(name, None)

    def detach_object_from_gripper(self, attached_object=None):
        if self._scene.robot is None:
            raise ValueError("Robot needs to be added first")

        if attached_object is None:
            attached_object = self.attached_obj_name

        self.robot_collision_mngr.remove_object(attached_object)
        self._scene.robot.info["collision"].pop(attached_object)
        self._scene.robot.info["visual"].pop(attached_object)

        self.gripper_collision_mngr.remove_object(attached_object)
        self._scene.robot.gripper.info.pop(attached_object)

        self.is_attached = False
        self._scene.robot.gripper.is_attached = False

    @staticmethod
    def set_key(dictionary, key, value):
        if key not in dictionary:
            dictionary[key] = value
        elif type(dictionary[key]) == list:
            dictionary[key].append(value)
        else:
            dictionary[key] = [dictionary[key], value]

    def set_logical_state(self, obj_name, *states: tuple):
        self._scene.logical_states[obj_name] = {}
        for state in states:
            if isinstance(state[1], str):
                if self._scene.logical_states[obj_name].get(state[0]) is not None:
                    if state[0] == list(self._scene.logical_states[obj_name].keys())[0]:
                        self.set_key(
                            self._scene.logical_states[obj_name],
                            state[0],
                            self._scene.objs[state[1]],
                        )
                else:
                    self._scene.logical_states[obj_name].update(
                        {state[0]: self._scene.objs[state[1]]}
                    )

            else:
                self._scene.logical_states[obj_name].update({state[0]: state[1]})

    def get_object_pose(self, name):
        if name not in self._scene.objs:
            raise ValueError("object {} needs to be added first".format(name))

        return self._scene.objs[name].h_mat

    def set_object_pose(self, name, pose):
        if name not in self._scene.objs:
            raise ValueError("object {} needs to be added first".format(name))

        if pose.shape != (4, 4):
            raise ValueError(
                "Expecting the shape of the pose to be (4,4), instead got: " "{}".format(pose.shape)
            )

        self._scene.objs[name].h_mat = pose
        self.obj_collision_mngr.set_transform(name, pose)

    def compute_ik(self, pose=np.eye(4), method="LM", max_iter=100):
        if self._scene.robot is None:
            raise ValueError("Robot needs to be added first")

        pose = np.asarray(pose)
        if pose.shape != (4, 4):
            raise ValueError(
                "Expecting the shape of the pose to be (4,4), instead got: " "{}".format(pose.shape)
            )

        return self._scene.robot.inverse_kin(
            current_joints=np.random.randn(self._scene.robot.arm_dof),
            target_pose=pose,
            method=method,
            max_iter=max_iter,
        )

    def get_robot_eef_pose(self):
        if self._scene.robot is None:
            raise ValueError("Robot needs to be added first")

        return self._scene.robot.info[self.geom][self._scene.robot.eef_name][3]

    def set_robot_eef_pose(self, thetas):
        if self._scene.robot is None:
            raise ValueError("Robot needs to be added first")

        self._scene.robot.set_transform(thetas)
        for link, info in self._scene.robot.info[self.geom].items():
            if link in self.robot_collision_mngr._objs:
                self.robot_collision_mngr.set_transform(link, info[3])

            if self._scene.robot.has_gripper:
                if link in self.gripper_collision_mngr._objs:
                    self.gripper_collision_mngr.set_transform(link, info[3])

        if self.is_attached:
            attached_obj_pose = np.dot(self.get_gripper_pose(), self._transform_bet_gripper_n_obj)
            self._scene.robot.info["collision"][self.attached_obj_name][3] = attached_obj_pose
            self._scene.robot.info["visual"][self.attached_obj_name][3] = attached_obj_pose
            self._scene.robot.gripper.info[self.attached_obj_name][3] = attached_obj_pose
            self.robot_collision_mngr.set_transform(self.attached_obj_name, attached_obj_pose)

    def get_gripper_pose(self):
        if not self._scene.robot.has_gripper:
            raise ValueError("Robot doesn't have a gripper")

        return self._scene.robot.gripper.get_gripper_pose()

    def set_gripper_pose(self, pose=np.eye(4)):
        if not self._scene.robot.has_gripper:
            raise ValueError("Robot doesn't have a gripper")

        self._scene.robot.gripper.set_gripper_pose(pose)
        for link, info in self._scene.robot.gripper.info.items():
            if link in self.gripper_collision_mngr._objs:
                self.gripper_collision_mngr.set_transform(link, info[3])

        if self.is_attached:
            attached_obj_pose = np.dot(self.get_gripper_pose(), self._transform_bet_gripper_n_obj)
            self._scene.robot.gripper.info[self.attached_obj_name][3] = attached_obj_pose
            self.gripper_collision_mngr.set_transform(self.attached_obj_name, attached_obj_pose)

    def get_gripper_tcp_pose(self):
        if not self._scene.robot.has_gripper:
            raise ValueError("Robot doesn't have a gripper")

        return self._scene.robot.gripper.get_gripper_tcp_pose()

    def set_gripper_tcp_pose(self, pose=np.eye(4)):
        if not self._scene.robot.has_gripper:
            raise ValueError("Robot doesn't have a gripper")

        self._scene.robot.gripper.set_gripper_tcp_pose(pose)
        for link, info in self._scene.robot.gripper.info.items():
            if link in self.gripper_collision_mngr._objs:
                self.gripper_collision_mngr.set_transform(link, info[3])

    def close_gripper(self, z_dis=None):
        if not self._scene.robot.has_gripper:
            raise ValueError("Robot doesn't have a gripper")

        if z_dis is None:
            if "panda" in self._scene.robot.gripper_name:
                z_dis = 0.003
            else:
                z_dis = 0.015

        self._scene.robot.close_gripper(z_dis)
        self._scene.robot.gripper.close_gripper(z_dis)

        for link, info in self._scene.robot.info[self.geom].items():
            if link in self._scene.robot.gripper.finger_names:
                self.robot_collision_mngr.set_transform(link, info[3])

            if self._scene.robot.has_gripper:
                if link in self._scene.robot.gripper.finger_names:
                    self.gripper_collision_mngr.set_transform(link, info[3])

    def open_gripper(self, z_dis=None):
        if not self._scene.robot.has_gripper:
            raise ValueError("Robot doesn't have a gripper")

        if z_dis is None:
            if "panda" in self._scene.robot.gripper_name:
                z_dis = 0.003
            else:
                z_dis = 0.015

        self._scene.robot.open_gripper(z_dis)
        self._scene.robot.gripper.open_gripper(z_dis)

        for link, info in self._scene.robot.info[self.geom].items():
            if link in self._scene.robot.gripper.finger_names:
                self.robot_collision_mngr.set_transform(link, info[3])

            if self._scene.robot.has_gripper:
                if link in self._scene.robot.gripper.finger_names:
                    self.gripper_collision_mngr.set_transform(link, info[3])

    def collide_objs_and_robot(self, return_names=False):
        if self._scene.robot is None:
            raise ValueError("Robot needs to be added first")
        return self.robot_collision_mngr.in_collision_other(self.obj_collision_mngr, return_names)

    def collide_self_robot(self, return_names=False):
        if self._scene.robot is None:
            raise ValueError("Robot needs to be added first")
        return self.robot_collision_mngr.in_collision_internal(return_names)

    def collide_objs_and_gripper(self, return_names=False):
        if not self._scene.robot.has_gripper:
            raise ValueError("Robot doesn't have a gripper")
        return self.gripper_collision_mngr.in_collision_other(self.obj_collision_mngr, return_names)

    def update_logical_states(self, is_init=False):
        self._scene.update_logical_states()
        if is_init:
            self.init_logical_states = deepcopy(self._scene.logical_states)
            self.init_scene = deepcopy(self._scene)

    def get_objs_info(self):
        return self._scene.objs

    def get_robot_info(self):
        if self._scene.robot is None:
            raise ValueError("Robot needs to be added first")

        return self._scene.robot.info

    def get_gripper_info(self):
        if not self._scene.robot.has_gripper:
            raise ValueError("Robot doesn't have a gripper")

        return self._scene.robot.gripper.info

    def show_scene_info(self):
        self._scene.show_scene_info()

    def show_logical_states(self):
        self._scene.show_logical_states()

    def render_debug(self, title="Error Scene"):
        fig, ax = p_utils.init_3d_figure(name=title)
        self.render_scene(ax)
        if self.scene.grasp_poses:
            self.render_axis(ax, self.scene.grasp_poses["grasp"])
            self.render_axis(ax, self.scene.grasp_poses["pre_grasp"])
            self.render_axis(ax, self.scene.grasp_poses["post_grasp"])
        if self.scene.release_poses:
            self.render_axis(ax, self.scene.release_poses["release"])
            self.render_axis(ax, self.scene.release_poses["pre_release"])
            self.render_axis(ax, self.scene.release_poses["post_release"])
        self.show()

    def render_scene(
        self,
        ax=None,
        scene=None,
        alpha=0.9,
        robot_color=None,
        only_visible_geom=True,
        visible_text=False,
        geom=None,
    ):
        scene = scene
        if scene is None:
            scene = self._scene

        if scene.robot is None:
            raise ValueError("Robot needs to be added first")

        if geom is None:
            geom = self.geom

        if self.is_pyplot:
            self.render.render_scene(
                ax,
                scene.objs,
                scene.robot,
                alpha,
                robot_color,
                geom=geom,
                only_visible_geom=only_visible_geom,
                visible_text=visible_text,
            )
        else:
            if not self.render.trimesh_scene:
                self.render = RenderTriMesh()
            print(scene.objs)
            self.render.render_scene(objs=scene.objs, robot=scene.robot, geom=geom)

    def render_objects_and_gripper(
        self, ax=None, scene=None, alpha=0.8, robot_color=None, visible_tcp=True
    ):
        scene = scene
        if scene is None:
            scene = self._scene

        if not scene.robot.has_gripper:
            raise ValueError("Robot doesn't have a gripper")

        if self.is_pyplot:
            self.render.render_objects_and_gripper(
                ax, scene.objs, scene.robot, alpha, robot_color, visible_tcp=visible_tcp
            )
        else:
            if not self.render.trimesh_scene:
                self.render = RenderTriMesh()
            self.render.render_objects_and_gripper(
                objs=scene.objs, robot=scene.robot, geom=self.geom
            )

    def render_objects(self, ax=None, scene=None, alpha=1.0):
        scene = scene
        if scene is None:
            scene = self._scene

        if self.is_pyplot:
            self.render.render_objects(ax, scene.objs, alpha)
        else:
            if not self.render.trimesh_scene:
                self.render = RenderTriMesh()
            self.render.render_objects(objs=scene.objs)

    def render_object(self, ax, obj, pose, alpha=0.8):
        if self.is_pyplot:
            self.render.render_object(ax, obj, pose, alpha)
        else:
            if not self.render.trimesh_scene:
                self.render = RenderTriMesh()
            self.render.render_object(obj, pose)

    def render_robot(
        self,
        ax=None,
        scene=None,
        alpha=0.3,
        robot_color=None,
        only_visible_geom=True,
        visible_text=False,
    ):
        scene = scene
        if scene is None:
            scene = self._scene

        if scene.robot is None:
            raise ValueError("Robot needs to be added first")

        if self.is_pyplot:
            self.render.render_robot(
                ax, scene.robot, alpha, robot_color, self.geom, only_visible_geom, visible_text
            )
        else:
            if not self.render.trimesh_scene:
                self.render = RenderTriMesh()
            self.render.render_robot(scene.robot, self.geom)

    def render_gripper(
        self,
        ax=None,
        scene=None,
        alpha=1.0,
        robot_color=None,
        visible_tcp=True,
        pose=None,
        only_visible_axis=False,
    ):
        scene = scene
        if scene is None:
            scene = self._scene

        if not scene.robot.has_gripper:
            raise ValueError("Robot doesn't have a gripper")

        if self.is_pyplot:
            self.render.render_gripper(
                ax=ax,
                robot=scene.robot,
                alpha=alpha,
                robot_color=robot_color,
                visible_tcp=visible_tcp,
                pose=pose,
                only_visible_axis=only_visible_axis,
            )
        else:
            if not self.render.trimesh_scene:
                self.render = RenderTriMesh()
            self.render.render_gripper(scene.robot, self.geom)

    def render_axis(self, ax, pose, scale=0.05):
        if self.is_pyplot:
            self.render.render_axis(ax, pose, axis=[1, 1, 1], scale=scale)
        else:
            if not self.render.trimesh_scene:
                self.render = RenderTriMesh()
            self.render.render_axis(pose)

    def animation(
        self,
        ax=None,
        fig=None,
        init_scene=None,
        alpha=0.3,
        robot_color=None,
        joint_path=[],
        eef_poses=[],
        visible_gripper=False,
        visible_text=True,
        interval=50,
        repeat=True,
        pick_object=None,
        attach_idx: list = None,
        detach_idx: list = None,
        place_obj_pose=None,
        is_save=False,
        video_name="test",
        fps=30,
    ):
        self.is_pyplot = True

        if init_scene is not None:
            self._scene = deepcopy(init_scene)

        if pick_object is None:
            pick_object = self.attached_obj_name

        self.is_attach = False

        def update(i):
            ax.clear()
            ax._axis3don = False

            if self._scene.objs:
                self.render.render_objects(ax, self._scene.objs, alpha)

            if eef_poses is not None:
                self.render.render_trajectory(ax, eef_poses, size=0.1)

            self.set_robot_eef_pose(joint_path[i])

            if attach_idx is not None:
                if i in attach_idx:
                    idx = attach_idx.index(i)
                    self.attach_object_on_gripper(pick_object[idx], False)
                    self.is_attach = True

            if self.is_attach:
                self.close_gripper()

            if detach_idx is not None:
                if i in detach_idx:
                    idx = detach_idx.index(i)
                    if place_obj_pose is None:
                        object_pose = self.get_gripper_info()[pick_object[idx]][3]
                    else:
                        object_pose = place_obj_pose[idx]
                    self.detach_object_from_gripper(pick_object[idx])
                    self.add_object(
                        name=pick_object[idx],
                        gtype=self.init_objects[pick_object[idx]].gtype,
                        gparam=self.init_objects[pick_object[idx]].gparam,
                        h_mat=object_pose,
                        color=self.init_objects[pick_object[idx]].color,
                    )
                    self.is_attach = False
                    self.open_gripper()

            visible_geom = True
            if visible_gripper:
                visible_geom = False

            self.render.render_robot(
                ax=ax,
                robot=self._scene.robot,
                alpha=alpha,
                robot_color=robot_color,
                geom=self.geom,
                only_visible_geom=visible_geom,
                visible_text=visible_text,
                visible_gripper=visible_gripper,
            )

            if i == len(joint_path) - 1:
                print("Animation Finished..")

        ani = animation.FuncAnimation(
            fig, update, np.arange(len(joint_path)), interval=interval, repeat=repeat
        )
        if is_save:
            video_name = video_name + ".mp4"
            ani.save(video_name, writer="ffmpeg", fps=fps)
            print("Save finished..")
        else:
            self.show()

    def show_scene(
        self,
        init_scene=None,
        joint_path=[],
        pick_object=None,
        attach_idx: list = None,
        detach_idx: list = None,
        place_obj_pose=None,
        is_save=False,
    ):
        # if init_scene is not None:
        #     self._scene = deepcopy(init_scene)

        # if pick_object is None:
        #     pick_object = self.attached_obj_name

        # self.is_attach = False

        for idx, joint in enumerate(joint_path):
            print(joint)
            trimesh_scene = trimesh.Scene()

            self.set_robot_eef_pose(joint)
            trimesh_scene = apply_robot_to_scene(
                trimesh_scene=trimesh_scene, robot=self.scene.robot, geom="visual"
            )
            trimesh_scene.set_camera(np.array([np.pi / 2, 0, np.pi / 2]), 5, resolution=(640, 512))
            # trimesh_scene.show()
            # self.render_scene()
            data = trimesh_scene.save_image(visible=True)
            data_io = io.BytesIO(data)
            # img = Image.open(data_io)
            im = Image.open(data_io)
            file_name = "test" + str(idx) + ".png"
            im.save(file_name)

    def show(self):
        self.render.show()

    def reset(self):
        self.obj_collision_mngr = None
        self._scene.objs = OrderedDict()

        if self._scene.robot is not None:
            self.robot_collision_mngr = None
            self._scene.robot = None
        if self._scene.robot.has_gripper:
            self.gripper_collision_mngr = None
            self._scene.robot.gripper = None

    def deepcopy_scene(self, scene_mngr=None):
        copied_scene = SceneManager(benchmark=self.scene.benchmark_config)
        if scene_mngr is None:
            scene_mngr = self
        for k, v in scene_mngr.__dict__.items():
            if not "collision_mngr" in k:
                copied_scene.__dict__[k] = deepcopy(v)
            else:
                copied_scene.__dict__[k] = v
        return copied_scene

    @property
    def gripper_name(self):
        if not self._scene.robot.has_gripper:
            raise ValueError("Robot doesn't have a gripper")

        return self._scene.robot.gripper.name

    @property
    def scene(self):
        return self._scene

    @scene.setter
    def scene(self, scene):
        self._scene = scene

    @property
    def heuristic(self):
        return self._heuristic

    @heuristic.setter
    def heuristic(self, heuristic):
        self._heuristic = heuristic

    @property
    def is_pyplot(self):
        return self._is_pyplot

    @is_pyplot.setter
    def is_pyplot(self, is_pyplot):
        self._is_pyplot = is_pyplot
        if is_pyplot:
            self.render = RenderPyPlot()
        else:
            self.render = RenderTriMesh()
