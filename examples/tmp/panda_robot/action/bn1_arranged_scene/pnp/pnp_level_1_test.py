import numpy as np

from pykin.utils import plot_utils as p_utils
from pykin.kinematics.transform import Transform
from pykin.robots.single_arm import SingleArm
from pykin.utils.mesh_utils import get_object_mesh

from pytamp.action.pick import PickAction
from pytamp.action.place import PlaceAction
from pytamp.scene.scene_manager import SceneManager

file_path = 'urdf/panda/panda.urdf'
robot = SingleArm(
    f_name=file_path, 
    offset=Transform(rot=[0.0, 0.0, 0.0], pos=[0, 0, 0.913]), 
    has_gripper=True)
robot.setup_link_name("panda_link_0", "right_hand")
robot.init_qpos = np.array([0, np.pi / 16.0, 0.00, -np.pi / 2.0 - np.pi / 3.0, 0.00, np.pi - 0.2, -np.pi/4])



red_box_pose = Transform(pos=np.array([0.6, 0.2, 0.77]))
blue_box_pose = Transform(pos=np.array([0.6, 0.35, 0.77]))
green_box_pose = Transform(pos=np.array([0.6, 0.05, 0.77]))
support_box_pose = Transform(pos=np.array([0.6, -0.2, 0.77]), rot=np.array([0, np.pi/2, 0]))
table_pose = Transform(pos=np.array([0.4, 0.24, 0.0]))

red_cube_mesh = get_object_mesh('ben_cube.stl', 0.06)
blue_cube_mesh = get_object_mesh('ben_cube.stl', 0.06)
green_cube_mesh = get_object_mesh('ben_cube.stl', 0.06)
goal_box_mesh = get_object_mesh('goal_box.stl', 0.001)
table_mesh = get_object_mesh('custom_table.stl', 0.01)

scene_mngr = SceneManager("collision", is_pyplot=True)
scene_mngr.add_object(name="table", gtype="mesh", gparam=table_mesh, h_mat=table_pose.h_mat, color=[0.39, 0.263, 0.129])
scene_mngr.add_object(name="red_box", gtype="mesh", gparam=red_cube_mesh, h_mat=red_box_pose.h_mat, color=[1.0, 0.0, 0.0])
scene_mngr.add_object(name="blue_box", gtype="mesh", gparam=blue_cube_mesh, h_mat=blue_box_pose.h_mat, color=[0.0, 0.0, 1.0])
scene_mngr.add_object(name="green_box", gtype="mesh", gparam=green_cube_mesh, h_mat=green_box_pose.h_mat, color=[0.0, 1.0, 0.0])
scene_mngr.add_object(name="goal_box", gtype="mesh", gparam=goal_box_mesh, h_mat=support_box_pose.h_mat, color=[1.0, 0, 1.0])
scene_mngr.add_robot(robot, robot.init_qpos)

scene_mngr.set_logical_state("goal_box", ("on", "table"))
scene_mngr.set_logical_state("red_box", ("on", "table"))
scene_mngr.set_logical_state("blue_box", ("on", "table"))
scene_mngr.set_logical_state("green_box", ("on", "table"))
scene_mngr.set_logical_state("table", ("static", True))
scene_mngr.set_logical_state(scene_mngr.gripper_name, ("holding", None))
scene_mngr.update_logical_states(is_init=True)

pick = PickAction(scene_mngr, n_contacts=5, n_directions=5)
place = PlaceAction(scene_mngr, n_samples_held_obj=3, n_samples_support_obj=3)

pick_actions = list(pick.get_possible_actions_level_1())

for pick_action in pick_actions:
    for pick_scene in pick.get_possible_transitions(scene_mngr.scene, action=pick_action):
        place_actions = list(place.get_possible_actions_level_1(pick_scene)) 
        for place_action in place_actions:
            for place_scene in place.get_possible_transitions(scene=pick_scene, action=place_action):
                fig, ax = p_utils.init_3d_figure( name="all possible pick transitions")
                place.scene_mngr.render_gripper(ax, pick_scene, alpha=0.9, only_visible_axis=False)
                pick_scene.show_logical_states()    
                place.scene_mngr.render_objects(ax, pick_scene)
                place.scene_mngr.show()

                fig, ax = p_utils.init_3d_figure( name="all possible place transitions")
                place.scene_mngr.render_gripper(ax, place_scene, alpha=0.9, only_visible_axis=False)
                place_scene.show_logical_states()                                
                place.scene_mngr.render_objects(ax, place_scene)
                place.scene_mngr.show()