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
# blue_box_pose = Transform(pos=np.array([0.6, 0.35, 0.77]))
# green_box_pose = Transform(pos=np.array([0.6, 0.05, 0.77]))
blue_box_pose = Transform(pos=np.array([0.6, 0.2, 0.77 + 0.06]))
green_box_pose = Transform(pos=np.array([0.6, 0.2, 0.77 + 0.12]))
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

scene_mngr.scene.logical_states["goal_box"] = {scene_mngr.scene.logical_state.on : scene_mngr.scene.objs["table"]}
scene_mngr.scene.logical_states["red_box"] = {scene_mngr.scene.logical_state.on : scene_mngr.scene.objs["table"]}
scene_mngr.scene.logical_states["blue_box"] = {scene_mngr.scene.logical_state.on : scene_mngr.scene.objs["red_box"]}
scene_mngr.scene.logical_states["green_box"] = {scene_mngr.scene.logical_state.on : scene_mngr.scene.objs["blue_box"]}
scene_mngr.scene.logical_states["table"] = {scene_mngr.scene.logical_state.static : True}
scene_mngr.scene.logical_states[scene_mngr.gripper_name] = {scene_mngr.scene.logical_state.holding : None}
scene_mngr.update_logical_states()


pick = PickAction(scene_mngr, n_contacts=5, n_directions=5)
place = PlaceAction(scene_mngr, n_samples_held_obj=3, n_samples_support_obj=3)

# pick_actions = list(pick.get_possible_actions_level_1())
# # fig, ax = p_utils.init_3d_figure(name="Level wise 1")
# for pick_action in pick_actions:
#     for pick_scene in pick.get_possible_transitions(scene_mngr.scene, action=pick_action):
#         place_actions = list(place.get_possible_actions_level_1(pick_scene)) 
#         for place_action in place_actions:
#             for place_scene in place.get_possible_transitions(scene=pick_scene, action=place_action):
#                 pick_actions2 = list(pick.get_possible_actions_level_1(place_scene))
#                 for pick_action2 in pick_actions2:
#                     for pick_scene_2 in pick.get_possible_transitions(place_scene, action=pick_action2):
#                         for place_action2 in list(place.get_possible_actions_level_1(pick_scene_2)):
#                             for place_scene2 in place.get_possible_transitions(pick_scene_2, action=place_action2):
#                                 fig, ax = p_utils.init_3d_figure( name="all possible transitions")
#                                 place.scene_mngr.render_gripper(ax, place_scene2, alpha=0.9, only_visible_axis=False)
#                                 place.scene_mngr.render_objects(ax, place_scene2)
#                                 place_scene2.show_logical_states()
#                                 place.scene_mngr.show()


pick_actions = list(pick.get_possible_actions_level_1())
# fig, ax = p_utils.init_3d_figure(name="Level wise 1")
for pick_action in pick_actions:
    for pick_scene in pick.get_possible_transitions(scene_mngr.scene, action=pick_action):
        place_actions = list(place.get_possible_actions_level_1(pick_scene)) 
        for place_action in place_actions:
            for place_scene in place.get_possible_transitions(scene=pick_scene, action=place_action):
                pick_actions2 = list(pick.get_possible_actions_level_1(place_scene))
                for pick_action2 in pick_actions2:
                    for pick_scene_2 in pick.get_possible_transitions(place_scene, action=pick_action2):
                        for place_action2 in list(place.get_possible_actions_level_1(pick_scene_2)):
                            for place_scene2 in place.get_possible_transitions(pick_scene_2, action=place_action2):
                                pick_actions3 = list(pick.get_possible_actions_level_1(place_scene2))
                                for pick_action3 in pick_actions3:
                                    for pick_scene_3 in pick.get_possible_transitions(place_scene2, action=pick_action3):
                                        for place_action3 in list(place.get_possible_actions_level_1(pick_scene_3)):
                                            for place_scene3 in place.get_possible_transitions(pick_scene_3, action=place_action3):
                                                fig, ax = p_utils.init_3d_figure( name="all possible transitions")
                                                place.scene_mngr.render_gripper(ax, place_scene3, alpha=0.9, only_visible_axis=False)
                                                place.scene_mngr.render_objects(ax, place_scene3)
                                                place_scene3.show_logical_states()
                                                place.scene_mngr.show()