from pykin.utils import plot_utils as p_utils
from pytamp.action.pick import PickAction
from pytamp.action.place import PlaceAction
from pytamp.benchmark import Benchmark3

benchmark3 = Benchmark3(robot_name="doosan", geom="collision", is_pyplot=True)
pick = PickAction(benchmark3.scene_mngr, n_contacts=0, n_directions=0, retreat_distance=0.1)
place = PlaceAction(benchmark3.scene_mngr, n_samples_held_obj=0, n_samples_support_obj=5, n_directions=10)

pick_actions = list(pick.get_possible_actions_level_1())
init_scene = pick.scene_mngr.init_scene

for pick_action in pick_actions:
    for pick_scene in pick.get_possible_transitions(init_scene, action=pick_action):
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
                                                fig, ax = p_utils.init_3d_figure( name="first pick")
                                                place.scene_mngr.render_gripper(ax, pick_scene, alpha=0.9, only_visible_axis=False)
                                                place.scene_mngr.render_objects(ax, pick_scene)
                                                print(f"Pick1 {pick_scene.pick_obj_name}")
                                                place.scene_mngr.show()
                                                
                                                fig, ax = p_utils.init_3d_figure( name="first place")
                                                place.scene_mngr.render_gripper(ax, place_scene, alpha=0.9, only_visible_axis=False)
                                                place.scene_mngr.render_objects(ax, place_scene)
                                                print(f"Place1 {place_scene.pick_obj_name} on {place_scene.cur_place_obj_name}")
                                                place.scene_mngr.show()

                                                fig, ax = p_utils.init_3d_figure( name="second pick")
                                                place.scene_mngr.render_gripper(ax, pick_scene_2, alpha=0.9, only_visible_axis=False)
                                                place.scene_mngr.render_objects(ax, pick_scene_2)
                                                print(f"Pick2 {pick_scene_2.pick_obj_name}")
                                                place.scene_mngr.show()

                                                fig, ax = p_utils.init_3d_figure( name="second place")
                                                place.scene_mngr.render_gripper(ax, place_scene2, alpha=0.9, only_visible_axis=False)
                                                place.scene_mngr.render_objects(ax, place_scene2)
                                                print(f"Place2 {place_scene2.pick_obj_name} on {place_scene2.cur_place_obj_name}")
                                                place.scene_mngr.show()
                                                
                                                fig, ax = p_utils.init_3d_figure( name="third pick")
                                                place.scene_mngr.render_gripper(ax, pick_scene_3, alpha=0.9, only_visible_axis=False)
                                                place.scene_mngr.render_objects(ax, pick_scene_3)
                                                print(f"Pick3 {pick_scene_3.pick_obj_name}")
                                                place.scene_mngr.show()
                                                
                                                fig, ax = p_utils.init_3d_figure( name="third place")
                                                place.scene_mngr.render_gripper(ax, place_scene3, alpha=0.9, only_visible_axis=False)
                                                place.scene_mngr.render_objects(ax, place_scene3)
                                                print(f"Place3 {place_scene3.pick_obj_name} on {place_scene3.cur_place_obj_name}")
                                                place.scene_mngr.show()