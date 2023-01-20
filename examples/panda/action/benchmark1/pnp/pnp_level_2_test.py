from pytamp.benchmark import Benchmark1
from pytamp.action.pick import PickAction
from pytamp.action.place import PlaceAction

benchmark1 = Benchmark1(robot_name="panda", geom="visual", is_pyplot=True, box_num=3)
pick = PickAction(benchmark1.scene_mngr, n_contacts=0, n_directions=0)
place = PlaceAction(
    benchmark1.scene_mngr, n_samples_held_obj=0, n_samples_support_obj=0
)
pick_actions = list(pick.get_possible_actions_level_1())

pnp_all_joint_path = []
pick_all_objects = []
place_all_object_poses = []

pnp_path = []
pick_objects = []
place_object_poses = []

success_joint_path = False
cnt = 0
for pick_action in pick_actions:
    for pick_scene in pick.get_possible_transitions(
        pick.scene_mngr.init_scene, action=pick_action
    ):
        pick_joint_path = pick.get_possible_joint_path_level_2(
            scene=pick_scene, grasp_poses=pick_scene.grasp_poses
        )
        if pick_joint_path:
            place_actions = list(place.get_possible_actions_level_1(pick_scene))
            for place_action in place_actions:
                for place_scene in place.get_possible_transitions(
                    scene=pick_scene, action=place_action
                ):
                    place_joint_path = place.get_possible_joint_path_level_2(
                        scene=place_scene,
                        release_poses=place_scene.release_poses,
                        init_thetas=pick_joint_path[-1][
                            place.move_data.MOVE_default_grasp
                        ][-1],
                    )
                    if place_joint_path:
                        success_joint_path = True
                        pnp_path += pick_joint_path + place_joint_path
                        pick_objects.append(pick_scene.robot.gripper.attached_obj_name)
                        place_object_poses.append(
                            place_scene.objs[place_scene.pick_obj_name].h_mat
                        )
                        break
                if success_joint_path:
                    break
        if success_joint_path:
            break
    if success_joint_path:
        break
pnp_all_joint_path.append(pnp_path)
pick_all_objects.append(pick_objects)
place_all_object_poses.append(place_object_poses)

place.simulate_path(pnp_all_joint_path, pick_all_objects, place_all_object_poses)
