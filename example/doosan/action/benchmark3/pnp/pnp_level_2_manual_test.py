from pykin.utils import plot_utils as p_utils
from pytamp.action.pick import PickAction
from pytamp.action.place import PlaceAction
from pytamp.benchmark import Benchmark3

benchmark3 = Benchmark3(robot_name="doosan", geom="collision", is_pyplot=True)
pick = PickAction(
    benchmark3.scene_mngr, n_contacts=0, n_directions=0, retreat_distance=0.1
)
place = PlaceAction(
    benchmark3.scene_mngr,
    n_samples_held_obj=0,
    n_samples_support_obj=10,
    retreat_distance=0.2,
)

pnp_all_joint_path = []
pick_all_objects = []
place_all_object_poses = []

pnp_path = []
pick_objects = []
place_object_poses = []

for pick_obj in ["rect_top_left_box"]:
    pick_action = pick.get_action_level_1_for_single_object(
        pick.scene_mngr.scene, pick_obj
    )
    success_joint_path = False

    for pick_scene in pick.get_possible_transitions(pick.scene_mngr.scene, pick_action):
        pick_joint_path = pick.get_possible_joint_path_level_2(
            scene=pick_scene, grasp_poses=pick_scene.grasp_poses
        )
        if pick_joint_path:
            place_action = place.get_action_level_1_for_single_object(
                "table", pick_obj, pick_scene.robot.gripper.grasp_pose, scene=pick_scene
            )
            for place_scene in place.get_possible_transitions(
                scene=pick_scene, action=place_action
            ):
                place_joint_path = place.get_possible_joint_path_level_2(
                    scene=place_scene,
                    release_poses=place_scene.release_poses,
                    init_thetas=pick_joint_path[-1][place.move_data.MOVE_default_grasp][
                        -1
                    ],
                )
                if place_joint_path:
                    success_joint_path = True
                    pnp_path = pick_joint_path + place_joint_path
                    pick_objects.append(pick_scene.robot.gripper.attached_obj_name)
                    place_object_poses.append(
                        place_scene.objs[place_scene.pick_obj_name].h_mat
                    )
                    break
        if success_joint_path:
            break

    pnp_all_joint_path.append(pnp_path)
    pick_all_objects.append(pick_objects)
    place_all_object_poses.append(place_object_poses)

    if pnp_path:
        place.simulate_path(
            pnp_all_joint_path, pick_all_objects, place_all_object_poses
        )
