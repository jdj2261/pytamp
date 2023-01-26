import numpy as np
import matplotlib.pyplot as plt
from pykin.utils import plot_utils as p_utils
from pytamp.benchmark import Benchmark2
from pytamp.action.place import PlaceAction


def get_final_all_values(algo, budgets, n_seed, end_num=0):
    result_all_values = {}

    perturb_final_level_1_values = []
    perturb_final_level_2_values = []
    perturb_final_pnp_all_joint_paths = []
    perturb_final_pick_all_objects = []
    perturb_final_place_all_object_poses = []
    for seed in range(1, n_seed + 1):
        filename = f"benchmark2_result/benchmark2_test_algo({algo})_budget({budgets})_seed({seed})_{end_num}.npy"
        with open(filename, "rb") as f:
            data_for_seed = np.load(f, allow_pickle=True)
            perturb_final_level_1_values.append(data_for_seed["level_1_values"])
            perturb_final_level_2_values.append(data_for_seed["level_2_values"])
            perturb_final_pnp_all_joint_paths.append(
                data_for_seed["pnp_all_joint_paths"]
            )
            perturb_final_pick_all_objects.append(data_for_seed["pick_all_objects"])
            perturb_final_place_all_object_poses.append(
                data_for_seed["place_all_object_poses"]
            )

    perturb_final_level_1_values = np.asarray(perturb_final_level_1_values)
    perturb_final_level_2_values = np.asarray(perturb_final_level_2_values)
    perturb_final_level_2_values[np.isinf(perturb_final_level_2_values)] = 0.0

    result_all_values["level_1_values"] = perturb_final_level_1_values
    result_all_values["level_2_values"] = perturb_final_level_2_values
    result_all_values["pnp_all_joint_paths"] = perturb_final_pnp_all_joint_paths
    result_all_values["pick_all_objects"] = perturb_final_pick_all_objects
    result_all_values["place_all_object_poses"] = perturb_final_place_all_object_poses

    return result_all_values


bai_perturb_result_all_values = get_final_all_values("bai_perturb", 100, 10, 0)
idx = np.unravel_index(
    bai_perturb_result_all_values["level_2_values"].argmax(),
    bai_perturb_result_all_values["level_2_values"].shape,
)
pnp_all_joint_path = bai_perturb_result_all_values["pnp_all_joint_paths"][idx[0]][
    idx[1]
]
pick_all_objects = bai_perturb_result_all_values["pick_all_objects"][idx[0]][idx[1]]
place_all_object_poses = bai_perturb_result_all_values["place_all_object_poses"][
    idx[0]
][idx[1]]

benchmark2 = Benchmark2(
    robot_name="doosan", geom="collision", is_pyplot=True, bottle_num=6
)
place = PlaceAction(
    benchmark2.scene_mngr, n_samples_held_obj=0, n_samples_support_obj=0
)

place.simulate_path(pnp_all_joint_path, pick_all_objects, place_all_object_poses)
