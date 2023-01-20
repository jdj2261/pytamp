import numpy as np
import sys
from pytamp.benchmark import Benchmark1
from pytamp.search.mcts import MCTS


def get_final_all_values(algo, budgets, n_seed, end_num=0):
    result_all_values = {}

    perturb_final_level_1_values = []
    perturb_final_level_2_values = []
    perturb_final_pnp_all_joint_paths = []
    perturb_final_pick_all_objects = []
    perturb_final_place_all_object_poses = []
    c_list = []
    for seed in range(1, n_seed + 1):
        filename = f"benchmark1_result/benchmark1_test_algo({algo})_budget({budgets})_seed({seed})_{end_num}.npy"
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
            c_list = data_for_seed["c"]

    perturb_final_level_1_values = np.asarray(perturb_final_level_1_values)
    perturb_final_level_2_values = np.asarray(perturb_final_level_2_values)
    perturb_final_level_2_values[np.isinf(perturb_final_level_2_values)] = 0.0

    result_all_values["level_1_values"] = perturb_final_level_1_values
    result_all_values["level_2_values"] = perturb_final_level_2_values
    result_all_values["pnp_all_joint_paths"] = perturb_final_pnp_all_joint_paths
    result_all_values["pick_all_objects"] = perturb_final_pick_all_objects
    result_all_values["place_all_object_poses"] = perturb_final_place_all_object_poses
    result_all_values["c"] = c_list

    return result_all_values


def get_mean_std_values(result_all_values, level=1):
    if level == 1:
        key_name = "level_1_values"
    if level == 2:
        key_name = "level_2_values"
    mean_values = np.mean(result_all_values[key_name], axis=0)
    # mean_values = np.ma.masked_invalid(result_all_values[key_name]).mean(axis=0)
    std_values = np.std(result_all_values[key_name], axis=0)
    return mean_values, std_values


method = "bai_perturb"
if len(sys.argv) > 1:
    method = sys.argv[1]

bai_perturb_result_all_values = get_final_all_values(method, 100, 5, 0)
perturb_mean_level_1_values, perturb_std_level_1_values = get_mean_std_values(
    bai_perturb_result_all_values, level=1
)
perturb_mean_level_2_values, perturb_std_level_2_values = get_mean_std_values(
    bai_perturb_result_all_values, level=2
)

max_perturb_idx = np.argmax(perturb_mean_level_2_values[:, -1])
perturb_level_2_mean_list = perturb_mean_level_2_values[max_perturb_idx]
perturb_level_2_std_list = perturb_std_level_2_values[max_perturb_idx]

c = bai_perturb_result_all_values["c"][max_perturb_idx]
print(c)

np.random.seed(1)

benchmark1 = Benchmark1(
    robot_name="doosan", geom="collision", is_pyplot=True, box_num=6
)
mcts = MCTS(
    scene_mngr=benchmark1.scene_mngr,
    sampling_method=method,
    budgets=100,
    max_depth=16,
    c=c,
)

for i in range(100):
    mcts.do_planning(i)
