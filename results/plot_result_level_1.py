import numpy as np
import sys, os, datetime
import matplotlib.pyplot as plt
from pykin.utils import plot_utils as p_utils
from pytamp.benchmark import Benchmark4
from pytamp.action.place import PlaceAction

bench_num = 1
if len(sys.argv) > 1:
    bench_num = int(sys.argv[1])


def linearplot_with_confidence(
    x_list, mean_list, std_list, label, marker="", color="r"
):
    a = 0.1
    plt.fill_between(
        x_list,
        mean_list - a * std_list,
        mean_list + a * std_list,
        alpha=0.13,
        color=color,
    )
    plt.plot(
        x_list, mean_list, label=label, marker=marker, ms=11, c=color, markevery=15
    )


def get_final_all_values(algo, budgets, start_seed, end_seed, end_num=0):
    result_all_values = {}

    perturb_final_level_1_values = []
    perturb_final_level_2_values = []
    perturb_final_pnp_all_joint_paths = []
    perturb_final_pick_all_objects = []
    perturb_final_place_all_object_poses = []
    c_list = []

    for seed in range(start_seed, end_seed + 1):
        if bench_num == 1:
            filename = f"benchmark1/benchmark1_result/benchmark1_test_algo({algo})_budget({budgets})_seed({seed})_{end_num}.npy"
        if bench_num == 2:
            filename = f"benchmark2/benchmark2_result/benchmark2_test_algo({algo})_budget({budgets})_seed({seed})_{end_num}.npy"
        if bench_num == 3:
            filename = f"benchmark3/benchmark3_result/benchmark3_test_algo({algo})_budget({budgets})_seed({seed})_{end_num}.npy"
        if bench_num == 4:
            filename = f"benchmark4/benchmark4_result/benchmark4_test_algo({algo})_budget({budgets})_seed({seed})_{end_num}.npy"
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


budgets = 100
start_seed = 1
end_seed = 10
if bench_num == 1 or bench_num == 3:
    end_seed = 5
end_num = 0

bai_perturb_result_all_values = get_final_all_values(
    "bai_perturb", budgets, start_seed, end_seed, end_num
)
perturb_mean_level_1_values, perturb_std_level_1_values = get_mean_std_values(
    bai_perturb_result_all_values, level=1
)
perturb_mean_level_2_values, perturb_std_level_2_values = get_mean_std_values(
    bai_perturb_result_all_values, level=2
)

uct_result_all_values = get_final_all_values(
    "uct", budgets, start_seed, end_seed, end_num
)
uct_mean_level_1_values, uct_std_level_1_values = get_mean_std_values(
    uct_result_all_values, level=1
)
uct_mean_level_2_values, uct_std_level_2_values = get_mean_std_values(
    uct_result_all_values, level=2
)

random_result_all_values = get_final_all_values(
    "random", budgets, start_seed, end_seed, end_num
)
random_mean_level_1_values, random_std_level_1_values = get_mean_std_values(
    random_result_all_values, level=1
)
random_mean_level_2_values, random_std_level_2_values = get_mean_std_values(
    random_result_all_values, level=2
)

bai_ucb_result_all_values = get_final_all_values(
    "bai_ucb", budgets, start_seed, end_seed, end_num
)
bai_ucb_mean_level_1_values, bai_ucb_std_level_1_values = get_mean_std_values(
    bai_ucb_result_all_values, level=1
)
bai_ucb_mean_level_2_values, bai_ucb_std_level_2_values = get_mean_std_values(
    bai_ucb_result_all_values, level=2
)

max_perturb_idx = np.argmax(perturb_mean_level_1_values[:, -1])
perturb_level_1_mean_list = perturb_mean_level_1_values[max_perturb_idx]
perturb_level_1_std_list = perturb_std_level_1_values[max_perturb_idx]

max_uct_idx = np.argmax(uct_mean_level_1_values[:, -1])
uct_level_1_mean_list = uct_mean_level_1_values[max_uct_idx]
uct_level_1_std_list = uct_std_level_1_values[max_uct_idx]

max_bai_ucb_idx = np.argmax(bai_ucb_mean_level_1_values[:, -1])
bai_ucb_level_1_mean_list = bai_ucb_mean_level_1_values[max_bai_ucb_idx]
bai_ucb_level_1_std_list = bai_ucb_std_level_1_values[max_bai_ucb_idx]

max_random_idx = np.argmax(random_mean_level_1_values[:, -1])
random_level_1_mean_list = random_mean_level_1_values[max_random_idx]
random_level_1_std_list = random_std_level_1_values[max_random_idx]

x_list = np.arange(len(perturb_level_1_mean_list))

if bench_num == 1:
    filename = "benchmark1"
if bench_num == 2:
    filename = "benchmark2"
if bench_num == 3:
    filename = "benchmark3"
if bench_num == 4:
    filename = "benchmark4"

plt.figure(figsize=(8, 6))

linearplot_with_confidence(
    x_list[:-1],
    perturb_level_1_mean_list[:-1],
    perturb_level_1_std_list[:-1],
    "PBAI",
    ">",
    "m",
)
linearplot_with_confidence(
    x_list[:-1], uct_level_1_mean_list[:-1], uct_level_1_std_list[:-1], "UCT", "v", "c"
)
linearplot_with_confidence(
    x_list[:-1],
    bai_ucb_level_1_mean_list[:-1],
    bai_ucb_level_1_std_list[:-1],
    "BAI-UCB",
    "o",
    "g",
)
linearplot_with_confidence(
    x_list[:-1],
    random_level_1_mean_list[:-1],
    random_level_1_std_list[:-1],
    "Random",
    "^",
    "r",
)

leg = plt.legend(prop={"size": 20})
leg.get_frame().set_linewidth(0.0)

plt.xlabel("Number of Trials", fontsize=20)
plt.ylabel("Task Reward", fontsize=20)

plt.xticks(x_list[::20], x_list[::20], fontsize=16)
plt.yticks(fontsize=16)

save_dir_name = "result_images"
p_utils.createDirectory(save_dir_name)

file_name = save_dir_name + "/" + filename + "_{}.png".format("level_1")
plt.savefig(file_name)
plt.show()
