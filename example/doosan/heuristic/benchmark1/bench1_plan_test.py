import numpy as np
import os

from pykin.utils import plot_utils as p_utils
from pytamp.benchmark import Benchmark1
from pytamp.search.planner import Planner

algo = "bai_ucb"
budgets = 100
directory_name = "bench1_planner"
p_utils.createDirectory(directory_name)

for seed in range(10, 14):
    # for seed in range(10, 13):
    benchmark1 = Benchmark1(
        robot_name="doosan", geom="visual", is_pyplot=True, box_num=6
    )

    # bai_ucb num 2
    planner = [
        ("pick", "F_box"),
        ("place", "tray_red"),
        ("pick", "E_box"),
        ("place", "table"),
        ("pick", "F_box"),
        ("place", "E_box"),
        ("pick", "A_box"),
        ("place", "tray_red"),
        ("pick", "B_box"),
        ("place", "A_box"),
        ("pick", "D_box"),
        ("place", "table"),
        ("pick", "C_box"),
        ("place", "B_box"),
        ("pick", "D_box"),
        ("place", "C_box"),
        ("pick", "F_box"),
        ("place", "table"),
        ("pick", "E_box"),
        ("place", "D_box"),
    ]

    print(
        f"Benchmark: {benchmark1.scene_mngr.scene.bench_num}, Algo: {algo}, Seed: {seed}"
    )
    np.random.seed(seed)
    try:
        planner = Planner(benchmark1.scene_mngr)
        (
            pnp_all_joint_path,
            pick_all_objects,
            place_all_object_poses,
        ) = planner.do_planning(planner)

        num = 2
        filename = (
            directory_name
            + "/benchmark1_test_algo({:})_budget({:})_seed({:})_{}.npy".format(
                algo, budgets, seed, num
            )
        )
        # video_name = directory_name + '/benchmark1_test_algo({:})_budget({:})_seed({:})_{}'.format(algo, budgets, seed, num)
        # plan.place_action.simulate_path(pnp_all_joint_path, pick_all_objects, place_all_object_poses, is_save=True, video_name=video_name)
        while os.path.exists(filename):
            filename = (
                directory_name
                + "/benchmark1_test_algo({:})_budget({:})_seed({:})_{}.npy".format(
                    algo, budgets, seed, num
                )
            )
            num += 1
        with open(filename, "wb") as f:
            np.savez(
                f,
                benchmark_number=benchmark1.scene_mngr.scene.bench_num,
                algo=algo,
                seed=seed,
                pnp_all_joint_paths=pnp_all_joint_path,
                pick_all_objects=pick_all_objects,
                place_all_object_poses=place_all_object_poses,
            )
        print("Data saved at {}".format(filename))
    except:
        print("=" * 20)
        print("Not found path")
        print("=" * 20)
        continue


# Episode 1
# uct num 2
# planner = [
#     ("pick", "A_box"),
#     ("place", "tray_red"),
#     ("pick", "F_box"),
#     ("place", "table"),
#     ("pick", "E_box"),
#     ("place", "table"),
#     ("pick", "B_box"),
#     ("place", "A_box"),
#     ("pick", "C_box"),
#     ("place", "B_box"),
#     ("pick", "D_box"),
#     ("place", "C_box"),
#     ("pick", "E_box"),
#     ("place", "D_box"),
#     ("pick", "F_box"),
#     ("place", "E_box"),
# ]


# TODO [Change]
# bai_ucb num 0
# planner = [
#     ("pick", "E_box"),
#     ("place", "table"),
#     ("pick", "F_box"),
#     ("place", "table"),
#     ("pick", "A_box"),
#     ("place", "tray_red"),
#     ("pick", "E_box"),
#     ("place", "table"),
#     ("pick", "D_box"),
#     ("place", "table"),
#     ("pick", "B_box"),
#     ("place", "A_box"),
#     ("pick", "C_box"),
#     ("place", "B_box"),
#     ("pick", "D_box"),
#     ("place", "C_box"),
#     ("pick", "E_box"),
#     ("place", "D_box"),
#     ("pick", "F_box"),
#     ("place", "E_box")
# ]

# bai_ucb num 1
# planner = [
#     ("pick", "A_box"),
#     ("place", "tray_red"),
#     ("pick", "B_box"),
#     ("place", "A_box"),
#     ("pick", "C_box"),
#     ("place", "B_box"),
#     ("pick", "F_box"),
#     ("place", "table"),
#     ("pick", "E_box"),
#     ("place", "table"),
#     ("pick", "D_box"),
#     ("place", "C_box"),
#     ("pick", "E_box"),
#     ("place", "D_box"),
#     ("pick", "F_box"),
#     ("place", "E_box"),
# ]

# # bai_ucb num 2
# planner = [
#     ("pick", "F_box"),
#     ("place", "table"),
#     ("pick", "E_box"),
#     ("place", "table"),
#     ("pick", "A_box"),
#     ("place", "tray_red"),
#     ("pick", "B_box"),
#     ("place", "A_box"),
#     ("pick", "C_box"),
#     ("place", "B_box"),
#     ("pick", "D_box"),
#     ("place", "C_box"),
#     ("pick", "E_box"),
#     ("place", "D_box"),
#     ("pick", "F_box"),
#     ("place", "E_box"),
# ]

# # random num 0
# planner = [
#     ("pick", "F_box"),
#     ("place", "table"),
#     ("pick", "A_box"),
#     ("place", "tray_red"),
#     ("pick", "B_box"),
#     ("place", "A_box"),
#     ("pick", "E_box"),
#     ("place", "C_box"),
#     ("pick", "B_box"),
#     ("place", "table"),
#     ("pick", "A_box"),
#     ("place", "table"),
#     ("pick", "F_box"),
#     ("place", "E_box"),
#     ("pick", "B_box"),
#     ("place", "A_box"),
# ]

# # random num 1
# planner = [
#     ("pick", "A_box"),
#     ("place", "tray_red"),
#     ("pick", "F_box"),
#     ("place", "B_box"),
#     ("pick", "E_box"),
#     ("place", "A_box"),
#     ("pick", "D_box"),
#     ("place", "C_box"),
#     ("pick", "E_box"),
#     ("place", "table"),
#     ("pick", "F_box"),
#     ("place", "E_box"),
#     ("pick", "B_box"),
#     ("place", "A_box"),
#     ("pick", "D_box"),
#     ("place", "B_box"),
# ]

# # random num 2
# planner = [
#     ("pick", "C_box"),
#     ("place", "tray_red"),
#     ("pick", "F_box"),
#     ("place", "B_box"),
#     ("pick", "E_box"),
#     ("place", "table"),
#     ("pick", "D_box"),
#     ("place", "C_box"),
#     ("pick", "F_box"),
#     ("place", "table"),
#     ("pick", "B_box"),
#     ("place", "A_box"),
#     ("pick", "E_box"),
#     ("place", "D_box"),
#     ("pick", "F_box"),
#     ("place", "E_box"),
# ]

# Episode 2

# # perturb num 0
# planner = [
#     ("pick", "F_box"),
#     ("place", "table"),
#     ("pick", "A_box"),
#     ("place", "tray_red"),
#     ("pick", "E_box"),
#     ("place", "table"),
#     ("pick", "B_box"),
#     ("place", "A_box"),
#     ("pick", "D_box"),
#     ("place", "table"),
#     ("pick", "C_box"),
#     ("place", "B_box"),
#     ("pick", "D_box"),
#     ("place", "C_box"),
#     ("pick", "E_box"),
#     ("place", "D_box"),
#     ("pick", "F_box"),
#     ("place", "E_box"),
# ]

# # perturb num 1
# planner = [
#     ("pick", "F_box"),
#     ("place", "table"),
#     ("pick", "A_box"),
#     ("place", "tray_red"),
#     ("pick", "E_box"),
#     ("place", "table"),
#     ("pick", "B_box"),
#     ("place", "A_box"),
#     ("pick", "D_box"),
#     ("place", "table"),
#     ("pick", "C_box"),
#     ("place", "B_box"),
#     ("pick", "D_box"),
#     ("place", "C_box"),
#     ("pick", "E_box"),
#     ("place", "D_box"),
#     ("pick", "F_box"),
#     ("place", "E_box"),
# ]

# # perturb num 2
# planner = [
#     ("pick", "F_box"),
#     ("place", "table"),
#     ("pick", "E_box"),
#     ("place", "table"),
#     ("pick", "A_box"),
#     ("place", "tray_red"),
#     ("pick", "B_box"),
#     ("place", "A_box"),
#     ("pick", "D_box"),
#     ("place", "table"),
#     ("pick", "C_box"),
#     ("place", "B_box"),
#     ("pick", "D_box"),
#     ("place", "C_box"),
#     ("pick", "E_box"),
#     ("place", "D_box"),
#     ("pick", "F_box"),
#     ("place", "E_box"),
# ]

# # uct num 0
# planner = [
#     ("pick", "F_box"),
#     ("place", "table"),
#     ("pick", "A_box"),
#     ("place", "tray_red"),
#     ("pick", "E_box"),
#     ("place", "table"),
#     ("pick", "B_box"),
#     ("place", "A_box"),
#     ("pick", "D_box"),
#     ("place", "table"),
#     ("pick", "C_box"),
#     ("place", "B_box"),
#     ("pick", "D_box"),
#     ("place", "C_box"),
#     ("pick", "E_box"),
#     ("place", "D_box"),
#     ("pick", "F_box"),
#     ("place", "E_box"),
# ]

# # uct num 1
# planner = [
#     ("pick", "F_box"),
#     ("place", "table"),
#     ("pick", "E_box"),
#     ("place", "table"),
#     ("pick", "D_box"),
#     ("place", "table"),
#     ("pick", "A_box"),
#     ("place", "tray_red"),
#     ("pick", "B_box"),
#     ("place", "A_box"),
#     ("pick", "C_box"),
#     ("place", "B_box"),
#     ("pick", "D_box"),
#     ("place", "C_box"),
#     ("pick", "E_box"),
#     ("place", "D_box"),
#     ("pick", "F_box"),
#     ("place", "E_box"),
# ]

# # uct num 2
# planner = [
#     ("pick", "F_box"),
#     ("place", "table"),
#     ("pick", "E_box"),
#     ("place", "table"),
#     ("pick", "A_box"),
#     ("place", "tray_red"),
#     ("pick", "D_box"),
#     ("place", "table"),
#     ("pick", "B_box"),
#     ("place", "A_box"),
#     ("pick", "C_box"),
#     ("place", "B_box"),
#     ("pick", "D_box"),
#     ("place", "C_box"),
#     ("pick", "E_box"),
#     ("place", "D_box"),
#     ("pick", "F_box"),
#     ("place", "E_box"),
# ]

# # bai_ucb num 0
# planner = [
#     ("pick", "F_box"),
#     ("place", "table"),
#     ("pick", "A_box"),
#     ("place", "tray_red"),
#     ("pick", "E_box"),
#     ("place", "table"),
#     ("pick", "B_box"),
#     ("place", "A_box"),
#     ("pick", "D_box"),
#     ("place", "table"),
#     ("pick", "C_box"),
#     ("place", "B_box"),
#     ("pick", "D_box"),
#     ("place", "C_box"),
#     ("pick", "E_box"),
#     ("place", "D_box"),
#     ("pick", "F_box"),
#     ("place", "E_box"),
# ]

# # bai_ucb num 1
# planner = [
#     ("pick", "F_box"),
#     ("place", "table"),
#     ("pick", "A_box"),
#     ("place", "tray_red"),
#     ("pick", "E_box"),
#     ("place", "A_box"),
#     ("pick", "D_box"),
#     ("place", "table"),
#     ("pick", "E_box"),
#     ("place", "table"),
#     ("pick", "B_box"),
#     ("place", "A_box"),
#     ("pick", "C_box"),
#     ("place", "B_box"),
#     ("pick", "D_box"),
#     ("place", "C_box"),
#     ("pick", "E_box"),
#     ("place", "D_box"),
#     ("pick", "F_box"),
#     ("place", "E_box"),
# ]

# # bai_ucb num 2
# planner = [
#     ("pick", "F_box"),
#     ("place", "tray_red"),
#     ("pick", "E_box"),
#     ("place", "table"),
#     ("pick", "F_box"),
#     ("place", "E_box"),
#     ("pick", "A_box"),
#     ("place", "tray_red"),
#     ("pick", "B_box"),
#     ("place", "A_box"),
#     ("pick", "D_box"),
#     ("place", "table"),
#     ("pick", "C_box"),
#     ("place", "B_box"),
#     ("pick", "D_box"),
#     ("place", "C_box"),
#     ("pick", "F_box"),
#     ("place", "table"),
#     ("pick", "E_box"),
#     ("place", "D_box"),
# ]


# # random num 0
# planner = [
#     ("pick", "D_box"),
#     ("place", "tray_red"),
#     ("pick", "E_box"),
#     ("place", "D_box"),
#     ("pick", "F_box"),
#     ("place", "table"),
#     ("pick", "B_box"),
#     ("place", "A_box"),
#     ("pick", "F_box"),
#     ("place", "B_box"),
#     ("pick", "E_box"),
#     ("place", "C_box"),
#     ("pick", "F_box"),
#     ("place", "table"),
#     ("pick", "E_box"),
#     ("place", "D_box"),
#     ("pick", "C_box"),
#     ("place", "B_box"),
# ]

# # random num 1
# planner = [
#     ("pick", "F_box"),
#     ("place", "table"),
#     ("pick", "D_box"),
#     ("place", "table"),
#     ("pick", "E_box"),
#     ("place", "D_box"),
#     ("pick", "F_box"),
#     ("place", "E_box"),
#     ("pick", "A_box"),
#     ("place", "tray_red"),
#     ("pick", "B_box"),
#     ("place", "A_box"),
#     ("pick", "C_box"),
#     ("place", "B_box"),
#     ("pick", "F_box"),
#     ("place", "C_box"),
# ]

# # random num 2
# planner = [
#     ("pick", "E_box"),
#     ("place", "tray_red"),
#     ("pick", "F_box"),
#     ("place", "table"),
#     ("pick", "D_box"),
#     ("place", "table"),
#     ("pick", "B_box"),
#     ("place", "A_box"),
#     ("pick", "C_box"),
#     ("place", "B_box"),
#     ("pick", "E_box"),
#     ("place", "D_box"),
#     ("pick", "F_box"),
#     ("place", "E_box"),
#     ("pick", "C_box"),
#     ("place", "tray_red"),
#     ("pick", "F_box"),
#     ("place", "C_box"),
# ]
