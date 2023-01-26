import numpy as np
import os

from pykin.utils import plot_utils as p_utils
from pytamp.benchmark import Benchmark3
from pytamp.search.planner import Planner

algo = "bai_ucb"
budgets = 100
directory_name = "bench3_planner"
p_utils.createDirectory(directory_name)

for seed in range(24, 50):
    # for seed in range(10, 13):
    benchmark3 = Benchmark3(robot_name="doosan", geom="visual", is_pyplot=True)

    # bai_ucb num 2
    planner = [
        ("pick", "milk1"),
        ("place", "table"),
        ("pick", "square_box2"),
        ("place", "table"),
        ("pick", "milk3"),
        ("place", "table"),
        ("pick", "rect_box2"),
        ("place", "table"),
        ("pick", "square_box1"),
        ("place", "table"),
        ("pick", "milk2"),
        ("place", "table"),
        ("pick", "goal_can"),
        ("place", "tray_blue"),
    ]

    print(
        f"Benchmark: {benchmark3.scene_mngr.scene.bench_num}, Algo: {algo}, Seed: {seed}"
    )
    np.random.seed(seed)
    try:
        planner = Planner(benchmark3.scene_mngr)
        (
            pnp_all_joint_path,
            pick_all_objects,
            place_all_object_poses,
        ) = planner.do_planning(planner)

        num = 2
        filename = (
            directory_name
            + "/benchmark3_test_algo({:})_budget({:})_seed({:})_{}.npy".format(
                algo, budgets, seed, num
            )
        )
        # video_name = directory_name + '/benchmark3_test_algo({:})_budget({:})_seed({:})_{}'.format(algo, budgets, seed, num)
        # plan.place_action.simulate_path(pnp_all_joint_path, pick_all_objects, place_all_object_poses, is_save=True, video_name=video_name)
        while os.path.exists(filename):
            filename = (
                directory_name
                + "/benchmark3_test_algo({:})_budget({:})_seed({:})_{}.npy".format(
                    algo, budgets, seed, num
                )
            )
            num += 1
        with open(filename, "wb") as f:
            np.savez(
                f,
                benchmark_number=benchmark3.scene_mngr.scene.bench_num,
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
