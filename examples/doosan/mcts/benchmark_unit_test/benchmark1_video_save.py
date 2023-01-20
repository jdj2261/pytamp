import numpy as np
import argparse
import os

from pykin.utils import plot_utils as p_utils
from pytamp.benchmark import Benchmark, Benchmark1
from pytamp.search.mcts import MCTS


# ? python3 benchmark1_test.py --budgets 1 --max_depth 1 --seed 3 --algo bai_ucb
parser = argparse.ArgumentParser(description="Test Benchmark 1.")
parser.add_argument("--budgets", metavar="T", type=int, default=100, help="Horizon")
parser.add_argument("--max_depth", metavar="H", type=int, default=16, help="Max depth")
parser.add_argument("--seed", metavar="i", type=int, default=1, help="A random seed")
parser.add_argument(
    "--algo",
    metavar="alg",
    type=str,
    default="uct",
    choices=["bai_perturb", "bai_ucb", "uct", "random"],
    help="Sampler Name",
)
parser.add_argument(
    "--debug_mode", metavar="debug", type=bool, default=False, help="Debug mode"
)
parser.add_argument(
    "--benchmark", metavar="N", type=int, default=1, help="Benchmark Number"
)
args = parser.parse_args()

debug_mode = args.debug_mode
budgets = args.budgets
max_depth = args.max_depth
algo = "bai_perturb"
seed = args.seed
np.random.seed(seed)

benchmark1 = Benchmark1(robot_name="doosan", geom="visual", is_pyplot=False, box_num=6)

c_list = 10 ** np.linspace(-2, 1.0, 10)
# c_list = [1, 10, 100, 1000]
for idx, c in enumerate(c_list):
    mcts = MCTS(
        scene_mngr=benchmark1.scene_mngr,
        sampling_method=algo,
        budgets=budgets,
        max_depth=max_depth,
        c=c,
        debug_mode=debug_mode,
    )
    mcts.debug_mode = False
    mcts.only_optimize_1 = False

    print(mcts.budgets)
    for i in range(mcts.budgets):
        print(
            f"\n[{idx+1}/{len(c_list)}] Benchmark: {benchmark1.scene_mngr.scene.bench_num}, Algo: {algo}, C: {c}, Seed: {seed}"
        )
        mcts.do_planning(i)

        #### File Save ####
        if mcts.level_wise_2_success and not mcts.has_aleardy_level_1_optimal_nodes:
            print("Create Trajectory video file")
            directory_name = "./benchmark1_result"
            directory_name = os.path.abspath(directory_name)
            p_utils.createDirectory(directory_name)

            num = 0
            file_name = (
                directory_name
                + "/benchmark1_test_algo({:})_budget({:})_seed({:})_iter({:})_{}.mp4".format(
                    algo, budgets, seed, i, num
                )
            )
            while os.path.exists(file_name):
                file_name = (
                    directory_name
                    + "/benchmark1_test_algo({:})_budget({:})_seed({:})_iter({:})_{}.mp4".format(
                        algo, budgets, seed, i, num
                    )
                )
                num += 1

            print(f"File Name : {file_name}")
            mcts.get_all_joint_path(mcts.optimal_nodes)
            (
                pnp_all_joint_path,
                pick_all_objects,
                place_all_object_poses,
            ) = mcts.get_all_joint_path(mcts.optimal_nodes)
            mcts.show_logical_actions(mcts.optimal_nodes)
            mcts.place_action.simulate_path(
                pnp_all_joint_path,
                pick_all_objects,
                place_all_object_poses,
                is_save=True,
                video_name=file_name,
            )
