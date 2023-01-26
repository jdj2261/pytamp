import numpy as np
import argparse
import os

from pykin.utils import plot_utils as p_utils
from pytamp.benchmark import Benchmark, Benchmark2
from pytamp.search.mcts import MCTS

# benchmark = Benchmark(robot_name="doosan", geom="visual", is_pyplot=True)
# fig, ax = p_utils.init_3d_figure(name="Benchmark")
# benchmark.scene_mngr.show()

# #? python3 benchmark2_test.py --budgets 1 --max_depth 1 --seed 3 --algo bai_ucb
parser = argparse.ArgumentParser(description="Test Benchmark 2.")
parser.add_argument("--budgets", metavar="T", type=int, default=100, help="Horizon")
parser.add_argument("--max_depth", metavar="H", type=int, default=20, help="Max depth")
parser.add_argument("--seed", metavar="i", type=int, default=1, help="A random seed")
parser.add_argument(
    "--algo",
    metavar="alg",
    type=str,
    default="bai_perturb",
    choices=["bai_perturb", "bai_ucb", "uct"],
    help="Sampler Name",
)
parser.add_argument(
    "--debug_mode", metavar="debug", type=bool, default=False, help="Debug mode"
)
parser.add_argument(
    "--benchmark", metavar="N", type=int, default=2, help="Benchmark Number"
)
args = parser.parse_args()

debug_mode = args.debug_mode
budgets = args.budgets
max_depth = args.max_depth
algo = args.algo
seed = args.seed
seeds = 10
np.random.seed(seed)

print("Create Trajectory video file")
directory_name = "./benchmark2_result"
directory_name = os.path.abspath(directory_name)
p_utils.createDirectory(directory_name)


for seed in range(1, 20):
    np.random.seed(seed)
    c_list = 10 ** np.linspace(-1, 2.0, 10)
    for seed in range(20, 25):
        for idx, c in enumerate(c_list):
            benchmark2 = Benchmark2(
                robot_name="doosan", geom="visual", bottle_num=6, is_pyplot=False
            )
            mcts = MCTS(benchmark2.scene_mngr)
            mcts.debug_mode = False
            mcts.only_optimize_1 = False

            # 최대부터
            mcts.budgets = 100
            mcts.max_depth = 20
            mcts.sampling_method = "random"
            mcts.c = c

            for i in range(mcts.budgets):
                print(
                    f"\n[{idx+1}/{len(c_list)}] Benchmark: {benchmark2.scene_mngr.scene.bench_num}, Algo: {algo}, C: {c}, Seed: {seed}"
                )
                mcts.do_planning(i)

                #### File Save ####
                if (
                    mcts.level_wise_2_success
                    and not mcts.has_aleardy_level_1_optimal_nodes
                ):
                    try:
                        (
                            pnp_all_joint_path,
                            pick_all_objects,
                            place_all_object_poses,
                        ) = mcts.get_all_joint_path(mcts.optimal_nodes)
                        mcts.show_logical_actions(mcts.optimal_nodes)
                        num = 2
                        filename = (
                            directory_name
                            + "/benchmark2_test_algo({:})_budget({:})_seed({:})_{}.npy".format(
                                algo, budgets, seed, num
                            )
                        )
                        # video_name = directory_name + '/benchmark2_test_algo({:})_budget({:})_seed({:})_{}'.format(algo, budgets, seed, num)
                        # plan.place_action.simulate_path(pnp_all_joint_path, pick_all_objects, place_all_object_poses, is_save=True, video_name=video_name)
                        while os.path.exists(filename):
                            filename = (
                                directory_name
                                + "/benchmark2_test_algo({:})_budget({:})_seed({:})_{}.npy".format(
                                    algo, budgets, seed, num
                                )
                            )
                            num += 1
                        with open(filename, "wb") as f:
                            np.savez(
                                f,
                                benchmark_number=benchmark2.scene_mngr.scene.bench_num,
                                algo=algo,
                                seed=seed,
                                pnp_all_joint_paths=pnp_all_joint_path,
                                pick_all_objects=pick_all_objects,
                                place_all_object_poses=place_all_object_poses,
                            )
                        print("Data saved at {}".format(filename))
                        break
                    except:
                        print("=" * 20)
                        print("Not found path")
                        print("=" * 20)
                        continue
