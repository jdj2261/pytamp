import numpy as np
import argparse
import os

from pykin.utils import plot_utils as p_utils
from pytamp.benchmark import Benchmark1
from pytamp.search.mcts import MCTS


# ? python3 benchmark1_test.py --budgets 1000 --max_depth 20 --seed 3 --algo bai_ucb
parser = argparse.ArgumentParser(description="Test Benchmark 1.")
parser.add_argument("--budgets", metavar="T", type=int, default=100, help="Horizon")
parser.add_argument("--max_depth", metavar="H", type=int, default=20, help="Max depth")
parser.add_argument("--seed", metavar="i", type=int, default=2, help="A random seed")
parser.add_argument(
    "--algo",
    metavar="alg",
    type=str,
    default="bai_perturb",
    choices=["bai_perturb", "bai_ucb", "uct", "random", "greedy"],
    help="Choose one (bai_perturb, bai_ucb, uct)",
)
parser.add_argument(
    "--debug_mode",
    default=False,
    type=lambda x: (str(x).lower() == "true"),
    help="Debug mode",
)
parser.add_argument(
    "--box_number", metavar="N", type=int, default=6, help="Box Number(6 or less)"
)
args = parser.parse_args()

debug_mode = args.debug_mode
budgets = args.budgets

algo = args.algo

algo = "bai_perturb"
max_depth = 16

# algo = "uct"
# max_depth = 18

# algo = "bai_ucb"
# max_depth = 20

# algo = "random"
seed = args.seed
number = args.box_number
# np.random.seed(seed)

directory_name = "benchmark1_trajectory"
p_utils.createDirectory(directory_name)

benchmark1 = Benchmark1(
    robot_name="doosan", geom="collision", is_pyplot=True, box_num=number
)


c_list = 10 ** np.linspace(-1, 2.0, 10)
# c_list = [4.64]
cnt = 0
n_seed = 300
for seed in range(40, n_seed + 1):
    np.random.seed(seed)
    for idx, c in enumerate(c_list):
        mcts = MCTS(
            scene_mngr=benchmark1.scene_mngr,
            sampling_method=algo,
            budgets=budgets,
            max_depth=max_depth,
            c=c,
            debug_mode=debug_mode,
        )
        mcts.only_optimize_1 = False
        for i in range(budgets):
            print(
                f"\n[{idx+1}/{len(c_list)}] Benchmark: {benchmark1.scene_mngr.scene.bench_num}, Algo: {algo}, C: {c}, Seed: {seed}"
            )
            mcts.do_planning(i)

            # if mcts.level_wise_1_success:
            #     sub_nodes = mcts.get_nodes_from_leaf_node(mcts.success_level_1_leaf_node)[::-1]
            #     for n in sub_nodes:
            #         if mcts.tree.nodes[n][mcts.node_data.TYPE] == "state":
            #             action = mcts.tree.nodes[n].get(mcts.node_data.ACTION)
            #             state = mcts.tree.nodes[n][mcts.node_data.STATE]
            #             if action:
            #                 if list(action.keys())[0] == 'grasp':
            #                     print("pick")
            #                     mcts.render_state("Success", state, close_gripper=True)
            #                 if list(action.keys())[0] == 'release':
            #                     print("place")
            #                     mcts.render_state("Success", state, close_gripper=False)
            #     break
            # # mcts.level_wise_1_success = False

            if mcts.level_wise_2_success:
                break

        if mcts.level_wise_2_success:
            (
                pnp_all_joint_paths,
                pick_all_objects,
                place_all_object_poses,
            ) = mcts.get_all_joint_path(mcts.optimal_nodes)
            # mcts.place_action.simulate_path(pnp_all_joint_paths, pick_all_objects, place_all_object_poses)
            num = 0
            filename = (
                directory_name
                + "/benchmark1_test_algo({:})_budget({:})_seed({:})_{}.npy".format(
                    algo, budgets, seed, num
                )
            )

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
                    budgets=budgets,
                    algo=algo,
                    c=c_list,
                    seed=seed,
                    pnp_all_joint_paths=pnp_all_joint_paths,
                    pick_all_objects=pick_all_objects,
                    place_all_object_poses=place_all_object_poses,
                )
            print("Data saved at {}".format(filename))
