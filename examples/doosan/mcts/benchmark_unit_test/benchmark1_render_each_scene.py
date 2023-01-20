import numpy as np
import argparse

from pykin.utils import plot_utils as p_utils
from pytamp.benchmark import Benchmark, Benchmark1
from pytamp.search.mcts import MCTS


benchmark = Benchmark(robot_name="doosan", geom="visual", is_pyplot=True)
fig, ax = p_utils.init_3d_figure(name="Benchmark")
benchmark.scene_mngr.show()

# ? python3 benchmark1_test.py --budgets 1 --max_depth 1 --seed 3 --algo bai_ucb
parser = argparse.ArgumentParser(description="Test Benchmark 1.")
parser.add_argument("--budgets", metavar="T", type=int, default=300, help="Horizon")
parser.add_argument("--max_depth", metavar="H", type=int, default=18, help="Max depth")
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
algo = args.algo
seed = args.seed
np.random.seed(seed)

benchmark1 = Benchmark1(robot_name="doosan", geom="visual", is_pyplot=False, box_num=6)

c_list = 10 ** np.linspace(-2, 1.0, 10)
# c_list = [1, 10, 100, 1000]
for idx, c in enumerate(c_list):
    mcts = MCTS(benchmark1.scene_mngr)
    mcts.debug_mode = False
    mcts.only_optimize_1 = True

    # 최대부터
    mcts.budgets = 100
    mcts.max_depth = 16
    mcts.sampling_method = "bai_perturb"
    mcts.c = c
    print(c)
    for i in range(mcts.budgets):
        mcts.do_planning(i)
        if mcts.level_wise_1_success:
            sub_nodes = mcts.get_nodes_from_leaf_node(mcts.success_level_1_leaf_node)[
                ::-1
            ]
            for n in sub_nodes:
                if mcts.tree.nodes[n][mcts.node_data.TYPE] == "state":
                    action = mcts.tree.nodes[n].get(mcts.node_data.ACTION)
                    state = mcts.tree.nodes[n][mcts.node_data.STATE]
                    if action:
                        if list(action.keys())[0] == "grasp":
                            print("pick")
                            mcts.render_state("Success", state, close_gripper=True)
                        if list(action.keys())[0] == "release":
                            print("place")
                            mcts.render_state("Success", state, close_gripper=False)
        mcts.level_wise_1_success = False
