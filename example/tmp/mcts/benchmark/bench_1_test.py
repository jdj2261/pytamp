import numpy as np
import argparse
import matplotlib.pyplot as plt

from pytamp.benchmark import Benchmark1
from pytamp.search.mcts import MCTS


#? python3 bench_1_test.py --budgets 1 --max_depth 1 --seed 3 --algo bai_ucb
parser = argparse.ArgumentParser(description='Test Benchmark 1.')
parser.add_argument('--budgets', metavar='T', type=int, default=5000, help='Horizon')
parser.add_argument('--max_depth', metavar='H', type=int, default=20, help='Max depth')
parser.add_argument('--seed', metavar='i', type=int, default=1, help='A random seed')
parser.add_argument('--algo', metavar='alg', type=str, default='bai_perturb', choices=['bai_perturb', 'bai_ucb', 'uct'], help='Sampler Name')
parser.add_argument('--debug_mode', metavar='debug', type=bool, default=False, help='Debug mode')
parser.add_argument('--benchmark', metavar='N', type=int, default=1, help='Benchmark Number')
args = parser.parse_args()

debug_mode = args.debug_mode
budgets = args.budgets
max_depth = args.max_depth
algo = args.algo
seed = args.seed
np.random.seed(seed)

benchmark1 = Benchmark1(geom="collision", is_pyplot=True, box_num=3)

mcts_list = []
c_list = 10**np.linspace(0., 4., 3)
for c in c_list:
    mcts = MCTS(benchmark1.scene_mngr, sampling_method=algo, budgets=budgets, max_depth=max_depth, c=c)
    for i in range(budgets):
        mcts.do_planning(i)
    

# for mcts in mcts_list:
#     mcts:MCTS = mcts
#     rewards = mcts.rewards
#     max_iter = np.argmax(rewards)
#     print(max_iter, mcts.c)
#     # plt.plot(rewards)
#     # plt.show()
#     subtree = mcts.get_subtree()
#     mcts.visualize_tree("MCTS", subtree)
#     best_nodes = mcts.get_best_node(subtree)
#     # for node in best_nodes:
#         # mcts.show_logical_action(node)
#     pnp_all_joint_path, pick_all_objects, place_all_object_poses = mcts.get_all_joint_path(best_nodes)
#     # mcts.simulate_path(pnp_all_joint_path, pick_all_objects, place_all_object_poses)
#     # mcts_list(mcts.do_planning())

#     # subtree = mcts.get_subtree()
#     # mcts.visualize_tree("MCTS", subtree)

#     # best_nodes = mcts.get_best_node(subtree)

#     # rewards = mcts.rewards
#     # max_iter = np.argmax(rewards)
#     # print(max_iter)
    
#     # if debug_mode:
#     #     plt.plot(rewards)
#     #     plt.show()

#     # # Do planning
#     # best_nodes = mcts.get_best_node(subtree)
#     # pnp_all_joint_path, pick_all_objects, place_all_object_poses = mcts.get_all_joint_path(best_nodes)

#     # if debug_mode:
#     #     mcts.simulate_path(pnp_all_joint_path, pick_all_objects, place_all_object_poses)