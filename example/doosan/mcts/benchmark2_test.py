import numpy as np
import argparse
import matplotlib.pyplot as plt

from pykin.utils import plot_utils as p_utils
from pytamp.benchmark import Benchmark2
from pytamp.search.mcts import MCTS


#? python3 bench_1_test.py --budgets 1 --max_depth 1 --seed 3 --algo bai_ucb
parser = argparse.ArgumentParser(description='Test Benchmark 1.')
parser.add_argument('--budgets', metavar='T', type=int, default=300, help='Horizon')
parser.add_argument('--max_depth', metavar='H', type=int, default=20, help='Max depth')
parser.add_argument('--seed', metavar='i', type=int, default=1, help='A random seed')
parser.add_argument('--algo', metavar='alg', type=str, default='bai_perturb', choices=['bai_perturb', 'bai_ucb', 'uct'], help='Sampler Name')
parser.add_argument('--debug_mode', metavar='debug', type=bool, default=False, help='Debug mode')
parser.add_argument('--benchmark', metavar='N', type=int, default=2, help='Benchmark Number')
args = parser.parse_args()

debug_mode = args.debug_mode
budgets = args.budgets
max_depth = args.max_depth
algo = args.algo
seed = args.seed
# np.random.seed(seed)

benchmark2 = Benchmark2(robot_name="doosan", geom="collision", is_pyplot=True, bottle_num=3)
mcts = MCTS(benchmark2.scene_mngr)
fig, ax = p_utils.init_3d_figure(name="Benchmark 3")
mcts.scene_mngr.render_scene(ax)
mcts.scene_mngr.show()

mcts.debug_mode = False

# 최대부터
mcts.budgets = 300
mcts.max_depth = 20
mcts.c = 30
# mcts.sampling_method = 'bai_ucb' # 405
mcts.sampling_method = 'bai_perturb' # 58
# mcts.sampling_method = 'uct' # 369

for i in range(mcts.budgets):
    mcts.do_planning(i)

subtree = mcts.get_success_subtree()
mcts.visualize_tree("MCTS", subtree)

best_nodes = mcts.get_best_node(subtree)
if best_nodes:
    print("\nBest Action Node")
    for node in best_nodes:
        mcts.show_logical_action(node)

rewards = mcts.rewards_for_level_1
max_iter = np.argmax(rewards)
print(max_iter)
plt.plot(rewards)
plt.show()

# Do planning
best_nodes = mcts.get_best_node(subtree)
pnp_all_joint_path, pick_all_objects, place_all_object_poses = mcts.get_all_joint_path(best_nodes)
mcts.place_action.simulate_path(pnp_all_joint_path, pick_all_objects, place_all_object_poses)