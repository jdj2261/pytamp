import numpy as np
import argparse

from pykin.utils import plot_utils as p_utils
from pytamp.benchmark import Benchmark4
from pytamp.search.mcts import MCTS


# #? python3 benchmark4_test.py --budgets 1 --max_depth 1 --seed 3 --algo bai_ucb
parser = argparse.ArgumentParser(description='Test Benchmark 4.')
parser.add_argument('--budgets', metavar='T', type=int, default=300, help='Horizon')
parser.add_argument('--max_depth', metavar='H', type=int, default=20, help='Max depth')
parser.add_argument('--seed', metavar='i', type=int, default=1, help='A random seed')
parser.add_argument('--algo', metavar='alg', type=str, default='bai_perturb', choices=['bai_perturb', 'bai_ucb', 'uct'], help='Sampler Name')
parser.add_argument('--debug_mode', metavar='debug', type=bool, default=False, help='Debug mode')
parser.add_argument('--benchmark', metavar='N', type=int, default=2, help='Benchmark Number')
parser.add_argument('--disk_number', metavar='N', type=int, default=3, help='Disk Number(5 or less.)')
args = parser.parse_args()

debug_mode = args.debug_mode
budgets = args.budgets
max_depth = args.max_depth
algo = args.algo
seed = args.seed
number = args.disk_number
np.random.seed(seed)

benchmark4 = Benchmark4(robot_name="doosan", geom="collision", disk_num=number)
# c_list = 10**np.linspace(-2, 2., 5)
c_list = 10**np.linspace(-2, 2., 10)
seeds = 10
for seed in range(6, seeds+1):
    for idx, c in enumerate(c_list):
        mcts = MCTS(benchmark4.scene_mngr)
        mcts.debug_mode = False 
        mcts.only_optimize_1 = True

        # 최대부터
        mcts.budgets = 100
        mcts.max_depth = 14
        mcts.sampling_method = 'bai_perturb'
        # mcts.sampling_method = 'uct'
        # mcts.sampling_method = 'bai_ucb'
        # mcts.sampling_method = 'random'
        mcts.c = c
        print(c)
        for i in range(mcts.budgets):
            mcts.do_planning(i)
            # if mcts.level_wise_2_success:
            #     pnp_all_joint_path, pick_all_objects, place_all_object_poses = mcts.get_all_joint_path(mcts.optimal_nodes)
            #     mcts.show_logical_actions(mcts.optimal_nodes)
            #     mcts.place_action.simulate_path(pnp_all_joint_path, pick_all_objects, place_all_object_poses)
        # subtree = mcts.get_success_subtree(optimizer_level=1)
        # mcts.visualize_tree("MCTS", subtree)
        # best_nodes = mcts.get_best_node(subtree)

        level_1_max_value = mcts.values_for_level_1
        max_iter = np.argmax(level_1_max_value)

        level_1_max_values = mcts.values_for_level_1
        level_2_max_values = mcts.values_for_level_2
        fig, ax = p_utils.init_2d_figure("test")

        p_utils.plot_values(
            ax,
            level_1_max_values, 
            label=f"Sum of Values({mcts.sampling_method}, {mcts.budgets}, {mcts.c})", 
            title="Benchamrk2_Level_1_" + mcts.sampling_method + "-" + str(mcts.budgets)+ "-" + str(seed) + "-" + str(mcts.c),
            save_dir_name='benchmark4_result', 
            is_save=True)
        
        del mcts
        # p_utils.plot_values(
        #     ax,
        #     level_2_max_values, 
        #     label="Optiaml Values", 
        #     title="Benchamrk1_Level_2_" + mcts.sampling_method,  
        #     save_dir_name='benchmark4_result', 
        #     is_save=True)
        # p_utils.show_figure()

        # # Do planning
        # # mcts.get_all_joint_path(mcts.optimal_nodes)
        # pnp_all_joint_path, pick_all_objects, place_all_object_poses = mcts.get_all_joint_path(mcts.optimal_nodes)
        # mcts.show_logical_actions(mcts.optimal_nodes)
        # mcts.place_action.simulate_path(pnp_all_joint_path, pick_all_objects, place_all_object_poses)