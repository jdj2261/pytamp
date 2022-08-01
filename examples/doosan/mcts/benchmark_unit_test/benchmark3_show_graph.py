import numpy as np
import argparse

from pykin.utils import plot_utils as p_utils
from pytamp.benchmark import Benchmark3
from pytamp.search.mcts import MCTS


# #? python3 benchmark3_test.py --budgets 1000 --max_depth 20 --seed 3 --algo bai_ucb
parser = argparse.ArgumentParser(description='Test Benchmark 3.')
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
np.random.seed(seed)

benchmark3 = Benchmark3(robot_name="doosan", geom="collision")

c_list = 10**np.linspace(0., 4., 3)
for idx, c in enumerate(c_list):
    mcts = MCTS(
        scene_mngr=benchmark3.scene_mngr, 
        sampling_method=algo, 
        budgets=5, 
        max_depth=20, 
        c=c,
        debug_mode=debug_mode)
    for i in range(mcts.budgets):
        print(f"\nBenchmark: {benchmark3.scene_mngr.scene.bench_num}, Algo: {algo}, C: {c}, Seed: {seed}")
        mcts.do_planning(i)

    level_1_max_values = mcts.values_for_level_1
    level_2_max_values = mcts.values_for_level_2

    fig, ax = p_utils.init_2d_figure(f"test_{idx}")
    
    configs = {}
    configs["num"] = benchmark3.scene_mngr.scene.bench_num
    configs["algo"] = args.algo
    configs["c"] = c
    
    p_utils.plot_values(
        ax,
        level_1_max_values, 
        label="Sum of Values", 
        title=f"Benchmark3_Level_1_c_{idx}", 
        save_dir_name='benchmark3_result', 
        is_save=False,
        **configs)
        
    p_utils.plot_values(
        ax,
        level_2_max_values, 
        label="Optiaml Values", 
        title=f"Benchmark3_Level_2_{idx}", 
        save_dir_name='benchmark3_result', 
        is_save=True,
        **configs)
    # p_utils.show_figure()

    # Do planning
    pnp_all_joint_path, pick_all_objects, place_all_object_poses = mcts.get_all_joint_path(mcts.optimal_nodes)
    mcts.show_logical_actions(mcts.optimal_nodes)
    # mcts.place_action.simulate_path(pnp_all_joint_path, pick_all_objects, place_all_object_poses)