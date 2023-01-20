import numpy as np
import sys
import matplotlib.pyplot as plt
from pykin.utils import plot_utils as p_utils
from pytamp.benchmark import Benchmark2
from pytamp.action.place import PlaceAction

method = "bai_perturb"
# method = "uct"
# method = "bai_ucb"
# method = "random"
budgets = 300
if len(sys.argv) > 1:
    method = sys.argv[1]

seed = 17
end_num = 2
filename = f"./bench2_planner/benchmark2_test_algo({method})_budget({budgets})_seed({seed})_{end_num}.npy"

benchmark2 = Benchmark2(
    robot_name="doosan", geom="collision", is_pyplot=True, bottle_num=6
)
place = PlaceAction(
    benchmark2.scene_mngr, n_samples_held_obj=0, n_samples_support_obj=0
)

with open(filename, "rb") as f:
    data_for_seed = np.load(f, allow_pickle=True)
    pnp_all_joint_path = data_for_seed["pnp_all_joint_paths"]
    pick_all_objects = data_for_seed["pick_all_objects"]
    place_all_object_poses = data_for_seed["place_all_object_poses"]

# place.simulate_path(pnp_all_joint_path, pick_all_objects, place_all_object_poses, is_save=True, fps=30)
place.simulate_path(pnp_all_joint_path, pick_all_objects, place_all_object_poses)
