import numpy as np
import sys
import matplotlib.pyplot as plt
from pykin.utils import plot_utils as p_utils
from pytamp.benchmark import Benchmark1
from pytamp.action.place import PlaceAction

# method = "bai_perturb"
method = "uct"
# method = "random"
budgets = 100
if len(sys.argv) > 1:
    method = sys.argv[1]

seed = 40
end_num = 2
filename = f"benchmark1_trajectory/final_trajectory/success/uct/benchmark1_test_algo({method})_budget({budgets})_seed({seed})_{end_num}.npy"
benchmark1 = Benchmark1(
    robot_name="doosan", geom="collision", is_pyplot=True, box_num=6
)
place = PlaceAction(
    benchmark1.scene_mngr, n_samples_held_obj=0, n_samples_support_obj=0
)

with open(filename, "rb") as f:
    data_for_seed = np.load(f, allow_pickle=True)
    pnp_all_joint_path = data_for_seed["pnp_all_joint_paths"]
    pick_all_objects = data_for_seed["pick_all_objects"]
    place_all_object_poses = data_for_seed["place_all_object_poses"]


place.simulate_path(pnp_all_joint_path, pick_all_objects, place_all_object_poses)
