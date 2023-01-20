import numpy as np
import sys
import trimesh, io

from PIL import Image
from pykin.utils import plot_utils as p_utils
from pytamp.benchmark import Benchmark1
from pytamp.action.place import PlaceAction
from pykin.utils.kin_utils import apply_robot_to_scene

directory_name = "bench1_scene"
p_utils.createDirectory(directory_name)

# method = "perturb"
method = "uct"
# method = "bai_ucb"
# method = "random"
budgets = 100
if len(sys.argv) > 1:
    method = sys.argv[1]

seed = 1
end_num = 0
# filename = f'./bench1_planner/benchmark1_test_algo({method})_budget({budgets})_seed({seed})_{end_num}.npy'
filename = f"./bench1_planner/Episode2/uct/benchmark1_test_algo({method})_budget({budgets})_seed({seed})_{end_num}.npy"
benchmark1 = Benchmark1(robot_name="doosan", geom="visual", is_pyplot=False, box_num=6)
place = PlaceAction(
    benchmark1.scene_mngr, n_samples_held_obj=0, n_samples_support_obj=0
)

with open(filename, "rb") as f:
    data_for_seed = np.load(f, allow_pickle=True)
    pnp_all_joint_path = data_for_seed["pnp_all_joint_paths"]
    pick_all_objects = data_for_seed["pick_all_objects"]
    place_all_object_poses = data_for_seed["place_all_object_poses"]

for pnp_joint_all_path, pick_all_object, place_all_object_pose in zip(
    pnp_all_joint_path, pick_all_objects, place_all_object_poses
):
    result_joint = []
    attach_idxes = []
    detach_idxes = []
    attach_idx = 0
    detach_idx = 0
    grasp_task_idx = 0
    post_grasp_task_idx = 0
    release_task_idx = 0
    post_release_task_idx = 0
    idx = 0

    for pnp_joint_path in pnp_joint_all_path:
        for _, (task, joint_path) in enumerate(pnp_joint_path.items()):
            for _, joint in enumerate(joint_path):
                idx += 1

                if task == place.move_data.MOVE_grasp:
                    grasp_task_idx = idx
                if task == place.move_data.MOVE_post_grasp:
                    post_grasp_task_idx = idx
                if post_grasp_task_idx - grasp_task_idx == 1:
                    attach_idx = grasp_task_idx
                    attach_idxes.append(attach_idx)

                if task == place.move_data.MOVE_release:
                    release_task_idx = idx
                if task == place.move_data.MOVE_post_release:
                    post_release_task_idx = idx
                if post_release_task_idx - release_task_idx == 1:
                    detach_idx = release_task_idx
                    detach_idxes.append(detach_idx)

                result_joint.append(joint)

                trimesh_scene = trimesh.Scene()

                place.scene_mngr.set_robot_eef_pose(joint)
                trimesh_scene = apply_robot_to_scene(
                    trimesh_scene=trimesh_scene,
                    robot=place.scene_mngr.scene.robot,
                    geom="visual",
                )
                trimesh_scene.set_camera(
                    np.array([np.pi / 2, 0, np.pi / 2]), 5, resolution=(640, 512)
                )
                # trimesh_scene.show()
                # self.render_scene()
                # data = trimesh_scene.save_image(visible=False)
                data = trimesh_scene.save_image(resolution=[640, 480], visible=True)

                data_io = io.BytesIO(data)
                # img = Image.open(data_io)
                im = Image.open(data_io)
                file_name = directory_name + "/test" + str(idx) + ".png"
                im.save(file_name)
