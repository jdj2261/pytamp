import numpy as np
import sys
from sympy import det
import trimesh, io

from PIL import Image
from pykin.utils import plot_utils as p_utils
from pytamp.benchmark import Benchmark3
from pytamp.action.place import PlaceAction
from pykin.utils.kin_utils import (
    apply_robot_to_scene,
    apply_gripper_to_scene,
    apply_objects_to_scene,
)

directory_name = "bench3_scene/random"
p_utils.createDirectory(directory_name)

# method = "bai_perturb"
# method = "uct"
method = "bai_ucb"
# method = "random"
budgets = 100
if len(sys.argv) > 1:
    method = sys.argv[1]

seed = 24
end_num = 2
filename = f"./bench3_planner/benchmark3_test_algo({method})_budget({budgets})_seed({seed})_{end_num}.npy"
benchmark3 = Benchmark3(robot_name="doosan", geom="visual", is_pyplot=False)
place = PlaceAction(
    benchmark3.scene_mngr, n_samples_held_obj=0, n_samples_support_obj=0
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
    attach_count = 0
    detach_count = 0
    is_attach = False
    is_detach = False
    for pnp_joint_path in pnp_joint_all_path:
        for _, (task, joint_path) in enumerate(pnp_joint_path.items()):
            for _, joint in enumerate(joint_path):
                idx += 1
                trimesh_scene = trimesh.Scene()
                place.scene_mngr.set_robot_eef_pose(joint)

                if task == place.move_data.MOVE_grasp:
                    grasp_task_idx = idx
                if task == place.move_data.MOVE_post_grasp:
                    post_grasp_task_idx = idx
                if post_grasp_task_idx - grasp_task_idx == 1:
                    attach_idx = grasp_task_idx
                    attach_count += 1
                    is_attach = True
                    place.scene_mngr.attach_object_on_gripper(
                        pick_all_object[attach_count - 1], False
                    )
                    # place.scene_mngr.close_gripper(0.05)

                if task == place.move_data.MOVE_release:
                    release_task_idx = idx
                if task == place.move_data.MOVE_post_release:
                    post_release_task_idx = idx
                if post_release_task_idx - release_task_idx == 1:
                    detach_idx = release_task_idx
                    detach_count += 1
                    is_attach = False
                    place.scene_mngr.detach_object_from_gripper(
                        pick_all_object[detach_count - 1]
                    )
                    place.scene_mngr.add_object(
                        name=pick_all_object[detach_count - 1],
                        gtype=place.scene_mngr.init_objects[
                            pick_all_object[detach_count - 1]
                        ].gtype,
                        gparam=place.scene_mngr.init_objects[
                            pick_all_object[detach_count - 1]
                        ].gparam,
                        h_mat=place_all_object_pose[detach_count - 1],
                        color=place.scene_mngr.init_objects[
                            pick_all_object[detach_count - 1]
                        ].color,
                    )
                    # place.scene_mngr.open_gripper(0.05)

                if is_attach == True:
                    place.scene_mngr.close_gripper(0.01)
                    if "milk" in pick_all_object[attach_count - 1]:
                        place.scene_mngr.close_gripper(0.04)
                    if "can" in pick_all_object[attach_count - 1]:
                        place.scene_mngr.close_gripper(0.03)

                    trimesh_scene = apply_robot_to_scene(
                        trimesh_scene=trimesh_scene,
                        robot=place.scene_mngr.scene.robot,
                        geom="visual",
                    )
                    trimesh_scene = apply_gripper_to_scene(
                        trimesh_scene=trimesh_scene,
                        robot=place.scene_mngr.scene.robot,
                        geom="visual",
                    )
                else:
                    trimesh_scene = apply_robot_to_scene(
                        trimesh_scene=trimesh_scene,
                        robot=place.scene_mngr.scene.robot,
                        geom="visual",
                    )
                trimesh_scene = apply_objects_to_scene(
                    trimesh_scene=trimesh_scene, objs=place.scene_mngr.scene.objs
                )
                trimesh_scene.set_camera(
                    np.array([-np.pi / 2 - np.pi / 3, np.pi, -np.pi / 6]),
                    2.5,
                    resolution=(480, 320),
                )
                # data = trimesh_scene.save_image(visible=False)
                data = trimesh_scene.save_image(resolution=[480, 320], visible=True)

                data_io = io.BytesIO(data)
                # img = Image.open(data_io)
                im = Image.open(data_io)
                file_name = directory_name + "/bench3_" + str(idx) + ".png"
                im.save(file_name)
