import numpy as np

from pykin.utils import plot_utils as p_utils
from pykin.kinematics.transform import Transform
from pykin.robots.single_arm import SingleArm
from pykin.utils.mesh_utils import get_object_mesh

from pytamp.scene.scene_manager import SceneManager
from pytamp.action.pick import PickAction
from pytamp.action.place import PlaceAction

file_path = "urdf/panda/panda.urdf"
robot = SingleArm(
    f_name=file_path,
    offset=Transform(rot=[0.0, 0.0, 0.0], pos=[0, 0, 0.913]),
    has_gripper=True,
)
robot.setup_link_name("panda_link_0", "right_hand")
robot.init_qpos = np.array(
    [0, np.pi / 16.0, 0.00, -np.pi / 2.0 - np.pi / 3.0, 0.00, np.pi - 0.2, -np.pi / 4]
)


bottle_meshes = []
for i in range(3):
    bottle_meshes.append(get_object_mesh("bottle.stl"))

bottle_pose1 = Transform(
    pos=np.array([0.6, 0.2, 0.74 + abs(bottle_meshes[0].bounds[0][2])])
)
bottle_pose2 = Transform(
    pos=np.array([0.6, 0.35, 0.74 + abs(bottle_meshes[0].bounds[0][2])])
)
bottle_pose3 = Transform(
    pos=np.array([0.6, 0.05, 0.74 + abs(bottle_meshes[0].bounds[0][2])])
)

support_box_pose = Transform(
    pos=np.array([0.6, -0.2, 0.77]), rot=np.array([0, np.pi / 2, 0])
)
table_pose = Transform(pos=np.array([0.4, 0.24, 0.0]))

goal_box_mesh = get_object_mesh("goal_box.stl", 0.001)
table_mesh = get_object_mesh("custom_table.stl", 0.01)

scene_mngr = SceneManager("collision", is_pyplot=True)
scene_mngr.add_object(
    name="table",
    gtype="mesh",
    gparam=table_mesh,
    h_mat=table_pose.h_mat,
    color=[0.823, 0.71, 0.55],
)
scene_mngr.add_object(
    name="bottle1",
    gtype="mesh",
    gparam=bottle_meshes[0],
    h_mat=bottle_pose1.h_mat,
    color=[1.0, 0.0, 0.0],
)
scene_mngr.add_object(
    name="bottle2",
    gtype="mesh",
    gparam=bottle_meshes[1],
    h_mat=bottle_pose2.h_mat,
    color=[0.0, 0.0, 1.0],
)
scene_mngr.add_object(
    name="bottle3",
    gtype="mesh",
    gparam=bottle_meshes[2],
    h_mat=bottle_pose3.h_mat,
    color=[0.0, 1.0, 0.0],
)
scene_mngr.add_object(
    name="goal_box",
    gtype="mesh",
    gparam=goal_box_mesh,
    h_mat=support_box_pose.h_mat,
    color=[1.0, 0, 1.0],
)
scene_mngr.add_robot(robot, robot.init_qpos)

scene_mngr.scene.logical_states["goal_box"] = {
    scene_mngr.scene.logical_state.on: scene_mngr.scene.objs["table"]
}
scene_mngr.scene.logical_states["bottle1"] = {
    scene_mngr.scene.logical_state.on: scene_mngr.scene.objs["table"]
}
scene_mngr.scene.logical_states["bottle2"] = {
    scene_mngr.scene.logical_state.on: scene_mngr.scene.objs["table"]
}
scene_mngr.scene.logical_states["bottle3"] = {
    scene_mngr.scene.logical_state.on: scene_mngr.scene.objs["table"]
}
scene_mngr.scene.logical_states["table"] = {scene_mngr.scene.logical_state.static: True}
scene_mngr.scene.logical_states[scene_mngr.gripper_name] = {
    scene_mngr.scene.logical_state.holding: None
}
scene_mngr.update_logical_states()

pick = PickAction(scene_mngr, n_contacts=1, n_directions=1)
place = PlaceAction(
    scene_mngr, release_distance=0.005, n_samples_held_obj=1, n_samples_support_obj=10
)

pnp_joint_all_pathes = []
place_all_object_poses = []
pick_all_objects = []
success_joint_path = False
# pick
# step 1. action[type] == pick
pick_action = pick.get_action_level_1_for_single_object(scene_mngr.scene, "bottle1")

cnt = 0
for pick_scene in pick.get_possible_transitions(scene_mngr.scene, pick_action):
    pick_joint_path = pick.get_possible_joint_path_level_2(
        scene=pick_scene, grasp_poses=pick_scene.grasp_poses
    )
    if pick_joint_path:
        place_action = place.get_action_level_1_for_single_object(
            "goal_box", "bottle1", pick_scene.robot.gripper.grasp_pose, scene=pick_scene
        )
        for place_scene in place.get_possible_transitions(
            scene=pick_scene, action=place_action
        ):
            ik_solve, release_poses = place.get_possible_ik_solve_level_2(
                scene=place_scene, release_poses=place_scene.release_poses
            )
            if ik_solve:
                place_joint_path = place.get_possible_joint_path_level_2(
                    scene=place_scene,
                    release_poses=release_poses,
                    init_thetas=pick_joint_path[-1][place.move_data.MOVE_default_grasp][
                        -1
                    ],
                )
                if place_joint_path:
                    success_joint_path = True
                    cnt += 1
                    # pick_path = deepcopy(pick_joint_path)
                    pnp_joint_all_pathes.append((pick_joint_path + place_joint_path))
                    pick_all_objects.append(
                        [pick_scene.robot.gripper.attached_obj_name]
                    )
                    place_all_object_poses.append(
                        [place_scene.objs[place_scene.pick_obj_name].h_mat]
                    )

                    if cnt >= 2:
                        break
    if success_joint_path:
        break

# pprint.pprint(pnp_joint_all_path)

# print(pnp_joint_all_pathes)

for pnp_joint_all_path, pick_all_object, place_all_object_pose in zip(
    pnp_joint_all_pathes, pick_all_objects, place_all_object_poses
):
    fig, ax = p_utils.init_3d_figure(name="Level wise 2")
    result_joint = []
    eef_poses = []
    attach_idx_list = []
    detach_idx_list = []

    attach_idx = 0
    detach_idx = 0

    grasp_task_idx = 0
    post_grasp_task_idx = 0

    release_task_idx = 0
    post_release_task_idx = 0
    cnt = 0
    print(pick_all_object)
    for pnp_joint_path in pnp_joint_all_path:
        for j, (task, joint_path) in enumerate(pnp_joint_path.items()):
            for k, joint in enumerate(joint_path):
                cnt += 1

                if task == pick.move_data.MOVE_grasp:
                    grasp_task_idx = cnt
                if task == pick.move_data.MOVE_post_grasp:
                    post_grasp_task_idx = cnt

                if post_grasp_task_idx - grasp_task_idx == 1:
                    attach_idx = grasp_task_idx
                    attach_idx_list.append(attach_idx)

                if task == place.move_data.MOVE_release:
                    release_task_idx = cnt
                if task == place.move_data.MOVE_post_release:
                    post_release_task_idx = cnt
                if post_release_task_idx - release_task_idx == 1:
                    detach_idx = release_task_idx
                    detach_idx_list.append(detach_idx)

                result_joint.append(joint)
                fk = pick.scene_mngr.scene.robot.forward_kin(joint)
                eef_poses.append(fk[place.scene_mngr.scene.robot.eef_name].pos)
    pick.scene_mngr.animation(
        ax,
        fig,
        init_scene=scene_mngr.scene,
        joint_path=result_joint,
        eef_poses=eef_poses,
        visible_gripper=True,
        visible_text=True,
        alpha=1.0,
        interval=50,
        repeat=False,
        pick_object=pick_all_object,
        attach_idx=attach_idx_list,
        detach_idx=detach_idx_list,
        place_obj_pose=place_all_object_pose,
    )
