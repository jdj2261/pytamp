import numpy as np

from pykin.utils import plot_utils as p_utils
from pykin.kinematics.transform import Transform
from pykin.robots.single_arm import SingleArm
from pykin.utils.mesh_utils import get_object_mesh
from pytamp.action.pick import PickAction
from pytamp.scene.scene_manager import SceneManager

file_path = "urdf/doosan/doosan_with_robotiq140.urdf"
robot = SingleArm(
    f_name=file_path,
    offset=Transform(rot=[0.0, 0.0, 0.0], pos=[0, 0, 0.913]),
    has_gripper=True,
    gripper_name="robotiq140",
)
robot.setup_link_name("base_0", "right_hand")
robot.init_qpos = np.array([0, 0, np.pi / 1.5, 0, np.pi / 3, 0])

red_box_pose = Transform(pos=np.array([0.6, 0.2, 0.77]))
blue_box_pose = Transform(pos=np.array([0.6, 0.35, 0.77]))
green_box_pose = Transform(pos=np.array([0.6, 0.05, 0.77]))
support_box_pose = Transform(
    pos=np.array([0.6, -0.2, 0.77]), rot=np.array([0, np.pi / 2, 0])
)
table_pose = Transform(pos=np.array([0.4, 0.24, 0.0]))

red_cube_mesh = get_object_mesh("ben_cube.stl", 0.06)
blue_cube_mesh = get_object_mesh("ben_cube.stl", 0.06)
green_cube_mesh = get_object_mesh("ben_cube.stl", 0.06)
goal_box_mesh = get_object_mesh("goal_box.stl", 0.001)
table_mesh = get_object_mesh("custom_table.stl", 0.01)

scene_mngr = SceneManager("visual", is_pyplot=False)
scene_mngr.add_object(
    name="table",
    gtype="mesh",
    gparam=table_mesh,
    h_mat=table_pose.h_mat,
    color=[0.823, 0.71, 0.55],
)
scene_mngr.add_object(
    name="red_box",
    gtype="mesh",
    gparam=red_cube_mesh,
    h_mat=red_box_pose.h_mat,
    color=[1.0, 0.0, 0.0],
)
scene_mngr.add_object(
    name="blue_box",
    gtype="mesh",
    gparam=blue_cube_mesh,
    h_mat=blue_box_pose.h_mat,
    color=[0.0, 0.0, 1.0],
)
scene_mngr.add_object(
    name="green_box",
    gtype="mesh",
    gparam=green_cube_mesh,
    h_mat=green_box_pose.h_mat,
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
scene_mngr.scene.logical_states["red_box"] = {
    scene_mngr.scene.logical_state.on: scene_mngr.scene.objs["table"]
}
scene_mngr.scene.logical_states["blue_box"] = {
    scene_mngr.scene.logical_state.on: scene_mngr.scene.objs["table"]
}
scene_mngr.scene.logical_states["green_box"] = {
    scene_mngr.scene.logical_state.on: scene_mngr.scene.objs["table"]
}
scene_mngr.scene.logical_states["table"] = {scene_mngr.scene.logical_state.static: True}
scene_mngr.scene.logical_states[scene_mngr.gripper_name] = {
    scene_mngr.scene.logical_state.holding: None
}
scene_mngr.update_logical_states()

pick = PickAction(scene_mngr, n_contacts=5, n_directions=10)

################# Action Test ##################
fig, ax = p_utils.init_3d_figure(name="Level wise 1")


for obj in ["red_box", "blue_box", "green_box"]:
    pose = list(pick.get_grasp_pose_from_heuristic(obj_name=obj))
    for i in range(len(pose)):
        pick.scene_mngr.render_axis(ax, pose[i][pick.move_data.MOVE_grasp])
pick.scene_mngr.render_objects(ax)
p_utils.plot_basis(ax)


fig, ax = p_utils.init_3d_figure(name="Level wise 2")
actions = list(pick.get_possible_actions_level_1())
for pick_actions in actions:
    for all_grasp_pose in pick_actions[pick.info.GRASP_POSES]:
        pick.scene_mngr.render_axis(ax, all_grasp_pose[pick.move_data.MOVE_grasp])
        pick.scene_mngr.render_axis(ax, all_grasp_pose[pick.move_data.MOVE_pre_grasp])
        pick.scene_mngr.render_axis(ax, all_grasp_pose[pick.move_data.MOVE_post_grasp])

pick.scene_mngr.render_objects(ax)
p_utils.plot_basis(ax)
pick.show()
