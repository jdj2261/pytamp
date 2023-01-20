import numpy as np

from pykin.utils import plot_utils as p_utils
from pykin.kinematics.transform import Transform
from pykin.robots.single_arm import SingleArm
from pykin.utils.mesh_utils import get_object_mesh, get_mesh_bounds

from pytamp.action.pick import PickAction
from pytamp.action.place import PlaceAction
from pytamp.scene.scene_manager import SceneManager

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

shelf_pose = Transform(pos=np.array([0.6, -0.2, 1.3]), rot=np.array([0, 0, np.pi / 2]))
shelf_9_mesh = get_object_mesh("shelf_9.stl", scale=0.9)
shelf_9_mesh_bound = get_mesh_bounds(mesh=shelf_9_mesh)


table_mesh = get_object_mesh("custom_table.stl", 0.01)

scene_mngr = SceneManager("collision", is_pyplot=True)
scene_mngr.scene.bench_num = 2
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
    name="shelf_9",
    gtype="mesh",
    gparam=shelf_9_mesh,
    h_mat=shelf_pose.h_mat,
    color=[1.0, 0, 1.0],
)
scene_mngr.add_robot(robot, robot.init_qpos)

scene_mngr.scene.logical_states["shelf_9"] = {
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

pick = PickAction(scene_mngr, n_contacts=0, n_directions=1, retreat_distance=0.01)
place = PlaceAction(scene_mngr, n_samples_held_obj=0, n_samples_support_obj=10)

################# Action Test ##################

fig, ax = p_utils.init_3d_figure(name="Level wise 1")
pick_action = pick.get_action_level_1_for_single_object(scene_mngr.scene, "bottle1")

for grasp_pose in pick_action[pick.info.GRASP_POSES]:
    pick.scene_mngr.render_axis(ax, grasp_pose[pick.move_data.MOVE_grasp])
    pick.scene_mngr.render_axis(ax, grasp_pose[pick.move_data.MOVE_pre_grasp])
    pick.scene_mngr.render_axis(ax, grasp_pose[pick.move_data.MOVE_post_grasp])

for pick_scene in pick.get_possible_transitions(scene_mngr.scene, pick_action):
    place_action = place.get_action_level_1_for_single_object(
        "shelf_9", "bottle1", pick_scene.robot.gripper.grasp_pose, scene=pick_scene
    )
    for release_pose, obj_pose in place_action[place.info.RELEASE_POSES]:
        place.scene_mngr.render.render_axis(
            ax, release_pose[place.move_data.MOVE_release]
        )
        place.scene_mngr.render.render_axis(
            ax, release_pose[place.move_data.MOVE_pre_release]
        )
        place.scene_mngr.render.render_axis(
            ax, release_pose[place.move_data.MOVE_post_release]
        )
        place.scene_mngr.render.render_object(
            ax, place.scene_mngr.scene.objs["bottle1"], obj_pose
        )
place.scene_mngr.render_objects(ax, alpha=0.3)
p_utils.plot_basis(ax)
place.show()
