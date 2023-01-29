from pytamp.benchmark import Benchmark1
from pykin.utils import plot_utils as p_utils
from pytamp.action.pick import PickAction
from pytamp.action.place import PlaceAction
import numpy as np 

benchmark1 = Benchmark1(robot_name="doosan", geom="visual", is_pyplot=False, box_num=3)
pick = PickAction(benchmark1.scene_mngr, n_contacts=15, n_directions=3)
place = PlaceAction(
    benchmark1.scene_mngr, n_samples_held_obj=10, n_samples_support_obj=0, n_directions=5
)

################# Action Test ##################
fig, ax = p_utils.init_3d_figure(name="Level wise 1")
pick_action = pick.get_action_level_1_for_single_object(
    pick.scene_mngr.init_scene, "A_box"
)

c =0 
for grasp_pose in pick_action[pick.info.GRASP_POSES]:
    pick.scene_mngr.render_axis(ax, grasp_pose[pick.move_data.MOVE_grasp])

for pick_scene in pick.get_possible_transitions(
    pick.scene_mngr.init_scene, pick_action
):

    place_action = place.get_action_level_1_for_single_object(
        "tray_red", "A_box", pick_scene.robot.gripper.grasp_pose, scene=pick_scene
    )

    for release_pose, obj_pose in place_action[place.info.RELEASE_POSES]:
        
        # print(release_pose[place.move_data.MOVE_release])
        # print(obj_pose[2,2])
    
        place.scene_mngr.render_axis(ax, pick_scene.robot.gripper.grasp_pose)
        place.scene_mngr.render.render_obj_axis(ax, pick_scene.robot.gripper.pick_obj_pose)

        place.scene_mngr.render.render_axis(
            ax,release_pose[place.move_data.MOVE_release]
        )
        place.scene_mngr.render.render_object(
            ax, place.scene_mngr.scene.objs["A_box"], obj_pose
        )
        place.scene_mngr.render.render_obj_axis(ax, obj_pose)   
        c += 1

print("number of get pnp consider only collision. (not consider inverse kinematics", c)

place.scene_mngr.render_objects(ax)
pick.scene_mngr.render_objects(ax)
pick.show()
p_utils.plot_basis(ax)
place.show()
