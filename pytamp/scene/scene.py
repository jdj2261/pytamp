import pprint
import string
import numpy as np
from collections import OrderedDict
from dataclasses import dataclass

from pykin.robots.single_arm import SingleArm
from pykin.utils.kin_utils import ShellColors as sc


@dataclass
class State:
    on = "on"
    support = "support"
    static = "static"
    held = "held"
    holding = "holding"
    hang = "hang"
    hung = "hung"


class Scene:
    def __init__(self, benchmark: dict):
        self.benchmark_config = None
        if benchmark is not None:
            self.benchmark_config: int = benchmark
            self.bench_num: int = list(self.benchmark_config.keys())[0]
            self.goal_object = "goal_box"

            if self.bench_num == 1:
                self._init_bench_1()

            if self.bench_num == 2:
                self._init_bench_2()

            if self.bench_num == 3:
                self._init_bench_3()

            if self.bench_num == 4:
                self._init_bench_4()

        self.objs: dict = {}
        self.robot: SingleArm = None
        self.logical_states: OrderedDict = OrderedDict()
        self.logical_state: State = State

        self.grasp_poses = None
        self.release_poses = None

        self.pick_obj_name = None
        self.cur_place_obj_name = None
        self.prev_place_obj_name = []
        self.pick_obj_default_pose = None

    def _init_bench_1(self):
        if self.benchmark_config[self.bench_num].get("goal_object"):
            self.goal_object = self.benchmark_config[self.bench_num]["goal_object"]
        self.goal_stacked_num: int = self.benchmark_config[self.bench_num]["stack_num"]
        self.alphabet_list: list = list(string.ascii_uppercase)[: self.goal_stacked_num]
        self.goal_objects: list = [alphabet + "_box" for alphabet in self.alphabet_list]
        self.stacked_box_num = 0
        self.success_stacked_box_num = 0

    def _init_bench_2(self):
        if self.benchmark_config[self.bench_num].get("goal_object"):
            self.goal_object = self.benchmark_config[self.bench_num]["goal_object"]
        self.goal_q = [-1.2417, -1.415, 0.3991, 0.0, -2.05, 0]
        self.ben_2_final_path = []
        self.has_already_final_path = False

    def _init_bench_3(self):
        if self.benchmark_config[self.bench_num].get("goal_object"):
            self.goal_object = self.benchmark_config[self.bench_num]["goal_object"]

    def _init_bench_4(self):
        self.disk_num = 6
        if self.benchmark_config[self.bench_num].get("disk_num"):
            self.disk_num = self.benchmark_config[self.bench_num].get("disk_num")
        self.goal_objects = ["hanoi_disk_" + str(i) for i in range(self.disk_num)]
        self.goal_object = self.goal_objects[0]
        self.pegs = ["peg_1", "peg_2", "peg_3"]
        self.prev_peg_name = None
        self.cur_peg_name = None

    def show_scene_info(self):
        print(f"*" * 23 + f" {sc.OKGREEN}Scene{sc.ENDC} " + f"*" * 23)
        pprint.pprint(self.objs)
        if self.robot:
            print(self.robot.robot_name, self.robot.offset)
            if self.robot.has_gripper:
                print(self.robot.gripper.name, self.robot.gripper.get_gripper_pose())
        print(f"*" * 63 + "\n")

    def show_logical_states(self):
        print(f"*" * 23 + f" {sc.OKGREEN}Logical States{sc.ENDC} " + f"*" * 23)
        pprint.pprint(self.logical_states)
        print(f"*" * 63 + "\n")

    def update_logical_states(self):
        for object_name, logical_state in self.logical_states.items():
            if logical_state.get(State.on):
                l_s_on = logical_state[State.on]
                if isinstance(l_s_on, list):
                    for on_obj in l_s_on:
                        if not self.logical_states[on_obj.name].get(State.support):
                            self.logical_states[on_obj.name][State.support] = []
                        if self.objs[object_name] not in list(
                            self.logical_states[on_obj.name].get(State.support)
                        ):
                            self.logical_states[on_obj.name][State.support].append(
                                self.objs[object_name]
                            )
                else:
                    if not self.logical_states[l_s_on.name].get(State.support):
                        self.logical_states[l_s_on.name][State.support] = []

                    if self.objs[object_name] not in list(
                        self.logical_states[l_s_on.name].get(State.support)
                    ):
                        self.logical_states[l_s_on.name][State.support].append(
                            self.objs[object_name]
                        )

            if logical_state.get(State.support) is not None and not logical_state.get(
                State.support
            ):
                self.logical_states[object_name].pop(State.support)

            if self.bench_num == 4:
                if logical_state.get(State.hang):
                    if not self.logical_states[logical_state[State.hang].name].get(
                        State.hung
                    ):
                        self.logical_states[logical_state[State.hang].name][
                            State.hung
                        ] = []

                    if self.objs[object_name] not in list(
                        self.logical_states[logical_state[State.hang].name].get(
                            State.hung
                        )
                    ):
                        self.logical_states[logical_state[State.hang].name][
                            State.hung
                        ].append(self.objs[object_name])

                if logical_state.get(State.hung) is not None and not logical_state.get(
                    State.hung
                ):
                    self.logical_states[object_name].pop(State.hung)

            if logical_state.get(State.holding):
                self.logical_states[logical_state[State.holding].name][
                    State.held
                ] = True

    # Add for MCTS
    def is_terminal_state(self):
        if self.bench_num == 1:
            return self.check_terminal_state_bench_1()
        if self.bench_num == 2:
            return self.check_terminal_state_bench_2()
        if self.bench_num == 3:
            return self.check_terminal_state_bench_3()
        if self.bench_num == 4:
            return self.check_terminal_state_bench_4()

    def check_terminal_state_bench_1(self):
        is_success = self.check_success_stacked_bench_1(is_terminal=True)
        if is_success and self.stacked_box_num == self.goal_stacked_num:
            return True
        return False

    def check_success_stacked_bench_1(self, is_terminal=False):
        is_success = False

        stacked_boxes = self.get_objs_chain_list_from_bottom(self.goal_object)[1:]

        stacked_box_num = len(stacked_boxes)
        self.stacked_box_num = stacked_box_num

        if stacked_box_num <= self.goal_stacked_num:
            goal_stacked_boxes = self.goal_objects[:stacked_box_num]
            if stacked_boxes == goal_stacked_boxes:
                is_success = True
                if not is_terminal:
                    self.success_stacked_box_num = stacked_box_num

        return is_success

    def get_objs_chain_list_from_bottom(self, bottom_obj):
        support_objs: list = self.logical_states[bottom_obj].get(
            self.logical_state.support
        )
        if not support_objs:
            return [bottom_obj]
        else:
            upper_obj = support_objs[0].name
            return [bottom_obj] + self.get_objs_chain_list_from_bottom(upper_obj)

    def check_terminal_state_bench_2(self):
        is_success = False
        if self.robot.gripper.attached_obj_name == self.goal_object:
            is_success = True
        return is_success

    def get_prev_place_object(self, pick_obj_name):
        if pick_obj_name is not None:
            return self.logical_states[pick_obj_name].get(self.logical_state.on).name

    def get_pose_from_goal_obj(self, pick_obj_name):
        if pick_obj_name is not None:
            return self.objs[pick_obj_name].h_mat

    def check_terminal_state_bench_3(self):
        is_success = False
        if self.robot.gripper.attached_obj_name == self.goal_object:
            if self.cur_place_obj_name == "tray_blue":
                is_success = True
        return is_success

    def get_distance_between_obj1_and_obj2(self, obj1_name, obj2_name):
        obj1_pos = self.objs[obj1_name].h_mat[:2, 3]
        obj2_pos = self.objs[obj2_name].h_mat[:2, 3]
        distance = np.linalg.norm(obj1_pos - obj2_pos)
        return distance

    def check_terminal_state_bench_4(self):
        is_success = False

        stacked_disks = self.get_objs_chain_list_from_bottom(self.goal_object)
        stacked_box_num = len(stacked_disks)
        peg = self.logical_states[self.goal_object].get(self.logical_state.hang)
        if peg is not None:
            if peg.name == self.pegs[2] and stacked_box_num == self.disk_num:
                peg_y_pos = self.get_pose_from_goal_obj(peg.name)[1, 3]
                for disk in stacked_disks:
                    disk_y_pos = self.get_pose_from_goal_obj(disk)[1, 3]
                    if not np.isclose(disk_y_pos, peg_y_pos, 1e-05):
                        is_success = False
                        break
                    else:
                        is_success = True
        return is_success
