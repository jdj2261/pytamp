import math
import numpy as np
import networkx as nx

from pykin.utils.log_utils import create_logger
from pykin.utils.kin_utils import ShellColors as sc, logging_time
from pykin.utils.transform_utils import get_linear_interpoation
from pytamp.planners.planner import NodeData, Planner
from pytamp.scene.scene_manager import SceneManager

logger = create_logger("PRM Star Planner", "debug")


class PRMStarPlanner(Planner):
    """
    PRM star path planner

    Args:
        delta_distance(float): distance between nearest point and new point
        epsilon1(float): 1-epsilon is probability of random sampling, rrt star에서의 epsilon과 같음
        epsilon2(float): random sampling에서, goal과의 오차 범위 
        gamma_PRM_star(int): factor used for search radius
        dimension(int): robot arm's dof
    """

    def __init__(self, epsilon1=0.1, epsilon2=0.1, gamma_PRM_star=8, dimension=7):
        super(PRMStarPlanner, self).__init__(dimension)
        self.gamma_PRMs = gamma_PRM_star
        self.epsilon1 = epsilon1
        self.epsilon2 = epsilon2
        self.tree = None

    def __repr__(self):
        return "pykin.planners.prm_star_planner.{}()".format(type(self).__name__)

    @staticmethod
    def _create_tree():
        tree = nx.Graph()
        tree.add_node(0)
        tree.update(nodes=[(0, {NodeData.POINT: None})])
        return tree

    @logging_time
    def run(
        self,
        scene_mngr: SceneManager,
        cur_q,
        goal_pose=np.eye(4),
        goal_q=None,
        max_iter=1000,
    ):
        """
        Compute prm-star path

        Args:
            cur_q (sequence of float): current joints
            goal_pose (h_mat): goal pose
            max_iter(int): maximum number of iterations
        """
        if not scene_mngr:
            raise ValueError("SceneManager needs to be added first")

        logger.info(f"Start to compute PRM-star Planning")

        self._scene_mngr = scene_mngr
        super()._setup_q_limits()
        super()._setup_eef_name()

        self._cur_qpos = super()._convert_numpy_type(cur_q)
        self._goal_pose = super()._convert_numpy_type(goal_pose)
        self._max_iter = max_iter
        self.goal_node_cost = 0

        if not super()._check_robot_col_mngr():
            logger.warning(f"This Planner does not do collision checking")

        cnt = 0
        total_cnt = 2
        init_q = self._cur_qpos

        while True:
            cnt += 1
            limit_cnt = 0
            success_check_limit = False
            if goal_q is not None:
                self.goal_q = goal_q
                self._goal_pose = self._scene_mngr.scene.robot.forward_kin(self.goal_q)[
                    self._scene_mngr.scene.robot.eef_name
                ].h_mat
                success_check_limit = True
            else:
                while not success_check_limit:
                    limit_cnt += 1
                    if limit_cnt > 50:
                        break

                    # goal q 계산(ik)
                    self.goal_q = self._scene_mngr.scene.robot.inverse_kin(
                        init_q, self._goal_pose, max_iter=100
                    )

                    # 계산한 q가 limit 넘으면 init_q 랜덤으로 바꾸고 반복문 처음으로
                    if not self._check_q_in_limits(self.goal_q):
                        init_q = np.random.randn(self._scene_mngr.scene.robot.arm_dof)
                        continue

                    # goal_q가 limit 안에 있을 때
                    self._scene_mngr.set_robot_eef_pose(self.goal_q)
                    grasp_pose_from_ik = (
                        self._scene_mngr.get_robot_eef_pose()
                    )  # goal_q로 그립퍼 포지션 계산한 값 저장
                    pose_error = self._scene_mngr.scene.robot.get_pose_error(
                        self._goal_pose, grasp_pose_from_ik
                    )  # 에러 계산

                    if pose_error < 0.02:
                        success_check_limit = True
                        logger.info(
                            f"The joint limit has been successfully checked. Pose error is {pose_error:6f}"
                        )
                        # limit 체크 완료 로그 출력

                        result, names = self._collide(self.goal_q, visible_name=True)
                        if result:  # 충돌 있으면 충돌 경고랑 info 표시, limit 체크 다시 false로
                            print(names)
                            logger.warning("Occur Collision for goal joints")
                            success_check_limit = False

                            self._scene_mngr.show_scene_info()
                            self._scene_mngr.robot_collision_mngr.show_collision_info()
                            self._scene_mngr.obj_collision_mngr.show_collision_info(
                                "Object"
                            )

                            # ![DEBUG]
                            if self._scene_mngr.is_debug_mode:
                                self._scene_mngr.render_debug(title="Collision Fail")
                    else:
                        if limit_cnt > 1:
                            print(
                                f"{sc.WARNING}Retry compute IK.. Pose error is {pose_error:6f}{sc.ENDC} "
                            )  # 에러 크면 다시 계산
                    init_q = np.random.randn(
                        self._scene_mngr.scene.robot.arm_dof
                    )  # 초기 위치 바꾸기

            if not success_check_limit:
                self.tree = None
                logger.error("Not found IK solution")
                if self._scene_mngr.is_debug_mode:
                    self._scene_mngr.render_debug(title="IK Fail")
                break
                ####위에는 동일(goal_q 찾고 Pose error 계산)

            self.goal_node = None
            self.tree = self._create_tree()  # tree 생성
            self.tree.nodes[0][NodeData.POINT] = self._cur_qpos  # 맨 처음 node에 현재 q
            self.tree.update(nodes=[(1, {NodeData.POINT: self.goal_q})])  # 1에 goal node

            # PRM*(make tree)
            i = 1
            for step in range(self._max_iter):
                if step % 100 == 0 and step != 0:  # step 100의 배수마다 진행상황 출력
                    logger.info(f"iter : {step}")
                i = i + 1

                q_new = self._sample_free()  # 랜덤 위치 생성

                if self._collide(q_new) or not self._check_q_in_limits(
                    q_new
                ):  # 생성된 랜덤 위치 충돌 체크
                    i = i - 1
                    continue

                self.tree.update(nodes=[(i, {NodeData.POINT: q_new})])

                near_nodes = self._near(q_new)
                for near_node in near_nodes:
                    col_check = True
                    path = self._get_linear_path(
                        self.tree.nodes[near_node][NodeData.POINT], q_new, 5
                    )
                    for pos in path:
                        if self._collide(pos):
                            col_check = False
                            break
                    if col_check:
                        self.tree.add_edge(near_node, i)

            print("num of edges: ", len(self.tree.edges))

            if 0 in nx.algorithms.descendants(
                self.tree, 1
            ):  # 0이랑 goal node 연결되어 있으면 성공로그
                logger.info(f"Generate Tree Successfully!!")
                break

            if cnt > total_cnt:  # 10번 넘으면 실패, tree 초기화
                logger.error(
                    f"Failed Generate Tree.. The number of retries of {cnt} exceeded"
                )
                self.tree = None
                if self._scene_mngr.is_debug_mode:
                    self._scene_mngr.render_debug(title="Excess")
                break
            self._max_iter += 100

            logger.error(f"Failed Generate Tree..")
            print(
                f"{sc.BOLD}Retry Generate Tree, the number of retries is {cnt}/{total_cnt} {sc.ENDC}\n"
            )

    def get_joint_path(self, n_step=10):
        """
        Get path in joint space

        Args:
            goal_node(int): goal node in PRM path
            n_step(int): number for n equal divisions between waypoints
    
        Returns:
            interpolate_paths(list) : interpoated paths from start joint pose to goal joint
        """
        if self.tree is None:
            return

        # A* search
        open_set = [0]
        closed_set = []
        past_cost = [np.inf] * (len(self.tree.nodes) + 1)
        past_cost[0] = 0
        parent_node = [None] * (len(self.tree.nodes) + 1)
        est_tot_cost = [np.inf] * (len(self.tree.nodes) + 1)
        path_node = []

        while open_set:
            current = open_set[0]
            del open_set[0]
            closed_set.append(current)

            if current == 1:
                logger.info(f"Path find success!")
                self.goal_node = current
                path = [self.tree.nodes[self.goal_node][NodeData.POINT]]
                path_node = [current]
                break

            nbr_nodes = self._get_nbr(current)
            for nbr_node in nbr_nodes:
                if not nbr_node in closed_set:
                    tentative_cost = past_cost[current] + np.linalg.norm(
                        self.tree.nodes[current][NodeData.POINT]
                        - self.tree.nodes[nbr_node][NodeData.POINT]
                    )

                    if tentative_cost < past_cost[nbr_node]:
                        past_cost[nbr_node] = tentative_cost
                        parent_node[nbr_node] = current
                        est_tot_cost[nbr_node] = past_cost[nbr_node] + self._h_cost(
                            nbr_node
                        )

                        if not open_set:
                            open_set.append(nbr_node)
                        else:
                            for i in range(len(open_set)):
                                if est_tot_cost[nbr_node] <= est_tot_cost[open_set[i]]:
                                    open_set.insert(i, nbr_node)
                                    break

        if path_node:
            while True:
                if path_node[-1] == 0:
                    break
                path_node.append(parent_node[path_node[-1]])
                path.append(self.tree.nodes[path_node[-1]][NodeData.POINT])

        path.reverse()
        path_node.reverse()

        # print(path_node)
        self.goal_node_cost = round(past_cost[self.goal_node], 3)
        print("PRM cost is ", self.goal_node_cost)
        unique_path = []
        for joints in path:
            if not any(
                np.array_equal(np.round(joints, 2), np.round(unique_joints, 2))
                for unique_joints in unique_path
            ):
                unique_path.append(joints)

        if n_step == 1:
            logger.info(f"Path Length : {len(unique_path)}")
            return unique_path

        interpolate_path = []
        interpolate_paths = []
        for i in range(len(unique_path) - 1):
            interpolate_path = np.array(
                [
                    unique_path.tolist()
                    for unique_path in self._get_linear_path(
                        unique_path[i], unique_path[i + 1], n_step
                    )
                ]
            )
            interpolate_paths.extend(interpolate_path)
        logger.info(f"Path length {len(unique_path)} --> {len(interpolate_paths)}")
        self.joint_path = interpolate_paths
        return interpolate_paths

    # def get_prm_tree(self):
    #     """
    #     Return obtained PRM Trees

    #     Returns:
    #         tree(list)
    #     """
    #     tree = []
    #     for edge in self.tree.edges:
    #         from_node = self.tree.vertices[edge[0]]
    #         goal_node = self.tree.vertices[edge[1]]
    #         tree.append((from_node, goal_node))
    #     return tree

    def _sample_free(self):
        """
        sampling joints in q space within joint limits

        Returns:
            q_outs(np.array)
        """
        q_outs = np.zeros(self._dimension)
        random_value = np.random.random()
        for i, (q_min, q_max) in enumerate(
            zip(self.q_limits_lower, self.q_limits_upper)
        ):
            if random_value > self.epsilon1:
                q_outs[i] = np.random.uniform(q_min, q_max)
            else:
                minmax = q_max - q_min
                q_outs[i] = np.random.uniform(
                    self.goal_q[i] - self.epsilon2 * minmax,
                    self.goal_q[i] + self.epsilon2 * minmax,
                )

        return q_outs

    def _get_distance(self, p1, p2):
        """
        Get distance from pointA to pointB

        Args:
            p1(np.array)
            p2(np.array)
            
        Returns:
            Norm(float or ndarray)
        """

        return np.linalg.norm(p2 - p1)

    def _near(self, q_rand):
        """
        Returns all neighbor nodes within the search radius from the new point

        Args:
            q_rand(np.array): new joint angles 

        Returns:
            near_nodes(list): all neighbor nodes
        """
        card_V = len(self.tree.nodes) + 1
        search_radius = self.gamma_PRMs * (
            (math.log(card_V) / card_V) ** (1 / self._dimension)
        )
        distances = [
            self._get_distance(self.tree.nodes[node][NodeData.POINT], q_rand)
            for node in self.tree.nodes
        ]

        near_nodes = []
        for node, dist in enumerate(distances):
            if dist <= search_radius:
                near_nodes.append(node)

        near_nodes.pop(-1)
        return near_nodes

    def _get_nbr(self, current):
        """
        Returns all neighbor nodes within the search radius from the current point

        Args:
            current(int): current node

        Returns:
            nbr(np.array): neighbor nodes
        """
        nbr = [node for node in nx.neighbors(self.tree, current)]
        return nbr

    def _h_cost(self, node):
        """
        Return heuristic cost (distance from node to goal_q )

        Args:
           node(int)

        Returns:
            Norm(float or ndarray): heuristic cost
        """
        return np.linalg.norm(self.goal_q - self.tree.nodes[node][NodeData.POINT])

    def _get_linear_path(self, init_pose, goal_pose, n_step=1):
        """
        Get linear path (only qpos)

        Args:
            init_pose (np.array): init robots' eef pose
            goal_pose (np.array): goal robots' eef pose  
            n_step(int): number for n equal divisions between waypoints
        
        Return:
            pos (np.array): position
        """
        for step in range(1, n_step + 1):
            delta_t = step / n_step
            pos = get_linear_interpoation(init_pose, goal_pose, delta_t)
            yield pos

    @property
    def max_iter(self):
        return self._max_iter

    @max_iter.setter
    def max_iter(self, max_iter):
        self._max_iter = max_iter
