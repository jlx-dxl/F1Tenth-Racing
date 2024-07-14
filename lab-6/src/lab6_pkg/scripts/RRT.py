#!/usr/bin/env python3

import numpy as np
import math
import random
from time import time

class TreeNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.path_x = []
        self.path_y = []
        self.parent = None
        self.cost = 0.0


class RRTStar:
    """
    Class for RRT*
    """
    def __init__(self,start,goal,occupancy_grid_map,collision_tolerance=0.5,expand_dis_threshold=0.5,path_resolution=0.05,
                 goal_sample_rate=5,max_iter=500,neighborhood_range=0.5,if_early_stop=True,smooth=False,smoother_type="dp",
                 smoother_iter=100):
        """
        Initicalization
        """
        # start node and end node
        self.start = TreeNode(start[0], start[1])
        self.end = TreeNode(goal[0], goal[1])
        # about occupancy map
        self.occupancy_grid_map = occupancy_grid_map
        self.grid_status = occupancy_grid_map[:, :, 0].flatten()
        self.grid_x = occupancy_grid_map[:, :, 1].flatten()
        self.grid_y = occupancy_grid_map[:, :, 2].flatten()
        occupied_x = self.grid_x[self.grid_status == 1.0]
        occupied_y = self.grid_y[self.grid_status == 1.0]
        self.occupied_positions = np.vstack((occupied_x, occupied_y))
        self.xmin = np.min(occupancy_grid_map[:, :, 1])    # boundaries
        self.xmax = np.max(occupancy_grid_map[:, :, 1])
        self.ymin = np.min(occupancy_grid_map[:, :, 2])
        self.ymax = np.max(occupancy_grid_map[:, :, 2])
        # params for RRTStar
        self.collision_tolerance = collision_tolerance
        self.expand_dis_threshold = expand_dis_threshold
        self.path_resolution = path_resolution   # step size
        self.goal_sample_rate = goal_sample_rate   # in percentage (%)
        self.max_iter = max_iter
        self.neighborhood_range = neighborhood_range
        self.if_early_stop = if_early_stop
        # optimize the trajector
        self.smooth = smooth
        self.smoother_type = smoother_type
        self.smoother_iter = smoother_iter

        self.tree = []

    def planning(self):
        """
        main loop
        """
        self.tree = [self.start]
        for _ in range(self.max_iter):
            # 1. sample a node
            rnd_node = self.sample()
            # 2. find the closest node on the tree
            nearest_node = self.find_nearest(self.tree, rnd_node)
            # 3. step forward for steps
            new_node = self.step_forward(nearest_node, rnd_node, expand_dis_threshold=self.expand_dis_threshold)
            # 4. update cost
            new_node.cost = nearest_node.cost + self.euclidean_cost(nearest_node, new_node)
            # 5. check for collision
            if self.check_collision(new_node):
                continue
            # 6. find neighbors and choose parent
            neighbors = self.find_neibors(self.tree, new_node)
            new_node_with_parent = self.choose_parent(new_node, neighbors)
            # 7. rewire
            if new_node_with_parent:
                self.rewire(new_node_with_parent, neighbors)
                self.tree.append(new_node_with_parent)
            else:
                self.tree.append(new_node)
            # 8. if search feasible or optimal
            if self.if_early_stop:  # if reaches goal
                last_idx = self.search_best_goal_node()
                if last_idx is not None:
                    return self.find_path(self.tree[last_idx], if_smooth=self.smooth)

        last_idx = self.search_best_goal_node()
        if last_idx is not None:
            return self.find_path(self.tree[last_idx], if_smooth=self.smooth)

        return None, None  # cannot find path

    def sample(self):
        """
        randomly sample a viable node

        Args:
        Returns:
            node (TreeNode)

        """
        if random.randint(0, 100) > self.goal_sample_rate:
            x = random.uniform(self.xmin, self.xmax)
            y = random.uniform(self.ymin, self.ymax)
        else:  # goal point sampling
            x = self.end.x
            y = self.end.y

        return TreeNode(x, y)

    @staticmethod
    def find_nearest(tree:list, sampled_node:TreeNode):
        """
        find the nearest node on the tree to the sampled node

        Returns:
            nearest_node (TreeNode): the nearest node on the tree
        """
        dlist = [(node.x - sampled_node.x) ** 2 + (node.y - sampled_node.y) ** 2
                 for node in tree]
        min_idx = dlist.index(min(dlist))
        return tree[min_idx]


    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        """
        helper function
        """
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta
    

    def step_forward(self, nearest_node:TreeNode, sampled_node:TreeNode, expand_dis_threshold=float('inf')):
        """
        take one step toward the sampled node

        Returns:
            new_node (TreeNode): new node created from steering
        """
        new_node = TreeNode(nearest_node.x, nearest_node.y)
        d, theta = self.calc_distance_and_angle(new_node, sampled_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        expand_dis_threshold = min(expand_dis_threshold, d)

        # number of step forward
        n_expand = math.floor(expand_dis_threshold / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, sampled_node)
        
        if d <= self.path_resolution:
            new_node.path_x.append(sampled_node.x)
            new_node.path_y.append(sampled_node.y)
            new_node.x = sampled_node.x
            new_node.y = sampled_node.y

        new_node.parent = nearest_node

        return new_node


    def check_collision(self, new_node):
        """
        This method should return whether the path between nearest and new_node is
        collision free.

        Args:
            new_node (TreeNode): new node from steering
        Returns:
            collision (bool): whether the path between the two nodes are in collision
                              with the occupancy grid
        """
        if new_node is None:
            return False

        for pos in zip(new_node.path_x, new_node.path_y):
            if self.dist_to_grid(pos) < self.collision_tolerance:
                return True  # collision

        return False  # safe

    def dist_to_grid(self, pos):
        """
        Calculate distance to occupancy grid

        Args:
            pos (numpy.ndarray or (x, y)): current position
        Returns:
            dist (float): distance to occupancy grid

        """
        return np.min(np.linalg.norm(self.occupied_positions.T - np.array(pos), axis=-1))

    def is_goal(self, latest_added_node, goal_node):
        """
        This method should return whether the latest added node is close enough
        to the goal.

        Args:
            latest_added_node (TreeNode): latest added node on the tree
            goal_node (TreeNode): goal node
        Returns:
            close_enough (bool): true if node is close enough to the goal
        """
        dx = latest_added_node.x - goal_node.x
        dy = latest_added_node.y - goal_node.y
        if math.hypot(dx, dy) > self.expand_dis_threshold:
            return False  # not close to goal
        final_node = self.step_forward(latest_added_node, goal_node, expand_dis_threshold=self.expand_dis_threshold)
        if self.check_collision(final_node):
            return False  # close enough but has collision

        return True

    def find_path(self, latest_added_node:TreeNode, if_smooth=False):
        """
        find the path by backtracking the tree in reverse.

        Args:
            latest_added_node (TreeNode): latest added node in the tree
            smooth (bool): whether to use path smoother
        Returns:
            path ([]): valid path as a list of TreeNodes
            smooth_path ([]): smoothed path as a list of TreeNodes
        """
        path = []
        if latest_added_node.x != self.end.x or latest_added_node.y != self.end.y:
            path.append([self.end.x, self.end.y])

        node = latest_added_node
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])
        path.reverse()

        if not if_smooth:
            smooth_path = None
        elif self.smoother_type == "rand":
            smooth_path = self.path_smoother_rand(path, max_iter=self.smoother_iter)
        elif self.smoother_type == "dp":
            smooth_path = self.path_smoother_dp(path)
        else:
            raise ValueError("Invalid smoother type")

        return path, smooth_path

    # The following methods are needed for RRT* and not RRT
    def choose_parent(self, new_node:TreeNode, neighbors:list):
        """
        find the cheapest point from the neighbors to the new_node 
        and set such a node as the parent of new_node.

        Returns:
            new_node (TreeNode): updated new_node
        """
        if not neighbors:
            return None

        # Search the nearest cost in neighbors
        costs = []
        for i in neighbors:
            neighbor = self.tree[i]
            t_node = self.step_forward(neighbor, new_node, expand_dis_threshold=self.expand_dis_threshold)
            if t_node and not self.check_collision(t_node):
                costs.append(self.cost(neighbor, new_node))
            else:
                costs.append(float("inf"))  # the cost of collision node

        min_cost = min(costs)
        if min_cost == float("inf"):
            return None

        min_idx = neighbors[costs.index(min_cost)]
        new_node = self.step_forward(self.tree[min_idx], new_node, expand_dis_threshold=self.expand_dis_threshold)
        new_node.cost = min_cost

        return new_node


    def rewire(self, new_node: TreeNode, neighbors:list):
        """
        check for each node in neighborhood, if it is cheaper to arrive to them from new_node, 
        re-assign the parent of the nodes in neighbors to new_node
        """
        for i in neighbors:
            neighbor = self.tree[i]
            edge_node = self.step_forward(new_node, neighbor, expand_dis_threshold=self.expand_dis_threshold)
            if not edge_node:
                continue
            edge_node.cost = self.cost(new_node, neighbor)

            collision = self.check_collision(edge_node)
            improved_cost = neighbor.cost > edge_node.cost

            if not collision and improved_cost:
                self.tree[i].x = edge_node.x
                self.tree[i].y = edge_node.y
                self.tree[i].cost = edge_node.cost
                self.tree[i].path_x = edge_node.path_x
                self.tree[i].path_y = edge_node.path_y
                self.tree[i].parent = edge_node.parent
        self.propagate_cost_to_leaves(new_node)


    def search_best_goal_node(self):
        """
        search the cheapest point to reach goal

        Args:
        Returns:
            best_goal_node (int): index of the cheapest node that is close enough to goal

        """
        goal_indices = []
        for idx, node in enumerate(self.tree):
            if not self.is_goal(node, self.end):
                continue
            goal_indices.append(idx)

        if not goal_indices:
            return None

        min_cost = min([self.tree[i].cost for i in goal_indices])
        for i in goal_indices:
            if self.tree[i].cost == min_cost:
                return i

        return None

    def cost(self, from_node:TreeNode, to_node:TreeNode):
        """
        calculate the cost of a node

        Returns:
            cost (float): the cost value of the node
        """

        return from_node.cost + self.euclidean_cost(from_node, to_node)

    @staticmethod
    def euclidean_cost(node_1, node_2):
        """
        calculate the cost(distance) of the straight line between n1 and n2

        Returns:
            cost (float): the cost value of the line
        """

        return math.hypot(node_1.x - node_2.x, node_1.y - node_2.y)


    def propagate_cost_to_leaves(self, parent_node:TreeNode):
        """
        recursively propagate the cost from parent node to leaf node
        """
        for node in self.tree:
            if node.parent == parent_node:
                node.cost = self.cost(parent_node, node)
                self.propagate_cost_to_leaves(node)


    def find_neibors(self, tree:TreeNode, new_node:TreeNode):
        """
        find neighbors around the given node within some range

        Returns:
            neighborhood (list of int): indices of neighborhood(on the tree list)
        """
        n_node = len(tree) + 1
        r = self.neighborhood_range * math.sqrt((math.log(n_node) / n_node))
        r = min(r, self.expand_dis_threshold)

        dlist = [(node.x - new_node.x) ** 2 + (node.y - new_node.y) ** 2 for node in tree]
        neighbors = [dlist.index(i) for i in dlist if i <= r ** 2]

        return neighbors

    @staticmethod
    def get_path_length(path):
        """
        This method should return the length of the whole path

        Args:
            path (list of [x, y]): path as 2d list
        Returns:
            length (float): length of the whole path
        """
        length = 0
        for i in range(len(path) - 1):
            dx = path[i + 1][0] - path[i][0]
            dy = path[i + 1][1] - path[i][1]
            d = math.hypot(dx, dy)
            length += d

        return length

    @staticmethod
    def get_target_point(path, target_length):
        """
        This method should return target point based on its distance to path's start point

        Args:
            path (list of [x, y]): path as 2d list
            target_length (float): sampled point's distance to start point
        Returns:
            target_point ([x, y, idx]): target point
        """
        length = 0
        idx = 0
        curr_length = 0
        for i in range(len(path) - 1):
            dx = path[i + 1][0] - path[i][0]
            dy = path[i + 1][1] - path[i][1]
            d = math.sqrt(dx * dx + dy * dy)
            length += d
            if length >= target_length:
                idx = i - 1
                curr_length = d
                break

        partRatio = (length - target_length) / curr_length

        x = path[idx][0] + (path[idx + 1][0] - path[idx][0]) * partRatio
        y = path[idx][1] + (path[idx + 1][1] - path[idx][1]) * partRatio

        return [x, y, idx]

    def path_smoother_rand(self, path, max_iter=100):
        """
        Remove redundant waypoints to smooth path using random sampling

        Args:
            path (list of [x, y]): rrt path as a list of TreeNodes
            max_iter (int): maximum iterations
        Returns:
            smooth_path (list of [x, y]): smoothed path as a list of TreeNodes

        """
        path_length = self.get_path_length(path)

        for i in range(max_iter):
            # Sample two points
            pickPoints = [random.uniform(0, path_length), random.uniform(0, path_length)]
            pickPoints.sort()
            first = self.get_target_point(path, pickPoints[0])
            second = self.get_target_point(path, pickPoints[1])

            # Check valid index
            if first[2] <= 0 or second[2] <= 0:
                continue
            if (second[2] + 1) > len(path):
                continue
            if second[2] == first[2]:
                continue

            # Check no collision
            node_1 = TreeNode(first[0], first[1])
            node_2 = TreeNode(second[0], second[1])
            new_node = self.step_forward(node_1, node_2)
            if self.check_collision(new_node):
                continue

            # Create New path
            newPath = []
            newPath.extend(path[:first[2] + 1])
            newPath.append([first[0], first[1]])
            newPath.append([second[0], second[1]])
            newPath.extend(path[second[2] + 1:])
            path = newPath
            path_length = self.get_path_length(path)

        return path

    def path_smoother_dp(self, path):
        """
        Remove redundant waypoints to smooth path using dynamic programming

        Args:
            path (list of [x, y]): rrt path as a list of TreeNodes
        Returns:
            smooth_path (list of [x, y]): smoothed path as a list of TreeNodes

        """
        n = len(path)

        # Construct dynamic programming table with size (n,)
        # dp[i] represents minimum cost from node 0 to node i

        # Initialization:
        #     dp[i] = 0   for i == 0
        #     dp[i] = +∞  for i != 0
        dp = np.full((n,), np.inf)
        dp[0] = 0

        # Also keep track of each node's parent
        # Initialize parent[i + 1] = i
        parents = {}
        for i in range(n - 1):
            parents[i + 1] = i

        # Iterate node index to fill dp table
        node_0 = TreeNode(path[0][0], path[0][1])
        for i in range(1, n):
            node_i = TreeNode(path[i][0], path[i][1])

            # If two nodes can be connected directly
            new_node = self.step_forward(node_0, node_i, expand_dis_threshold=float('inf'))
            if not self.check_collision(new_node):
                dp[i] = self.euclidean_cost(node_0, node_i)  # cost equal to nodes distance
                parents[i] = 0  # update node i parent
                continue

            # If two nodes cannot be connected directly,
            # then dp[i] = min{dp[j] + d(j, i)} for 0 < j < i
            for j in range(1, i):
                node_j = TreeNode(path[j][0], path[j][1])
                new_node = self.step_forward(node_j, node_i, expand_dis_threshold=float('inf'))
                if self.check_collision(new_node):
                    continue  # cannot connect j and i directly
                cost = dp[j] + self.euclidean_cost(node_j, node_i)
                if cost >= dp[i]:
                    continue
                dp[i] = cost
                parents[i] = j

        # Back track final path
        smooth_path = []
        node_idx = n - 1
        while node_idx != 0:
            smooth_path.append(path[node_idx])
            node_idx = parents[node_idx]
        smooth_path.append(path[0])
        smooth_path.reverse()

        return smooth_path