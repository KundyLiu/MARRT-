'''
10 agents. configA. differentGoals.
This solution need 'Pareto optimal' part.
This solution solve collision with priority based approach, min utility 'Pd' first.
'''
import os
import math
import random
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev, splrep
from call import *
plt.ion()
class RRTStar:
    class node:
        def __init__(self, x, y, t=0, cost=0):
            self.x = x
            self.y = y
            self.t = t
            self.parent = None
            self.x_path = None
            self.y_path = None
            self.cost = cost

        def pretty_print(self):
            print("x: " + str(self.x))
            print("y: " + str(self.y))
            print("t: " + str(self.t))
    def __init__(self, start, goal, s):
        self.start_node = self.node(start[0], start[1])
        self.goal_node = self.node(goal[0], goal[1])
        self.nodes = [self.start_node]
        self.lower_lim_x = 0
        self.lower_lim_y = 0
        self.upper_lim_x = 100
        self.upper_lim_y = 100
        self.neigh_dist = 1
        self.vel = 2 #1    # robot speed
        self.r = 0.177  # robot radius
        self.c = 0.4  # robot clearance
        self.s = s
        self.thresh = self.c + self.r
        self.nodes_at_t = {}
        self.other_traj = []
    def check_collision(self, node):
        ret = False
        if node.x - self.thresh < self.lower_lim_x:
            ret = True
        elif node.x + self.thresh > self.upper_lim_x:
            ret = True
        elif node.y - self.thresh < self.lower_lim_y:
            ret = True
        elif node.y + self.thresh > self.upper_lim_y:
            ret = True

        if node.x > 20 - self.thresh and node.x < 35 + self.thresh and node.y > 0.9 - self.thresh and node.y < 10.7 + self.thresh:
            ret = True
        if node.x > 70 - self.thresh and node.x < 85 + self.thresh and node.y > 0.9 - self.thresh and node.y < 10.9 + self.thresh:
            ret = True
        if node.x > 45 - self.thresh and node.x < 53 + self.thresh and node.y > 57 - self.thresh and node.y < 63 + self.thresh:
            ret = True
        if node.x > 35 - self.thresh and node.x < 60 + self.thresh and node.y > 25 - self.thresh and node.y < 43 + self.thresh:
            ret = True
        if node.x > 30 - self.thresh and node.x <35  + self.thresh and node.y > 63 - self.thresh and node.y < 83 + self.thresh:
            ret = True
        if node.x > 18 - self.thresh and node.x < 26 + self.thresh and node.y > 81 - self.thresh and node.y < 89 + self.thresh:
            ret = True
        if node.x > 70 - self.thresh and node.x < 78 + self.thresh and node.y > 80 - self.thresh and node.y < 88 + self.thresh:
            ret = True
        if node.x > 70 - self.thresh and node.x < 90 + self.thresh and node.y > 50 - self.thresh and node.y < 55 + self.thresh:
            ret = True
        if node.x > 3 - self.thresh and node.x < 8 + self.thresh and node.y > 60 - self.thresh and node.y < 65 + self.thresh:
            ret = True
        for traj in self.other_traj:
            t = node.t
            if node.t > len(traj[0])-1:
                t = len(traj[0])-1

            if np.sqrt((node.x - traj[0][t])**2 + (node.y - traj[1][t])**2) < self.s:
                ret = Tru
        return ret
    def goal_check(self, node):
        if self.get_dist(node, self.goal_node) < 5:
            return True
        return False
    def get_random_node(self):
        x = random.randint(1, 100)
        y = random.randint(1, 100)
        new_node = self.node(x, y)
        return new_node
    def get_dist(self, node1, node2):
        return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)
    def get_nearest_node(self, rand_node, nodes):
        nearest_node_idx = 0
        min_dist = float('inf')
        for i, node in enumerate(nodes):
            dist = self.get_dist(rand_node, node)
            if dist < min_dist:
                nearest_node_idx = i
                min_dist = dist
        return nearest_node_idx
    def step_ahead(self, parent, dest_node):
        par_x = parent.x
        par_y = parent.y
        tm = parent.t
        dest_x = dest_node.x
        dest_y = dest_node.y
        theta = np.arctan2((dest_y - par_y), (dest_x - par_x))
        count = 0
        x_path = []
        y_path = []
        x = par_x
        y = par_y
        dist = 0
        while (count < 10):
            dx = 0.1 * math.cos(theta) * self.vel
            dy = 0.1 * math.sin(theta) * self.vel
            x = x + dx
            y = y + dy
            dist = dist + np.sqrt(dx ** 2 + dy ** 2)
            if self.check_collision(self.node(x, y, t=tm + count + 1)):
                return None
            x_path.append(x)
            y_path.append(y)
            count = count + 1
        new_node = self.node(x, y, t=parent.t + 10)
        new_node.parent = parent
        new_node.x_path = x_path
        new_node.y_path = y_path
        new_node.cost = parent.cost + dist
        return new_node
    def add_to_nodes_dict(self, new_node, index):
        t = new_node.t
        nodes = self.nodes_at_t.get(t)

        if nodes == None:
            self.nodes_at_t.update({t: [index]})

        else:
            nodes.append(index)
            self.nodes_at_t.update({t: nodes})
    def get_path(self, parent, child):
        dist = self.get_dist(parent, child)

        if (dist * 10) % self.vel == 0:
            max_count = (dist * 10) / self.vel
        else:
            max_count = (dist * 10) / self.vel + 1

        par_x = parent.x
        par_y = parent.y
        t = parent.t

        child_x = child.x
        child_y = child.y

        theta = np.arctan2((child_y - par_y), (child_x - par_x))

        count = 0
        x_path = []
        y_path = []
        x = par_x
        y = par_y
        dist = 0

        if max_count == 0:
            return None, None

        while (count < max_count):
            dx = 0.1 * math.cos(theta) * self.vel
            dy = 0.1 * math.sin(theta) * self.vel
            x = x + dx
            y = y + dy

            dist = dist + np.sqrt(dx ** 2 + dy ** 2)

            if self.check_collision(self.node(x, y, t=t + count + 1)):
                return None, None

            x_path.append(x)
            y_path.append(y)
            count = count + 1

        if x != child.x:
            child.x = x
        if y != child.y:
            child.y = y

        return x_path, y_path
    def get_neighbours(self, new_node):
        ngh_indx = []
        for i, node in enumerate(self.nodes):
            dist = self.get_dist(new_node, node)
            if dist <= self.neigh_dist:
                ngh_indx.append(i)
        return ngh_indx
    def set_parent(self, new_node, ngh_indx):
        for i in ngh_indx:
            dist = self.get_dist(new_node, self.nodes[i])
            if self.nodes[i].cost + dist < new_node.cost:
                x_path, y_path = self.get_path(self.nodes[i], new_node)
                if x_path == None:
                    continue
                new_node.t = self.nodes[i].t + len(x_path)
                new_node.x_path = x_path
                new_node.y_path = y_path
                new_node.cost = self.nodes[i].cost + dist
                new_node.parent = self.nodes[i]
    def deleteAllChildren(self, parent):
        for idx, node in enumerate(self.nodes):
            if node.parent == parent:
                del self.nodes[idx]
                self.deleteAllChildren(node)
                idx = idx - 1
    def propagate_cost_to_leaves(self, parent):
        for i, node in enumerate(self.nodes):
            if node.parent == parent:
                dist = self.get_dist(parent, node)
                node.cost = parent.cost + dist
                node.t = parent.t + len(node.x_path)
                if self.check_collision(node):
                    del self.nodes[i]
                    self.deleteAllChildren(node)
                    i = i - 1
                else:
                    self.propagate_cost_to_leaves(self.node)
    def rewire(self, new_node, ngh_indx):
        new_path_x = []
        new_path_y = []
        for i in ngh_indx:
            dist = self.get_dist(new_node, self.nodes[i])
            if new_node.cost + dist < self.nodes[i].cost:
                x_path, y_path = self.get_path(new_node, self.nodes[i])
                if x_path == None:
                    continue
                self.nodes[i].t = new_node.t + len(x_path)
                self.nodes[i].x_path = x_path
                self.nodes[i].y_path = y_path
                self.nodes[i].cost = new_node.cost + dist
                self.nodes[i].parent = new_node
                self.propagate_cost_to_leaves(self.nodes[i])
                new_path_x.append(x_path)
                new_path_y.append(y_path)
        return new_path_x, new_path_y
    def backtrace(self, cur_node):
        if (cur_node.parent == None):
            return np.asarray([cur_node.x]), np.asarray([cur_node.y]), np.asarray([cur_node.t]), np.asarray(
                [cur_node.x]), np.asarray([cur_node.x])

        x, y, t, path_x, path_y = self.backtrace(cur_node.parent)

        x_s = np.hstack((x, cur_node.x))
        y_s = np.hstack((y, cur_node.y))
        t_s = np.hstack((t, cur_node.t))
        path_x = np.hstack((path_x, cur_node.x_path))
        path_y = np.hstack((path_y, cur_node.y_path))

        # print("path_x len: " + str(len(path_x)))
        # print("path_y len: " + str(len(path_y)))
        # print("time steps: " + str(t_s[len(t_s)-1]))

        return x_s, y_s, t_s, path_x, path_y
    def smooth_path(self, res, ax, text_name, img_name):
        t = res.t
        # backtrace path
        x_s, y_s, t_s, path_x, path_y = self.backtrace(res)
        step = 0.1 * float(1 / float(t))
        m = len(x_s)
        # Path smoothing
        m = m - math.sqrt(2* m)
        tck, u = splprep([x_s, y_s], s=m)
        u_s = np.arange(0, 1.01, step)
        new_points = splev(u_s, tck)
        new_points = np.asarray(new_points)
        # Plot both trajectories
        ax.plot(x_s, y_s, color='r', linewidth=1.5)
        ax.plot(new_points[0], new_points[1], label="S", color='c', linewidth=1.5)
        plt.savefig(img_name)
        # save data in txt file
        out = new_points.T
        if os.path.exists(text_name):
            os.remove(text_name)
        f1 = open(text_name, "a")
        for i in range(len(out)):
            np.savetxt(f1, out[i], fmt="%s", newline=' ')
            f1.write("\n")
        return new_points
    def plot_again(self, ax):
        count = 0
        for n in self.nodes:
            if count == 0:
                count += 1
                continue
            cir_node = plt.Circle((n.x, n.y), 0.02, fill=True, color='r')
            ax.add_patch(cir_node)
            ax.plot(n.x_path, n.y_path, color='g', linewidth=1)

        for traj in self.other_traj:
            ax.plot(traj[0], traj[1], color='b', linewidth=1)
    def plan(self, text_name, img_name, replan=False):
        if self.check_collision(self.start_node):
            print("Start node inside obstacle")
            exit()
        if self.check_collision(self.goal_node):
            print("Goal node inside obstacle")
            exit()
        fig, ax = plt.subplots()
        ax.set_xlim([0, 100])
        ax.set_ylim([0, 100])
        cir_start = plt.Circle((self.start_node.x, self.start_node.y), 1.8, fill=True, color='m')
        cir_goal = plt.Circle((self.goal_node.x, self.goal_node.y), 1.8, fill=True, color='b')
        ax.add_patch(cir_start)
        ax.add_patch(cir_goal)
        obs1 = plt.Rectangle((20, 0.9), 15, 10, fill=True, color='k')
        ax.add_patch(obs1)
        obs2 = plt.Rectangle((70, 0.9), 15, 10, fill=True, color='k')
        ax.add_patch(obs2)
        obs3 = plt.Rectangle((45, 57), 8, 6, fill=True, color='k')
        ax.add_patch(obs3)
        obs4 = plt.Rectangle((35, 25), 25, 18, fill=True, color='k')
        ax.add_patch(obs4)
        obs5 = plt.Rectangle((30, 63), 5, 20, fill=True, color='k')
        ax.add_patch(obs5)
        obs6 = plt.Rectangle((18, 81), 8, 8, fill=True, color='k')  # ok
        ax.add_patch(obs6)
        obs7 = plt.Rectangle((70, 80), 8, 8, fill=True, color='k')  # ok
        ax.add_patch(obs7)
        obs8 = plt.Rectangle((70, 50), 20, 5, fill=True, color='k')  # ok
        ax.add_patch(obs8)
        obs9 = plt.Rectangle((3, 60), 5, 5, fill=True, color='k')  # ok
        ax.add_patch(obs9)
        if replan:
            self.plot_again(ax)
        count = 0
        while (True):
            rand_node = self.get_random_node()
            if rand_node == None:
                continue
            nearest_node_idx = self.get_nearest_node(rand_node, self.nodes)
            new_node = self.step_ahead(self.nodes[nearest_node_idx], rand_node)
            # if collision detected, continue
            if new_node == None:
                continue
            ngh_indx = self.get_neighbours(new_node)
            self.set_parent(new_node, ngh_indx)
            new_path_x, new_path_y = self.rewire(new_node, ngh_indx)
            self.nodes.append(new_node)
            index = len(self.nodes) - 1
            self.add_to_nodes_dict(new_node, index)
            cir_node = plt.Circle((new_node.x, new_node.y), 0.2, fill=True, color='r')
            ax.add_patch(cir_node)
            ax.plot(new_node.x_path, new_node.y_path, color='g', linewidth=1)
            for i in range(len(new_path_x)):
                ax.plot(new_path_x[i], new_path_y[i], color='g', linewidth=1)
            plt.pause(0.001)
            if self.goal_check(new_node):
                print("Goal reached")
                break
            count = count + 1
        traj = self.smooth_path(new_node, ax, text_name, img_name)
        return traj
    def prune(self, t):
        indx = np.asarray([])
        for key in self.nodes_at_t.keys():
            if key >= t:
                i = np.asarray(self.nodes_at_t.get(key))
                indx = np.hstack((indx, i))
                self.nodes_at_t.update({key: []})
        indx = np.asarray(indx, dtype='int')
        for idx in sorted(indx, reverse=True):
            del self.nodes[idx]
    def replan(self, trajs, t, text_name, img_name):
        self.other_traj = trajs
        self.prune(t)
        traj = self.plan(text_name, img_name, replan=True)
        return traj
    ####################################################################################################################
def check_for_replanning_util(traj1, traj2, s):
    l = min(len(traj1[0]), len(traj2[0]))
    L = max(len(traj1[0]), len(traj2[0]))
    col = False
    for i in range(l):
        dist = np.sqrt((traj1[0][i] - traj2[0][i]) ** 2 + (traj1[1][i] - traj2[1][i]) ** 2)
        if dist < (s):
            print("Collision detected at: " + str(i))
            # print(dist)
            col = True
            break
    step = -1
    if not col:
        flag = False
        if len(traj1[0]) == l:
            flag = True
        for i in range(l, L):
            if flag:
                dist = np.sqrt((traj1[0][l - 1] - traj2[0][i]) ** 2 + (traj1[1][l - 1] - traj2[1][i]) ** 2)
            else:
                dist = np.sqrt((traj1[0][i] - traj2[0][l - 1]) ** 2 + (traj1[1][i] - traj2[1][l - 1]) ** 2)
            if dist < s:
                print("Collision detected at: " + str(i))
                # print(dist)
                col = True
                step = i
                break

    return col, step
def check_for_replanning(traj1, traj2, traj3,traj4,traj5,traj6,traj7,traj8,traj9,traj10, s, flag):
    if not flag:
        s = s - 0.1
    
    col12, t12 = check_for_replanning_util(traj1, traj2, s)
    col13, t13 = check_for_replanning_util(traj1, traj3, s)
    col14, t14 = check_for_replanning_util(traj1, traj4, s)
    col15, t15 = check_for_replanning_util(traj1, traj5, s)
    col16, t16 = check_for_replanning_util(traj1, traj6, s)
    col17, t17 = check_for_replanning_util(traj1, traj7, s)
    col18, t18 = check_for_replanning_util(traj1, traj8, s)
    col19, t19 = check_for_replanning_util(traj1, traj9, s)
    col110, t110 = check_for_replanning_util(traj1, traj10, s)
    col23, t23 = check_for_replanning_util(traj2, traj3, s)
    col24, t24 = check_for_replanning_util(traj2, traj4, s)
    col25, t25 = check_for_replanning_util(traj2, traj5, s)
    col26, t26 = check_for_replanning_util(traj2, traj6, s)
    col27, t27 = check_for_replanning_util(traj2, traj7, s)
    col28, t28 = check_for_replanning_util(traj2, traj8, s)
    col29, t29 = check_for_replanning_util(traj2, traj9, s)
    col210, t210 = check_for_replanning_util(traj2, traj10, s)
    col34, t34 = check_for_replanning_util(traj3, traj4, s)
    col35, t35 = check_for_replanning_util(traj3, traj5, s)
    col36, t36 = check_for_replanning_util(traj3, traj6, s)
    col37, t37 = check_for_replanning_util(traj3, traj7, s)
    col38, t38 = check_for_replanning_util(traj3, traj8, s)
    col39, t39 = check_for_replanning_util(traj3, traj9, s)
    col310, t310 = check_for_replanning_util(traj3, traj10, s)
    col45, t45 = check_for_replanning_util(traj4, traj5, s)
    col46, t46 = check_for_replanning_util(traj4, traj6, s)
    col47, t47 = check_for_replanning_util(traj4, traj7, s)
    col48, t48 = check_for_replanning_util(traj4, traj8, s)
    col49, t49 = check_for_replanning_util(traj4, traj9, s)
    col410, t410 = check_for_replanning_util(traj4, traj10, s)
    col56, t56 = check_for_replanning_util(traj5, traj6, s)
    col57, t57 = check_for_replanning_util(traj5, traj7, s)
    col58, t58 = check_for_replanning_util(traj5, traj8, s)
    col59, t59 = check_for_replanning_util(traj5, traj9, s)
    col510, t510 = check_for_replanning_util(traj5, traj10, s)
    col67, t67 = check_for_replanning_util(traj6, traj7, s)
    col68, t68 = check_for_replanning_util(traj6, traj8, s)
    col69, t69 = check_for_replanning_util(traj6, traj9, s)
    col610, t610 = check_for_replanning_util(traj6, traj10, s)
    col78, t78 = check_for_replanning_util(traj7, traj8, s)
    col79, t79 = check_for_replanning_util(traj7, traj9, s)
    col710, t710 = check_for_replanning_util(traj7, traj10, s)
    col89, t89 = check_for_replanning_util(traj8, traj9, s)
    col810, t810 = check_for_replanning_util(traj8, traj10, s)
    col910, t910 = check_for_replanning_util(traj9, traj10, s)
    col = [False, False, False,False,False, False, False,False,False,False]
    t = [-1, -1, -1,-1,-1, -1, -1,-1,-1,-1]

    #1
    if col12 or col13 or col14 or col15 or col16 or col17 or col18 or col19 or col110:
        col[0] = True
        if not col13 and not col14 and not col15 and not col16 and not col17 and not col18 and not col19 and not col110:
            t[0] = t12
        elif not col12 and not col14 and not col15 and not col16 and not col17 and not col18 and not col19 and not col110:
            t[0] = t13
        elif not col13 and not col12 and not col15 and not col16 and not col17 and not col18 and not col19 and not col110:
            t[0] = t14
        elif not col13 and not col12 and not col14 and not col16 and not col17 and not col18 and not col19 and not col110:
            t[0] = t15
        elif not col13 and not col12 and not col15 and not col14 and not col17 and not col18 and not col19 and not col110:
            t[0] = t16
        elif not col13 and not col12 and not col15 and not col14 and not col16 and not col18 and not col19 and not col110:
            t[0] = t17
        elif not col13 and not col12 and not col15 and not col14 and not col16 and not col17 and not col19 and not col110:
            t[0] = t18
        elif not col13 and not col12 and not col15 and not col14 and not col16 and not col17 and not col18 and not col110:
            t[0] = t19
        elif not col13 and not col12 and not col15 and not col14 and not col16 and not col17 and not col18 and not col19:
            t[0] = t110
        else:
            t[0] = min(t12, t13, t14 , t15, t16, t17 , t18, t19, t110)

    #2
    if col12 or col23 or col24 or col25 or col26 or col27 or col28 or col29 or col210:
        col[1] = True
        if not col23 and not col24 and not col25 and not col26 and not col27 and not col28 and not col29 and not col210:
            t[1] = t12
        elif not col12 and not col24 and not col25 and not col26 and not col27 and not col28 and not col29 and not col210:
            t[1] = t23
        elif not col23 and not col12 and not col25 and not col26 and not col27 and not col28 and not col29 and not col210:
            t[1] = t24
        elif not col23 and not col12 and not col24 and not col26 and not col27 and not col28 and not col29 and not col210:
            t[1] = t25
        elif not col23 and not col12 and not col25 and not col24 and not col27 and not col28 and not col29 and not col210:
            t[1] = t26
        elif not col23 and not col12 and not col25 and not col24 and not col26 and not col28 and not col29 and not col210:
            t[1] = t27
        elif not col23 and not col12 and not col25 and not col24 and not col26 and not col27 and not col29 and not col210:
            t[1] = t28
        elif not col23 and not col12 and not col25 and not col24 and not col26 and not col27 and not col28 and not col210:
            t[1] = t29
        elif not col23 and not col12 and not col25 and not col24 and not col26 and not col27 and not col28 and not col29:
            t[1] = t210
        else:
            t[1] = min(t12, t23, t24 , t25, t26, t27 , t28, t29, t210)

    #3##
    if col23 or col13 or col34 or col35 or col36 or col37 or col38 or col39 or col310:
        col[2] = True
        if not col23 and not col34 and not col35 and not col36 and not col37 and not col38 and not col39 and not col310:
            t[2] = t13
        elif not col13 and not col34 and not col35 and not col36 and not col37 and not col38 and not col39 and not col310:
            t[2] = t23
        elif not col23 and not col13 and not col35 and not col36 and not col37 and not col38 and not col39 and not col310:
            t[2] = t34
        elif not col23 and not col34 and not col13 and not col36 and not col37 and not col38 and not col39 and not col310:
            t[2] = t35
        elif not col23 and not col34 and not col35 and not col13 and not col37 and not col38 and not col39 and not col310:
            t[2] = t36
        elif not col23 and not col34 and not col35 and not col36 and not col13 and not col38 and not col39 and not col310:
            t[2] = t37
        elif not col23 and not col34 and not col35 and not col36 and not col37 and not col13 and not col39 and not col310:
            t[2] = t38
        elif not col23 and not col34 and not col35 and not col36 and not col37 and not col38 and not col13 and not col310:
            t[2] = t39
        elif not col23 and not col34 and not col35 and not col36 and not col37 and not col38 and not col39 and not col13:
            t[2] = t310
        else:
            t[2] = min(t13, t23, t34 , t35, t36, t37 , t38, t39, t310)

    #4
    if col14 or col24 or col34 or col45 or col46 or col47 or col48 or col49 or col410:
        col[3] = True
        if not col24 and not col34 and not col45 and not col46 and not col47 and not col48 and not col49 and not col410:
            t[3] = t14
        elif not col14 and not col34 and not col45 and not col46 and not col47 and not col48 and not col49 and not col410:
            t[3] = t24
        elif not col24 and not col14 and not col45 and not col46 and not col47 and not col48 and not col49 and not col410:
            t[3] = t34
        elif not col24 and not col34 and not col14 and not col46 and not col47 and not col48 and not col49 and not col410:
            t[3] = t45
        elif not col24 and not col34 and not col45 and not col14 and not col47 and not col48 and not col49 and not col410:
            t[3] = t46
        elif not col24 and not col34 and not col45 and not col46 and not col14 and not col48 and not col49 and not col410:
            t[3] = t47
        elif not col24 and not col34 and not col45 and not col46 and not col47 and not col14 and not col49 and not col410:
            t[3] = t48
        elif not col24 and not col34 and not col45 and not col46 and not col47 and not col48 and not col14 and not col410:
            t[3] = t49
        elif not col24 and not col34 and not col45 and not col46 and not col47 and not col48 and not col49 and not col14:
            t[3] = t410
        else:
            t[3] = min(t14, t24, t34 , t45, t46, t47 , t48, t49, t410)

    #5
    if col15 or col25 or col35 or col45 or col56 or col57 or col58 or col59 or col510:
        col[4] = True
        if not col25 and not col35 and not col45 and not col56 and not col57 and not col58 and not col59 and not col510:
            t[4] = t15
        elif not col15 and not col35 and not col45 and not col56 and not col57 and not col58 and not col59 and not col510:
            t[4] = t25
        elif not col25 and not col15 and not col45 and not col56 and not col57 and not col58 and not col59 and not col510:
            t[4] = t35
        elif not col25 and not col35 and not col15 and not col56 and not col57 and not col58 and not col59 and not col510:
            t[4] = t45
        elif not col25 and not col35 and not col45 and not col15 and not col57 and not col58 and not col59 and not col510:
            t[4] = t56
        elif not col25 and not col35 and not col45 and not col56 and not col15 and not col58 and not col59 and not col510:
            t[4] = t57
        elif not col25 and not col35 and not col45 and not col56 and not col57 and not col15 and not col59 and not col510:
            t[4] = t58
        elif not col25 and not col35 and not col45 and not col56 and not col57 and not col58 and not col15 and not col510:
            t[4] = t59
        elif not col25 and not col35 and not col45 and not col56 and not col57 and not col58 and not col59 and not col15:
            t[4] = t510
        else:
            t[4] = min(t15, t25, t35 , t45, t56, t57 , t58, t59, t510)

    #6
    if col16 or col26 or col36 or col46 or col56 or col67 or col68 or col69 or col610:
        col[5] = True
        if not col26 and not col36 and not col46 and not col56 and not col67 and not col68 and not col69 and not col610:
            t[5] = t16
        elif not col16 and not col36 and not col46 and not col56 and not col67 and not col68 and not col69 and not col610:
            t[5] = t26
        elif not col26 and not col16 and not col46 and not col56 and not col67 and not col68 and not col69 and not col610:
            t[5] = t36
        elif not col26 and not col36 and not col16 and not col56 and not col67 and not col68 and not col69 and not col610:
            t[5] = t46
        elif not col26 and not col36 and not col46 and not col16 and not col67 and not col68 and not col69 and not col610:
            t[5] = t56
        elif not col26 and not col36 and not col46 and not col56 and not col16 and not col68 and not col69 and not col610:
            t[5] = t67
        elif not col26 and not col36 and not col46 and not col56 and not col67 and not col16 and not col69 and not col610:
            t[5] = t68
        elif not col26 and not col36 and not col46 and not col56 and not col67 and not col68 and not col16 and not col610:
            t[5] = t69
        elif not col26 and not col36 and not col46 and not col56 and not col67 and not col68 and not col69 and not col16:
            t[5] = t610
        else:
            t[5] = min(t16, t26, t36 , t46, t56, t67 , t68, t69, t610)

    #7
    if col17 or col27 or col37 or col47 or col57 or col67 or col78 or col79 or col710:
        col[6] = True
        if not col27 and not col37 and not col47 and not col57 and not col67 and not col78 and not col79 and not col710:
            t[6] = t17
        elif not col17 and not col37 and not col47 and not col57 and not col67 and not col78 and not col79 and not col710:
            t[6] = t27
        elif not col27 and not col17 and not col47 and not col57 and not col67 and not col78 and not col79 and not col710:
            t[6] = t37
        elif not col27 and not col37 and not col17 and not col57 and not col67 and not col78 and not col79 and not col710:
            t[6] = t47
        elif not col27 and not col37 and not col47 and not col17 and not col67 and not col78 and not col79 and not col710:
            t[6] = t57
        elif not col27 and not col37 and not col47 and not col57 and not col17 and not col78 and not col79 and not col710:
            t[6] = t67
        elif not col27 and not col37 and not col47 and not col57 and not col67 and not col17 and not col79 and not col710:
            t[6] = t78
        elif not col27 and not col37 and not col47 and not col57 and not col67 and not col78 and not col17 and not col710:
            t[6] = t79
        elif not col27 and not col37 and not col47 and not col57 and not col67 and not col78 and not col79 and not col17:
            t[6] = t710
        else:
            t[6] = min(t17, t27, t37 , t47, t57, t67 , t78, t79, t710)

    #8
    if col18 or col28 or col38 or col48 or col58 or col68 or col78 or col89 or col810:
        col[7] = True
        if not col28 and not col38 and not col48 and not col58 and not col68 and not col78 and not col89 and not col810:
            t[7] = t18
        elif not col18 and not col38 and not col48 and not col58 and not col68 and not col78 and not col89 and not col810:
            t[7] = t28
        elif not col28 and not col18 and not col48 and not col58 and not col68 and not col78 and not col89 and not col810:
            t[7] = t38
        elif not col28 and not col38 and not col18 and not col58 and not col68 and not col78 and not col89 and not col810:
            t[7] = t48
        elif not col28 and not col38 and not col48 and not col18 and not col68 and not col78 and not col89 and not col810:
            t[7] = t58
        elif not col28 and not col38 and not col48 and not col58 and not col18 and not col78 and not col89 and not col810:
            t[7] = t68
        elif not col28 and not col38 and not col48 and not col58 and not col68 and not col18 and not col89 and not col810:
            t[7] = t78
        elif not col28 and not col38 and not col48 and not col58 and not col68 and not col78 and not col18 and not col810:
            t[7] = t89
        elif not col28 and not col38 and not col48 and not col58 and not col68 and not col78 and not col89 and not col18:
            t[7] = t810
        else:
            t[7] = min(t17, t27, t37 , t47, t57, t67 , t78, t89, t810)

    #9
    if col19 or col29 or col39 or col49 or col59 or col69 or col79 or col89 or col910:
        col[8] = True
        if not col29 and not col39 and not col49 and not col59 and not col69 and not col79 and not col89 and not col910:
            t[8] = t19
        elif not col19 and not col39 and not col49 and not col59 and not col69 and not col79 and not col89 and not col910:
            t[8] = t29
        elif not col29 and not col19 and not col49 and not col59 and not col69 and not col79 and not col89 and not col910:
            t[8] = t39
        elif not col29 and not col39 and not col19 and not col59 and not col69 and not col79 and not col89 and not col910:
            t[8] = t49
        elif not col29 and not col39 and not col49 and not col19 and not col69 and not col79 and not col89 and not col910:
            t[8] = t59
        elif not col29 and not col39 and not col49 and not col59 and not col19 and not col79 and not col89 and not col910:
            t[8] = t69
        elif not col29 and not col39 and not col49 and not col59 and not col69 and not col19 and not col89 and not col910:
            t[8] = t79
        elif not col29 and not col39 and not col49 and not col59 and not col69 and not col79 and not col19 and not col910:
            t[8] = t89
        elif not col29 and not col39 and not col49 and not col59 and not col69 and not col79 and not col89 and not col19:
            t[8] = t910
        else:
            t[8] = min(t19, t29, t39 , t49, t59, t69 , t79, t89, t910)

    #10
    if col110 or col210 or col310 or col410 or col510 or col610 or col710 or col810 or col910:
        col[9] = True
        if not col210 and not col310 and not col410 and not col510 and not col610 and not col710 and not col810 and not col910:
            t[9] = t110
        elif not col110 and not col310 and not col410 and not col510 and not col610 and not col710 and not col810 and not col910:
            t[9] = t210
        elif not col210 and not col110 and not col410 and not col510 and not col610 and not col710 and not col810 and not col910:
            t[9] = t310
        elif not col210 and not col310 and not col110 and not col510 and not col610 and not col710 and not col810 and not col910:
            t[9] = t410
        elif not col210 and not col310 and not col410 and not col110 and not col610 and not col710 and not col810 and not col910:
            t[9] = t510
        elif not col210 and not col310 and not col410 and not col510 and not col110 and not col710 and not col810 and not col910:
            t[9] = t610
        elif not col210 and not col310 and not col410 and not col510 and not col610 and not col110 and not col810 and not col910:
            t[9] = t710
        elif not col210 and not col310 and not col410 and not col510 and not col610 and not col710 and not col110 and not col910:
            t[9] = t810
        elif not col210 and not col310 and not col410 and not col510 and not col610 and not col710 and not col810 and not col110:
            t[9] = t910
        else:
            t[9] = min(t110, t210, t310 , t410, t510, t610 , t710, t810, t910)

    return col, t
def main():
    import os
    dir = "C:/Users/zakar/Desktop/RRT/Output"
    for f in os.listdir(dir):
        os.remove(os.path.join(dir, f))
    if not os.path.exists('Output'):
        os.makedirs('Output')
    # starting position (x, y)
    start1,start2,start3,start4,start5,start6,start7,start8,start9,start10,goal1,goal2,goal3,goal4,goal5,goal6,goal7,goal8,goal9,goal10,s= call_start_goal_s()
    # Initial Planning
    rrt_star1 = RRTStar(start1, goal1, s=s)
    traj1 = rrt_star1.plan("Output/plan1.txt", "Output/explored1.png")
    rrt_star2 = RRTStar(start2, goal2, s=s)
    traj2 = rrt_star2.plan("Output/plan2.txt", "Output/explored2.png")
    rrt_star3 = RRTStar(start3, goal3, s=s)
    traj3 = rrt_star3.plan("Output/plan3.txt", "Output/explored3.png")
    rrt_star4 = RRTStar(start4, goal4, s=s)
    traj4 = rrt_star4.plan("Output/plan4.txt", "Output/explored4.png")
    rrt_star5 = RRTStar(start5, goal5, s=s)
    traj5 = rrt_star5.plan("Output/plan5.txt", "Output/explored5.png")
    rrt_star6 = RRTStar(start6, goal6, s=s)
    traj6 = rrt_star6.plan("Output/plan6.txt", "Output/explored6.png")
    rrt_star7 = RRTStar(start7, goal7, s=s)
    traj7 = rrt_star7.plan("Output/plan7.txt", "Output/explored7.png")
    rrt_star8 = RRTStar(start8, goal8, s=s)
    traj8 = rrt_star8.plan("Output/plan8.txt", "Output/explored8.png")
    rrt_star9 = RRTStar(start9, goal9, s=s)
    traj9 = rrt_star9.plan("Output/plan9.txt", "Output/explored9.png")
    rrt_star10 = RRTStar(start10, goal10, s=s)
    traj10 = rrt_star10.plan("Output/plan10.txt", "Output/explored10.png")
    # Plot planned trajectories
    fig, ax = plt.subplots()
    ax.set_xlim([0, 100])
    ax.set_ylim([0, 100])
    ax.plot(traj1[0], traj1[1], color='g', linewidth=1)
    ax.plot(traj2[0], traj2[1], color='g', linewidth=1)
    ax.plot(traj3[0], traj3[1], color='g', linewidth=1)
    ax.plot(traj4[0], traj4[1], color='g', linewidth=1)
    ax.plot(traj5[0], traj5[1], color='g', linewidth=1)
    ax.plot(traj6[0], traj6[1], color='g', linewidth=1)
    ax.plot(traj7[0], traj7[1], color='g', linewidth=1)
    ax.plot(traj8[0], traj8[1], color='g', linewidth=1)
    ax.plot(traj9[0], traj9[1], color='g', linewidth=1)
    ax.plot(traj10[0], traj10[1], color='g', linewidth=1)
    plt.savefig("Output/Plan0.png")
    replan = [True, True, True,True,True, True, True,True,True, True]
    traj_all = [traj1, traj2, traj3,traj4,traj5, traj6, traj7,traj8,traj9, traj10]
    rrt_star = [rrt_star1, rrt_star2, rrt_star3,rrt_star4,rrt_star5, rrt_star6, rrt_star7,rrt_star8,rrt_star9, rrt_star10]
    flag = True
    for i in range(10):
        print('iteration : {}'.format(i))
        col, t = check_for_replanning(traj1, traj2, traj3,traj4,traj5,traj6,traj7,traj8,traj9,traj10, s, flag)
        flag = False
        print("Collision: ", col)
        pd1 = float('inf')
        pd2 = float('inf')
        pd3 = float('inf')
        pd4 = float('inf')
        pd5 = float('inf')
        pd6 = float('inf')
        pd7 = float('inf')
        pd8 = float('inf')
        pd9 = float('inf')
        pd10 = float('inf')
        #1
        if col[0] and replan[0]:
            new_traj1 = rrt_star1.replan([traj2, traj3,traj4,traj5, traj6,traj7,traj8, traj9,traj10], t[0], "Output/replanned" + str(i) + "_1.txt",
                                         "Output/re_explored" + str(i) + "_1.png")
            pd1 = (len(new_traj1[0]) - len(traj1[0])) / float(len(traj1[0])) * 100
        #2
        if col[1] and replan[1]:
            new_traj2 = rrt_star2.replan([traj1, traj3,traj4,traj5, traj6,traj7,traj8, traj9,traj10], t[1], "Output/replanned" + str(i) + "_2.txt",
                                         "Output/re_explored" + str(i) + "_2.png")
            pd2 = (len(new_traj2[0]) - len(traj2[0])) / float(len(traj2[0])) * 100
        #3
        if col[2] and replan[2]:
            new_traj3 = rrt_star3.replan([traj2, traj1,traj4,traj5, traj6,traj7,traj8, traj9,traj10], t[2], "Output/replanned" + str(i) + "_3.txt",
                                         "Output/re_explored" + str(i) + "_3.png")
            pd3 = (len(new_traj3[0]) - len(traj3[0])) / float(len(traj3[0])) * 100
        #4
        if col[3] and replan[3]:
            new_traj4 = rrt_star4.replan([traj2, traj3,traj1,traj5, traj6,traj7,traj8, traj9,traj10], t[3], "Output/replanned" + str(i) + "_4.txt",
                                         "Output/re_explored" + str(i) + "_4.png")
            pd4 = (len(new_traj4[0]) - len(traj4[0])) / float(len(traj4[0])) * 100
        #5
        if col[4] and replan[4]:
            new_traj5 = rrt_star5.replan([traj2, traj3,traj4,traj1, traj6,traj7,traj8, traj9,traj10], t[4], "Output/replanned" + str(i) + "_5.txt",
                                         "Output/re_explored" + str(i) + "_5.png")
            pd5 = (len(new_traj5[0]) - len(traj5[0])) / float(len(traj5[0])) * 100
        #6
        if col[5] and replan[5]:
            new_traj6 = rrt_star6.replan([traj2, traj3,traj4,traj1, traj5,traj7,traj8, traj9,traj10], t[5], "Output/replanned" + str(i) + "_6.txt",
                                         "Output/re_explored" + str(i) + "_6.png")
            pd6 = (len(new_traj6[0]) - len(traj6[0])) / float(len(traj6[0])) * 100
        #7
        if col[6] and replan[6]:
            new_traj7 = rrt_star7.replan([traj2, traj3,traj4,traj1, traj6,traj5,traj8, traj9,traj10], t[6], "Output/replanned" + str(i) + "_7.txt",
                                         "Output/re_explored" + str(i) + "_7.png")
            pd7 = (len(new_traj7[0]) - len(traj7[0])) / float(len(traj7[0])) * 100
        #8
        if col[7] and replan[7]:
            new_traj8 = rrt_star8.replan([traj2, traj3,traj4,traj1, traj6,traj7,traj5, traj9,traj10], t[7], "Output/replanned" + str(i) + "_8.txt",
                                         "Output/re_explored" + str(i) + "_8.png")
            pd8 = (len(new_traj8[0]) - len(traj8[0])) / float(len(traj8[0])) * 100
        #9
        if col[8] and replan[8]:
            new_traj9 = rrt_star9.replan([traj2, traj3,traj4,traj1, traj6,traj7,traj8, traj5,traj10], t[8], "Output/replanned" + str(i) + "_9.txt",
                                         "Output/re_explored" + str(i) + "_9.png")
            pd9 = (len(new_traj9[0]) - len(traj9[0])) / float(len(traj9[0])) * 100
        #10
        if col[9] and replan[9]:
            new_traj10 = rrt_star10.replan([traj2, traj3,traj4,traj1, traj6,traj7,traj8, traj9,traj5], t[9], "Output/replanned" + str(i) + "_10.txt",
                                         "Output/re_explored" + str(i) + "_5.png")
            pd10 = (len(new_traj10[0]) - len(traj10[0])) / float(len(traj10[0])) * 100
        m = min(pd1, pd2, pd3,pd4,pd5,pd6,pd7,pd8,pd9,pd10)
        if m == float('inf'):
            print("Final trajectories found at iteration: " + str(i + 1))
            break
        if m == pd1:
            print("Trajectory 1 changed at iteration " + str(i + 1))
            traj1 = new_traj1
            replan[0] = False
        elif m == pd2:
            print("Trajectory 2 changed at iteration " + str(i + 1))
            traj2 = new_traj2
            replan[1] = False
        elif m == pd3:
            print("Trajectory 3 changed at iteration " + str(i + 1))
            traj3 = new_traj3
            replan[2] = False
        elif m == pd4:
            print("Trajectory 4 changed at iteration " + str(i + 1))
            traj4 = new_traj4
            replan[3] = False
        elif m == pd5:
            print("Trajectory 5 changed at iteration " + str(i + 1))
            traj5 = new_traj5
            replan[4] = False
        elif m == pd6:
            print("Trajectory 6 changed at iteration " + str(i + 1))
            traj6 = new_traj6
            replan[5] = False
        elif m == pd7:
            print("Trajectory 7 changed at iteration " + str(i + 1))
            traj7 = new_traj7
            replan[6] = False
        elif m == pd8:
            print("Trajectory 8 changed at iteration " + str(i + 1))
            traj8 = new_traj8
            replan[7] = False
        elif m == pd9:
            print("Trajectory 9 changed at iteration " + str(i + 1))
            traj9 = new_traj9
            replan[8] = False
        else:
            print("Trajectory 10 changed at iteration " + str(i + 1))
            traj10 = new_traj10
            replan[9] = False
        fig6, ax6 = plt.subplots()
        ax6.set_xlim([0, 100])
        ax6.set_ylim([0, 100])
        ax6.plot(traj1[0], traj1[1], color='g', linewidth=1)
        ax6.plot(traj2[0], traj2[1], color='g', linewidth=1)
        ax6.plot(traj3[0], traj3[1], color='g', linewidth=1)
        ax6.plot(traj4[0], traj4[1], color='g', linewidth=1)
        ax6.plot(traj5[0], traj5[1], color='g', linewidth=1)
        ax6.plot(traj6[0], traj6[1], color='g', linewidth=1)
        ax6.plot(traj7[0], traj7[1], color='g', linewidth=1)
        ax6.plot(traj8[0], traj8[1], color='g', linewidth=1)
        ax6.plot(traj9[0], traj9[1], color='g', linewidth=1)
        ax6.plot(traj10[0], traj10[1], color='g', linewidth=1)
        plt.savefig("Output/Plan" + str(i + 1) + ".png")
    fig2, ax2 = plt.subplots()
    ax2.set_xlim([0, 100])
    ax2.set_ylim([0, 100])

    obs1, obs2, obs3, obs4, obs5, obs6, obs7, obs8, obs9 = call_obs()
    ax.add_patch(obs1)
    ax.add_patch(obs2)
    ax.add_patch(obs3)
    ax.add_patch(obs4)
    ax.add_patch(obs5)
    ax.add_patch(obs6)
    ax.add_patch(obs7)
    ax.add_patch(obs8)
    ax.add_patch(obs9)

    tr = [traj1, traj2, traj3,traj4,traj5, traj6, traj7,traj8,traj9, traj10]
    # save data in txt file
    out1 = traj1.T
    if os.path.exists("Output/final_path1.txt"):
        os.remove("Output/final_path1.txt")
    final1 = open("Output/final_path1.txt", "a")
    for i in range(len(out1)):
        np.savetxt(final1, out1[i], fmt="%s", newline=' ')
        final1.write("\n")
    out2 = traj2.T
    if os.path.exists("Output/final_path2.txt"):
        os.remove("Output/final_path2.txt")
    final2 = open("Output/final_path2.txt", "a")
    for i in range(len(out2)):
        np.savetxt(final2, out2[i], fmt="%s", newline=' ')
        final2.write("\n")
    out3 = traj3.T
    if os.path.exists("Output/final_path3.txt"):
        os.remove("Output/final_path3.txt")
    final3 = open("Output/final_path3.txt", "a")
    for i in range(len(out3)):
        np.savetxt(final3, out3[i], fmt="%s", newline=' ')
        final3.write("\n")
    out4 = traj4.T
    if os.path.exists("Output/final_path4.txt"):
        os.remove("Output/final_path4.txt")
    final4 = open("Output/final_path4.txt", "a")
    for i in range(len(out4)):
        np.savetxt(final4, out4[i], fmt="%s", newline=' ')
        final4.write("\n")
    out5 = traj5.T
    if os.path.exists("Output/final_path5.txt"):
        os.remove("Output/final_path5.txt")
    final5 = open("Output/final_path5.txt", "a")
    for i in range(len(out5)):
        np.savetxt(final5, out5[i], fmt="%s", newline=' ')
        final5.write("\n")
    out6 = traj6.T
    if os.path.exists("Output/final_path6.txt"):
        os.remove("Output/final_path6.txt")
    final6 = open("Output/final_path6.txt", "a")
    for i in range(len(out6)):
        np.savetxt(final6, out6[i], fmt="%s", newline=' ')
        final6.write("\n")
    out7 = traj7.T
    if os.path.exists("Output/final_path7.txt"):
        os.remove("Output/final_path7.txt")
    final7 = open("Output/final_path7.txt", "a")
    for i in range(len(out7)):
        np.savetxt(final7, out7[i], fmt="%s", newline=' ')
        final7.write("\n")
    out8 = traj8.T
    if os.path.exists("Output/final_path8.txt"):
        os.remove("Output/final_path8.txt")
    final8 = open("Output/final_path8.txt", "a")
    for i in range(len(out8)):
        np.savetxt(final8, out8[i], fmt="%s", newline=' ')
        final8.write("\n")
    out9 = traj9.T
    if os.path.exists("Output/final_path9.txt"):
        os.remove("Output/final_path9.txt")
    final9 = open("Output/final_path9.txt", "a")
    for i in range(len(out9)):
        np.savetxt(final9, out9[i], fmt="%s", newline=' ')
        final9.write("\n")
    out10 = traj10.T
    if os.path.exists("Output/final_path10.txt"):
        os.remove("Output/final_path10.txt")
    final10 = open("Output/final_path10.txt", "a")
    for i in range(len(out10)):
        np.savetxt(final10, out10[i], fmt="%s", newline=' ')
        final10.write("\n")
    plt.savefig("Output/final_path.png")
    plt.show()
if __name__ == "__main__":
    main()

