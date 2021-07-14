import os
import math
import random
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev
plt.ion()
from call import *

class RRTStar:
    class node:
        def __init__(self, x, y, t=0, cost = 0):
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
        self.upper_lim_x = 20
        self.upper_lim_y = 20 
        self.neigh_dist = 0.6
        self.vel = 0.2    # robot speed 
        self.r = 0.177 # robot radius
        self.c = 0.4     # robot clearance
        self.s = s
        self.thresh = self.c + self.r
        self.nodes_at_t = {}
        self.other_traj = []

    def check_collision(self, node):

        #node.pretty_print()
        ret = False
        

        if node.x - self.thresh < self.lower_lim_x:
            ret = True
        elif node.x + self.thresh > self.upper_lim_x:
            ret = True
        elif node.y - self.thresh < self.lower_lim_y:
            ret = True
        elif node.y + self.thresh > self.upper_lim_y:
            ret = True

        if node.x > 4 - self.thresh and node.x < 6 + self.thresh and node.y > 5 - self.thresh and node.y < 6 + self.thresh:
            ret = True
        if node.x > 9 - self.thresh and node.x < 10 + self.thresh and node.y > 3 - self.thresh and node.y < 4.5 + self.thresh:
            ret = True
        if node.x > 7 - self.thresh and node.x < 8 + self.thresh and node.y > 18 - self.thresh and node.y < 19 + self.thresh:
            ret = True
        if node.x > 5 - self.thresh and node.x < 6 + self.thresh and node.y > 13.5 - self.thresh and node.y < 15 + self.thresh:
            ret = True
        if node.x > 13.5 - self.thresh and node.x < 15.5 + self.thresh and node.y > 10 - self.thresh and node.y < 12 + self.thresh:
            ret = True
        if node.x > 14 - self.thresh and node.x < 18 + self.thresh and node.y > 2.5 - self.thresh and node.y < 5 + self.thresh:
            ret = True
        if node.x > 8 - self.thresh and node.x < 10 + self.thresh and node.y > 12.5 - self.thresh and node.y < 14 + self.thresh:
            ret = True
        if node.x > 1 - self.thresh and node.x < 2.5 + self.thresh and node.y > 8 - self.thresh and node.y < 11 + self.thresh:
            ret = True
        if node.x > 17.5 - self.thresh and node.x < 18.5 + self.thresh and node.y > 12.5 - self.thresh and node.y < 15 + self.thresh:
            ret = True

        for traj in self.other_traj:
            t = node.t
            if node.t > len(traj[0])-1:
                t = len(traj[0])-1

            if np.sqrt((node.x - traj[0][t])**2 + (node.y - traj[1][t])**2) < self.s:
                ret = True
                

        return ret

    def goal_check(self, node):
        if self.get_dist(node, self.goal_node) < 0.6:
            return True

        return False

    def get_random_node(self):
        x = random.randint(1, 20)
        y = random.randint(1, 20)

        new_node = self.node(x, y)

        return new_node

    def get_dist(self, node1, node2):
        return math.sqrt((node1.x -node2.x)**2 + (node1.y - node2.y)**2)

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

        theta = np.arctan2((dest_y-par_y), (dest_x-par_x))

        count = 0
        x_path = []
        y_path = []
        x = par_x
        y = par_y
        dist = 0

        while(count < 10): 
            dx = 0.1 * math.cos(theta) * self.vel
            dy = 0.1 * math.sin(theta) * self.vel
            x = x + dx
            y = y + dy

            dist = dist + np.sqrt(dx**2 + dy**2)
            
            if self.check_collision(self.node(x, y, t=tm+count+1)):
                return None

            x_path.append(x)
            y_path.append(y)
            count = count + 1

        new_node = self.node(x, y, t=parent.t+10)
        new_node.parent = parent
        new_node.x_path = x_path
        new_node.y_path = y_path
        new_node.cost = parent.cost + dist
        return new_node

    def add_to_nodes_dict(self, new_node, index):
        t = new_node.t
        nodes = self.nodes_at_t.get(t)

        if nodes == None:
            self.nodes_at_t.update({t:[index]})

        else:
            nodes.append(index)
            self.nodes_at_t.update({t:nodes})


    def get_path(self, parent, child):
        dist = self.get_dist(parent, child)

        if (dist*10) % self.vel == 0:
            max_count = (dist*10)/self.vel
        else:
            max_count = (dist*10)/self.vel + 1

        par_x = parent.x
        par_y = parent.y
        t = parent.t

        child_x = child.x
        child_y = child.y

        theta = np.arctan2((child_y-par_y), (child_x-par_x))

        count = 0
        x_path = []
        y_path = []
        x = par_x
        y = par_y
        dist = 0

        if max_count == 0:
            return None, None

        while(count < max_count): 
            dx = 0.1 * math.cos(theta) * self.vel
            dy = 0.1 * math.sin(theta) * self.vel
            x = x + dx
            y = y + dy

            dist = dist + np.sqrt(dx**2 + dy**2)
            
            if self.check_collision(self.node(x, y, t=t+count+1)):
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
                idx = idx-1

    def propagate_cost_to_leaves(self, parent):
        for i, node in enumerate(self.nodes):
            if node.parent == parent:
                dist = self.get_dist(parent, node)
                node.cost = parent.cost + dist
                node.t = parent.t + len(node.x_path)
                if self.check_collision(node):
                    del self.nodes[i]
                    self.deleteAllChildren(node)
                    i = i-1
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
        if(cur_node.parent == None):
            return np.asarray([cur_node.x]), np.asarray([cur_node.y]), np.asarray([cur_node.t]), np.asarray([cur_node.x]), np.asarray([cur_node.x])

        x, y, t, path_x, path_y = self.backtrace(cur_node.parent)

        x_s = np.hstack((x, cur_node.x))
        y_s = np.hstack((y, cur_node.y))
        t_s = np.hstack((t, cur_node.t))
        path_x = np.hstack((path_x, cur_node.x_path))
        path_y = np.hstack((path_y, cur_node.y_path))

        return x_s, y_s, t_s, path_x, path_y

    def smooth_path(self, res, ax, text_name, img_name):
        t = res.t

        # backtrace path 
        x_s, y_s, t_s, path_x, path_y = self.backtrace(res)

        step = 5*float(1/float(t))
        m = len(x_s)

        # Path smoothing
        m = m - math.sqrt(5*m)
        tck, u = splprep([x_s, y_s], s=m)
        u_s = np.arange(0, 1.01, step)
        new_points = splev(u_s, tck)
        new_points = np.asarray(new_points)

        # Plot both trajectories
        ax.plot(x_s, y_s, color = 'r', linewidth = 1.5)
        ax.plot(new_points[0], new_points[1], label="S", color = 'c', linewidth = 1.5)

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
            cir_node = plt.Circle((n.x, n.y), 0.02, fill=True, color = 'r')
            ax.add_patch(cir_node)
            ax.plot(n.x_path, n.y_path, color = 'g', linewidth = 1)

        for traj in self.other_traj:
            ax.plot(traj[0], traj[1], color = 'b', linewidth = 1)


    def plan(self, text_name, img_name, replan = False):
        if self.check_collision(self.start_node):
            print("Start node inside obstacle")
            exit()

        if self.check_collision(self.goal_node):
            print("Goal node inside obstacle")
            exit()

        fig, ax = plt.subplots()
        ax.set_xlim([0,20])
        ax.set_ylim([0,20])

        cir_start = plt.Circle((self.start_node.x, self.start_node.y), 0.18, fill=True, color = 'm')
        cir_goal = plt.Circle((self.goal_node.x, self.goal_node.y), 0.18, fill=True, color = 'b')

        ax.add_patch(cir_start)
        ax.add_patch(cir_goal)

        obs1 = plt.Rectangle((4,5),2,1,fill=True, color='k')
        ax.add_patch(obs1)
        obs2 = plt.Rectangle((9,3),1,1.5,fill=True, color='k')
        ax.add_patch(obs2)
        obs3 = plt.Rectangle((7,18),1,1,fill=True, color='k')
        ax.add_patch(obs3)
        obs4 = plt.Rectangle((5,13.5),1,1.5,fill=True, color='k')
        ax.add_patch(obs4)
        obs5 = plt.Rectangle((13.5,10),2,2,fill=True, color='k')
        ax.add_patch(obs5)
        obs6 = plt.Rectangle((14.5,2.5),4,2.5,fill=True, color='k')
        ax.add_patch(obs6)
        obs7 = plt.Rectangle((8,12.5),2,1.5,fill=True, color='k')
        ax.add_patch(obs7)
        obs8 = plt.Rectangle((1,8),1.5,3,fill=True, color='k')
        ax.add_patch(obs8)
        obs9 = plt.Rectangle((17.5,12.5),1,2.5,fill=True, color='k')
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
            index = len(self.nodes)-1
            self.add_to_nodes_dict(new_node, index)

            cir_node = plt.Circle((new_node.x, new_node.y), 0.02, fill=True, color = 'r')
            ax.add_patch(cir_node)
            ax.plot(new_node.x_path, new_node.y_path, color = 'g', linewidth = 1)

            for i in range(len(new_path_x)):
                ax.plot(new_path_x[i], new_path_y[i], color = 'g', linewidth = 1)

            plt.pause(0.1)

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
                self.nodes_at_t.update({key:[]})

        indx = np.asarray(indx, dtype='int')

        for idx in sorted(indx, reverse=True):
            del self.nodes[idx]

    def replan(self, trajs, t, text_name, img_name):
        self.other_traj = trajs

        self.prune(t)
        traj = self.plan(text_name, img_name, replan = True)

        return traj
def check_for_replanning_util(traj1, traj2, s):
    l = min(len(traj1[0]), len(traj2[0]))
    L = max(len(traj1[0]), len(traj2[0]))
    col = False
    for i in range(l):
        dist = np.sqrt((traj1[0][i]-traj2[0][i])**2 + (traj1[1][i]-traj2[1][i])**2)
        # print(dist)
        if dist < (s):
            print("Collision detected at: " + str(i) + " at distance: "+ str(dist))
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
                dist = np.sqrt((traj1[0][l-1]-traj2[0][i])**2 + (traj1[1][l-1]-traj2[1][i])**2)
            else:
                dist = np.sqrt((traj1[0][i]-traj2[0][l-1])**2 + (traj1[1][i]-traj2[1][l-1])**2)
            if dist < s:
                print("Collision detected at: " + str(i))
                # print(dist)
                col = True
                step = i
                break
    return col, step
def check_for_replanning(traj1, traj2, traj3, s, flag):
    if not flag:
        s = s - 0.1

    col12, t12 = check_for_replanning_util(traj1, traj2, s)
    col23, t23 = check_for_replanning_util(traj2, traj3, s)
    col31, t31 = check_for_replanning_util(traj1, traj3, s)

    col = [False, False, False]
    t = [-1, -1, -1]

    if col12 or col31:
        col[0] = True
        if not col12:
            t[0] = t31
        elif not col31:
            t[0] = t12
        else:
            t[0] = min(t12, t31)

    if col12 or col23:
        col[1] = True
        if not col12:
            t[1] = t23
        elif not col23:
            t[1] = t12
        else:
            t[1] = min(t12, t23)

    if col23 or col31:
        col[2] = True
        if not col23:
            t[2] = t31
        elif not col31:
            t[2] = t23
        else:
            t[2] = min(t23, t31)

    return col, t
def main():
    delete_outputs()
    start1 = [3, 3]
    start2 = [15, 18]
    start3 = [3, 16]
    goal1 = [8, 8]
    goal2 = [8, 8]
    goal3 = [8, 8]
    # safe distance
    s = 2#0.5
    # Initial Planning
    rrt_star1 = RRTStar(start1, goal1, s=s)
    traj1 = rrt_star1.plan("Output/plan1.txt", "Output/explored1.png")
    rrt_star2 = RRTStar(start2, goal2, s=s)
    traj2 = rrt_star2.plan("Output/plan2.txt", "Output/explored2.png")
    rrt_star3 = RRTStar(start3, goal3, s=s)
    traj3 = rrt_star3.plan("Output/plan3.txt", "Output/explored3.png")
    # Plot planned trajectories
    fig, ax = plt.subplots()
    ax.set_xlim([0,20])
    ax.set_ylim([0,20])
    ax.plot(traj1[0], traj1[1], color = 'b', linewidth = 1)
    ax.plot(traj2[0], traj2[1], color = 'r', linewidth = 1)
    ax.plot(traj3[0], traj3[1], color = 'g', linewidth = 1)
    plt.savefig("Output/Plan0.png")
    flag = True

    for i in range(3):
        print('iteration:{}'.format(i+1))
        col, t = check_for_replanning(traj1, traj2, traj3, s, flag)
        flag = False
        print("Collision: ", col)
        if col == [False,False,False]:
            print('no collision')
            print('end')
            break

        stock_pd_1=[float('inf'),float('inf'),float('inf')]
        stock_pd_2 = [float('inf'), float('inf'), float('inf')]
        stock_pd_3=[float('inf'),float('inf'),float('inf')]
        stock_new_traj1=[]
        stock_new_traj2 = []
        stock_new_traj3 = []
        replan=[True,True,True]
        m0=[]

        if col[0] and replan[0]:
            new_traj1 = rrt_star1.replan([traj2, traj3], t[0], "Output/replanned"+str(i)+"_1.txt", "Output/re_explored"+str(i)+"_1.png")
            pd1 = (len(new_traj1[0])-len(traj1[0]))/float(len(traj1[0]))*100
            stock_pd_1[i]=pd1
            stock_new_traj1.append(new_traj1)

        if col[1] and replan[1]:
            new_traj2 = rrt_star2.replan([traj1, traj3], t[1], "Output/replanned"+str(i)+"_2.txt", "Output/re_explored"+str(i)+"_2.png")
            pd2 = (len(new_traj2[0])-len(traj2[0]))/float(len(traj2[0]))*100
            stock_pd_2[i]=pd2
            stock_new_traj2.append(new_traj2)

        if col[2] and replan[2]:
            new_traj3 = rrt_star3.replan([traj1, traj2], t[2], "Output/replanned"+str(i)+"_3.txt", "Output/re_explored"+str(i)+"_3.png")
            pd3 = (len(new_traj3[0])-len(traj3[0]))/float(len(traj3[0]))*100
            stock_pd_3[i]=pd3
            stock_new_traj3.append(new_traj3)

        #iteration 1
        if i == 0:
            print('*********************** i = {} *********************** '.format(i+1))
            m = min(stock_pd_1[i], stock_pd_2[i], stock_pd_3[i])
            m0.append(m)
            if m==stock_pd_1[i]:
                print("Trajectory 1 changed at iteration " + str(i + 1))
                print('Agent1 desactivated')
                traj1 = stock_new_traj1[i]
                replan[0] = False
            if m==stock_pd_2[i]:
                print("Trajectory 2 changed at iteration " + str(i + 1))
                print('Agent2 desactivated')
                traj2 = stock_new_traj2[i]
                replan[1] = False
            else:
                print("Trajectory 3 changed at iteration " + str(i + 1))
                print('Agent3 desactivated')
                traj3 = stock_new_traj3[i]
                replan[2] = False

        #iteration 2 , 3
        if i != 0:
            print('*********************** i = {} *********************** '.format(i+1))
            if not(stock_pd_1[i]>=stock_pd_1[i-1]) or not(stock_pd_2[i]>=stock_pd_2[i-1]) or not(stock_pd_3[i]>=stock_pd_3[i-1]): #improvement
                print('improvement in iteration: {}'.format(i+1))
                m=min(stock_pd_1[i],stock_pd_2[i],stock_pd_3[i])
                m0.append(m)
                #choose trajectory
                if m == float('inf'):
                    print("No collision, Final trajectories found at iteration: " + str(i + 1))
                    print('END')
                    break
                if m == stock_pd_1[i]:
                    print("Trajectory 1 changed at iteration " + str(i + 1))
                    print('Agent1 desactivated')
                    traj1 = stock_new_traj1[i]
                    replan[0] = False
                if m == stock_pd_2[i]:
                    print("Trajectory 2 changed at iteration " + str(i + 1))
                    print('Agent2 desactivated')
                    traj2 = stock_new_traj2[i]
                    replan[1] = False
                if m == stock_pd_3[i]:
                    print("Trajectory 3 changed at iteration " + str(i + 1))
                    print('Agent3 desactivated')
                    traj3 = stock_new_traj3[i]
                    replan[2] = False

            #No improvement
            if (not(stock_pd_1[i]<stock_pd_1[i-1]) or not(stock_pd_2[i]<stock_pd_2[i-1]) or not(stock_pd_3[i]>stock_pd_3[i-1])) and not(col==[False , False, False]) :
                print('no improvement in iteration: {}'.format(i + 1))
                m1=m0[i-1]
                q=[stock_pd_1[i-1],stock_pd_2[i-1],stock_pd_3[i-1]]
                #get m
                if q[0]==m1:
                    m=min(q[1],q[2])
                    replan[0]=True
                    print('reactivate agent1')
                if q[1]==m1:
                    m=min(q[0],q[2])
                    replan[1]=True
                    print('reactivate agent2')
                if q[2]==m1:
                    m=min(q[0],q[1])
                    replan[2]=True
                    print('reactivate agent3')
                #choose trajectory
                if m== stock_pd_1[i-1]:
                    print("Trajectory 1 changed at iteration " + str(i))
                    print('Agent1 desactivated')
                    traj1 = stock_new_traj1[i-1]
                    replan[0] = False
                if m== stock_pd_2[i-1]:
                    print("Trajectory 2 changed at iteration " + str(i))
                    print('Agent2 desactivated')
                    traj2 = stock_new_traj2[i-1]
                    replan[1] = False
                if m== stock_pd_3[i-1]:
                    print("Trajectory 3 changed at iteration " + str(i))
                    print('Agent3 desactivated')
                    traj3 = stock_new_traj3[i-1]
                    replan[2] = False

        fig6, ax6 = plt.subplots()
        ax6.set_xlim([0,20])
        ax6.set_ylim([0,20])
        ax6.plot(traj1[0], traj1[1], color = 'b', linewidth = 1)
        ax6.plot(traj2[0], traj2[1], color = 'r', linewidth = 1)
        ax6.plot(traj3[0], traj3[1], color = 'g', linewidth = 1)
        plt.savefig("Output/Plan"+str(i+1)+".png")

    if not col==[False,False,False]:
        print('Rerun')
        main()

    fig2, ax2 = plt.subplots()
    ax2.set_xlim([0,20])
    ax2.set_ylim([0,20])

    obs1, obs2, obs3, obs4, obs5, obs6, obs7, obs8, obs9=call_obs_pareto1g()
    ax2.add_patch(obs1)
    ax2.add_patch(obs2)
    ax2.add_patch(obs3)
    ax2.add_patch(obs4)
    ax2.add_patch(obs5)
    ax2.add_patch(obs6)
    ax2.add_patch(obs7)
    ax2.add_patch(obs8)
    ax2.add_patch(obs9)
    tr = [traj1, traj2, traj3]
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

    plt.savefig("Output/final_path.png")
    plt.show()

if __name__ == "__main__":
    main()

