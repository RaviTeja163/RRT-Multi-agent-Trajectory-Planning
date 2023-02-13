import shutil
import numpy as np
import random
import math
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import os
from scipy.interpolate import splprep, splev

plt.ion()  # turns on interactive mode

class RRTStar:
    class node:
        def __init__(self, x, y, t=0, cost = 0):
            self.x = x      # position
            self.y = y
            self.t = t      # time
            self.cost = cost
            self.x_path = None
            self.y_path = None
            self.parent = None

    def __init__(self, start, goal, s):
        self.start_node = self.node(start[0], start[1])
        self.goal_node = self.node(goal[0], goal[1])
        self.nodes = [self.start_node]
        self.neighbor_node_dist = 0.6
        self.robot_clearance = 0.4
        self.safe_distance = s
        self.robot_radius = 0.177
        self.robot_velocity = 0.2 
        self.threshold = self.robot_clearance + self.robot_radius
        self.agent_trajs = []
        self.nodes_at_t = {}

    def get_distance(self, node1, node2):
        # calculate the distance between 2 nodes
        return math.sqrt((node1.x -node2.x)**2 + (node1.y - node2.y)**2)

    def check_collision(self, node):

        # checking boundaries
        if node.x - self.threshold < 0:
            return True
        elif node.x + self.threshold > 10:
            return True
        elif node.y - self.threshold < 0:
            return True
        elif node.y + self.threshold > 10:
            return True

        # checking collision with the rectangular obstacle
        if node.x > 4 - self.threshold and node.x < 6 + self.threshold and node.y > 3 - self.threshold and node.y < 7 + self.threshold:
            return True

        # checking inter-agent collisions
        for traj in self.agent_trajs:
            t = node.t
            if node.t > len(traj[0])-1:
                t = len(traj[0])-1

            if np.sqrt((node.x - traj[0][t])**2 + (node.y - traj[1][t])**2) < self.safe_distance:
                return True

        return False

    def check_for_goal(self, node):
        # check if the node is within the distance of 0.5 to the goal
        near_goal = False
        if self.get_distance(node, self.goal_node) < 0.5:
            near_goal = True

        return near_goal

    def get_random_node(self, goal_bias):

        # choose the goal with a probability of goal bias
        if np.random.random() < goal_bias:
            new_node = self.node(self.goal_node.x, self.goal_node.y)

        # Or generate a random point
        else:
            new_node = self.node(random.randint(1, 10), random.randint(1, 10))

        return new_node

    def get_nearest_node(self, rand_node, nodes):
        nearest_node_index = 0
        min_distance = float('inf')

        for i, node in enumerate(nodes):
            dist = self.get_distance(rand_node, node)
            if dist < min_distance:
                nearest_node_index = i
                min_distance = dist

        return nearest_node_index

    def step_ahead(self, parent, rand_node):
        par_x = parent.x
        par_y = parent.y

        rand_x = rand_node.x
        rand_y = rand_node.y

        theta = np.arctan2((rand_y-par_y), (rand_x-par_x))

        count = 0
        x_path = []
        y_path = []
        x = par_x
        y = par_y
        dist = 0

        while(count < 10): 
            dx = 0.1 * self.robot_velocity * math.cos(theta)
            dy = 0.1 * self.robot_velocity * math.sin(theta)
            x = x + dx
            y = y + dy

            dist = dist + np.sqrt(dx**2 + dy**2)
            
            if self.check_collision(self.node(x, y, t = parent.t + count + 1)):
                return None

            x_path.append(x)
            y_path.append(y)
            count = count + 1

        new_node = self.node(x, y, t = parent.t + 10)
        new_node.parent = parent
        new_node.x_path = x_path
        new_node.y_path = y_path
        new_node.cost = parent.cost + dist
        return new_node

    def add_nodes(self, new_node, index):
        t = new_node.t
        nodes = self.nodes_at_t.get(t)

        if nodes == None:
            self.nodes_at_t.update({t:[index]})

        else:
            nodes.append(index)
            self.nodes_at_t.update({t:nodes})


    def generate_path(self, parent, child):
        dist = self.get_distance(parent, child)

        if (dist*10) % self.robot_velocity == 0:
            max_count = (dist*10)/self.robot_velocity
        else:
            max_count = (dist*10)/self.robot_velocity + 1

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
            dx = 0.1 * self.robot_velocity * math.cos(theta)
            dy = 0.1 * self.robot_velocity * math.sin(theta)
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
        neighbor_indexes = []

        for i, node in enumerate(self.nodes):
            if self.get_distance(new_node, node) <= self.neighbor_node_dist:
                neighbor_indexes.append(i)

        return neighbor_indexes

    def set_parent(self, new_node, ngh_indx):
        for i in ngh_indx:
            dist = self.get_distance(new_node, self.nodes[i])
            if self.nodes[i].cost + dist < new_node.cost:
                x_path, y_path = self.generate_path(self.nodes[i], new_node)
                if x_path == None:
                    continue
                new_node.t = self.nodes[i].t + len(x_path)
                new_node.x_path = x_path
                new_node.y_path = y_path
                new_node.cost = self.nodes[i].cost + dist
                new_node.parent = self.nodes[i]

    def deleteChildNodes(self, parent):
        for idx, node in enumerate(self.nodes):
            if node.parent == parent:
                del self.nodes[idx]
                self.deleteChildNodes(node)
                idx = idx-1

    def update_cost(self, parent):
        for i, node in enumerate(self.nodes):
            if node.parent == parent:
                dist = self.get_distance(parent, node)
                node.cost = parent.cost + dist
                node.t = parent.t + len(node.x_path)
                if self.check_collision(node):
                    del self.nodes[i]
                    self.deleteChildNodes(node)
                    i = i-1
                else:
                    self.update_cost(self.node)

    def rewire(self, new_node, neighbor_indexes):
        new_path_x = []
        new_path_y = []

        for i in neighbor_indexes:
            dist = self.get_distance(new_node, self.nodes[i])
            if new_node.cost + dist < self.nodes[i].cost:
                x_path, y_path = self.generate_path(new_node, self.nodes[i])
                if x_path == None:
                    continue
                self.nodes[i].t = new_node.t + len(x_path)
                self.nodes[i].x_path = x_path
                self.nodes[i].y_path = y_path
                self.nodes[i].cost = new_node.cost + dist
                self.nodes[i].parent = new_node
                self.update_cost(self.nodes[i])
                new_path_x.append(x_path)
                new_path_y.append(y_path)

        return new_path_x, new_path_y

    def backtrack(self, cur_node):
        if(cur_node.parent == None):
            return np.asarray([cur_node.x]), np.asarray([cur_node.y]), np.asarray([cur_node.t]), np.asarray([cur_node.x]), np.asarray([cur_node.x])

        x, y, t, path_x, path_y = self.backtrack(cur_node.parent)

        x_s = np.hstack((x, cur_node.x))
        y_s = np.hstack((y, cur_node.y))
        t_s = np.hstack((t, cur_node.t))
        path_x = np.hstack((path_x, cur_node.x_path))
        path_y = np.hstack((path_y, cur_node.y_path))

        return x_s, y_s, t_s, path_x, path_y

    def smooth_path(self, res, ax, text_name, img_name):

        x_sm, y_sm, t_s, path_x, path_y = self.backtrack(res)
        ma = len(x_sm)
        step = 5*float(1/float(res.t))

        # B-spline smoothing of the path
        ma = ma - math.sqrt(2*ma)
        tk, w = splprep([x_sm, y_sm], s = ma)
        u_sm = np.arange(0, 1.01, step)
        smooth_points = splev(u_sm, tk)
        smooth_points = np.asarray(smooth_points)

        # Plotting both actual and smoothened trajectories
        ax.plot(x_sm, y_sm, color = 'r', linewidth = 1.5)
        ax.plot(smooth_points[0], smooth_points[1], label="S", color = 'c', linewidth = 1.5)

        plt.savefig(img_name)

        # save data in txt file
        op = smooth_points.T
        if os.path.exists(text_name):
            os.remove(text_name)
        w1 = open(text_name, "a")

        for i in range(len(op)):
            np.savetxt(w1, op[i], fmt="%s", newline=' ')
            w1.write("\n")

        return smooth_points

    def plot_init_traj(self, ax):
        count = 0
        for node in self.nodes:
            if count == 0:
                count += 1
                continue
            point_node = plt.Circle((node.x, node.y), 0.02, fill=True, color = 'r')
            ax.add_patch(point_node)
            ax.plot(node.x_path, node.y_path, color = 'g', linewidth = 1)

        for traj in self.agent_trajs:
            ax.plot(traj[0], traj[1], color = 'b', linewidth = 1)


    def plan(self, text_file, image_file, goal_bias, replan = False):
        # checking edge cases where start or goal position is inside the obstacle
        if self.check_collision(self.start_node) or self.check_collision(self.goal_node):
            print("Start/Goal node is inside the obstacle")
            exit()

        fig4, ax4 = plt.subplots()
        ax4.set_xlim([0,10])
        ax4.set_ylim([0,10])

        start_point = plt.Circle((self.start_node.x, self.start_node.y), 0.18, fill=True, color = 'm')
        goal_point = plt.Circle((self.goal_node.x, self.goal_node.y), 0.18, fill=True, color = 'b')

        ax4.add_patch(start_point)
        ax4.add_patch(goal_point)

        obstacle = plt.Rectangle((4, 3), 2, 4, fill = True, color = 'k')
        ax4.add_patch(obstacle)

        if replan:
            self.plot_init_traj(ax4)

        count = 0
        while (True):
            rand_node = self.get_random_node(goal_bias)

            nearest_node_index = self.get_nearest_node(rand_node, self.nodes)
            new_node = self.step_ahead(self.nodes[nearest_node_index], rand_node)

            if new_node == None:
                continue    # continue if there is a collision 

            neighbor_indexes = self.get_neighbours(new_node)
            self.set_parent(new_node, neighbor_indexes)
            new_path_x, new_path_y = self.rewire(new_node, neighbor_indexes)
            self.nodes.append(new_node)
            index = len(self.nodes)-1
            self.add_nodes(new_node, index)

            cir_node = plt.Circle((new_node.x, new_node.y), 0.02, fill=True, color = 'r')
            ax4.add_patch(cir_node)
            ax4.plot(new_node.x_path, new_node.y_path, color = 'y', linewidth = 1)

            plt.pause(0.01)

            for i in range(len(new_path_x)):
                ax4.plot(new_path_x[i], new_path_y[i], color = 'y', linewidth = 1)

            if self.check_for_goal(new_node):                
                print("Trajectory found: ("+ str(self.start_node.x) + "," + str(self.start_node.x) + ") - ("+ str(self.goal_node.x) + "," + str(self.goal_node.y) + ")")
                x_path, y_path = self.generate_path(new_node, self.goal_node)
                self.goal_node.parent = new_node
                self.goal_node.t = new_node.t + len(x_path)
                self.goal_node.x_path = x_path
                self.goal_node.y_path = y_path
                self.goal_node.cost = new_node.cost + self.get_distance(new_node, self.goal_node)
                new_node = self.goal_node
                break

            count = count + 1

        traj = self.smooth_path(new_node, ax4, text_file, image_file)

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

    def replan(self, trajs, t, text_name, img_name, goal_bias):
        self.agent_trajs = trajs

        self.prune(t)
        traj = self.plan(text_name, img_name, goal_bias, replan = True)

        return traj

def check_trajectory_collision(traj1, traj2, safe_dist):
    collision = False
    min_len = min(len(traj1[0]), len(traj2[0]))
    max_len = max(len(traj1[0]), len(traj2[0]))

    for i in range(min_len):
        distance = np.sqrt((traj1[0][i]-traj2[0][i])**2 + (traj1[1][i]-traj2[1][i])**2)
        
        # considered a collision if distance between trajectories is less than the safe distance
        if distance < (safe_dist):
            collision = True
            break

    step = -1
    if not collision:
        is_first_traj = False
        if len(traj1[0]) == min_len:
            is_first_traj = True
        for i in range(min_len, max_len):
            if is_first_traj:
                distance = np.sqrt((traj1[0][min_len-1]-traj2[0][i])**2 + (traj1[1][min_len-1]-traj2[1][i])**2)
            else:
                distance = np.sqrt((traj1[0][i]-traj2[0][min_len-1])**2 + (traj1[1][i]-traj2[1][min_len-1])**2)
            
            # considered a collision if distance between trajectories is less than the safe distance
            if distance < safe_dist:
                step = i
                collision = True
                break

    return collision, step

def check_for_replanning(traj1, traj2, traj3, safe_dist):

    col12, t12 = check_trajectory_collision(traj1, traj2, safe_dist)
    col23, t23 = check_trajectory_collision(traj2, traj3, safe_dist)
    col31, t31 = check_trajectory_collision(traj1, traj3, safe_dist)

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
    # creating an Results folder to store the results
    if os.path.exists("Results"):
        shutil.rmtree("Results")
    
    os.makedirs("Results")

    safe_distance = 0.5         # safety clearance distance between robots
    goal_bias = 0.2             # probability for selecting the goal as the next random point
    goal_bias_reduction = 0.1   # reducing factor of goal bias for replanning

    # start and goal positions of the 3 robots
    start1 = [3, 3]
    goal1 = [1, 6]

    start2 = [1, 3]
    goal2 = [3, 8]

    start3 = [1, 2]
    goal3 = [7, 9]

    # Initial RRT* Planning
    robot1 = RRTStar(start1, goal1, safe_distance)
    robot2 = RRTStar(start2, goal2, safe_distance)
    robot3 = RRTStar(start3, goal3, safe_distance)

    traj1 = robot1.plan("Results/plan1.txt", "Results/explored1.png", goal_bias)
    traj2 = robot2.plan("Results/plan2.txt", "Results/explored2.png", goal_bias)
    traj3 = robot3.plan("Results/plan3.txt", "Results/explored3.png", goal_bias)

    fig, ax = plt.subplots()
    ax.set_xlim([0,10])
    ax.set_ylim([0,10])

    ax.plot(traj1[0], traj1[1], color = 'r', linewidth = 1)
    ax.plot(traj2[0], traj2[1], color = 'g', linewidth = 1)
    ax.plot(traj3[0], traj3[1], color = 'b', linewidth = 1)
    plt.savefig("Results/Plan0.png")

    replan = [True, True, True]

    for i in range(3):
        print("Replan Iteration " + str(i+1))
        goal_bias = goal_bias - goal_bias_reduction
        collision, collision_time = check_for_replanning(traj1, traj2, traj3, safe_distance)

        path_degrade1 = float('inf')
        path_degrade2 = float('inf')
        path_degrade3 = float('inf')

        if collision[0] and replan[0]:
            new_traj1 = robot1.replan([traj2, traj3], collision_time[0], "Results/replanned"+str(i)+"_1.txt", "Results/re_explored"+str(i)+"_1.png", goal_bias)
            path_degrade1 = (len(new_traj1[0])-len(traj1[0])) * 100 / float(len(traj1[0]))

        if collision[1] and replan[1]:
            new_traj2 = robot2.replan([traj1, traj3], collision_time[1], "Results/replanned"+str(i)+"_2.txt", "Results/re_explored"+str(i)+"_2.png", goal_bias)
            path_degrade2 = (len(new_traj2[0])-len(traj2[0])) * 100 / float(len(traj2[0]))

        if collision[2] and replan[2]:
            new_traj3 = robot3.replan([traj1, traj2], collision_time[2], "Results/replanned"+str(i)+"_3.txt", "Results/re_explored"+str(i)+"_3.png", goal_bias)
            path_degrade3 = (len(new_traj3[0])-len(traj3[0])) * 100 / float(len(traj3[0]))

        min_degrade = min(path_degrade1, path_degrade2, path_degrade3)

        if min_degrade == float('inf'):
            print("Collision-free trajectories are found")
            break

        elif min_degrade == path_degrade1:
            print("Trajectory 1 is replanned since it has minimum degradation")
            traj1 = new_traj1
            replan[0] = False

        elif min_degrade == path_degrade2:
            print("Trajectory 2 is replanned since it has minimum degradation")
            traj2 = new_traj2
            replan[1] = False

        elif min_degrade == path_degrade3:
            print("Trajectory 3 is replanned since it has minimum degradation")
            traj3 = new_traj3
            replan[2] = False

        fig2, ax2 = plt.subplots()
        ax2.set_xlim([0,10])
        ax2.set_ylim([0,10])
        ax2.plot(traj1[0], traj1[1], color = 'r', linewidth = 1)
        ax2.plot(traj2[0], traj2[1], color = 'g', linewidth = 1)
        ax2.plot(traj3[0], traj3[1], color = 'b', linewidth = 1)
        plt.savefig("Results/Plan"+str(i+1)+".png")

    fig3, ax3 = plt.subplots()
    ax3.set_xlim([0,10])
    ax3.set_ylim([0,10])

    obstacle = plt.Rectangle((4, 3), 2, 4, fill = True, color = 'k')
    ax3.add_patch(obstacle)
    
    ax3.plot(traj1[0], traj1[1], color = 'm', linewidth = 1)
    ax3.plot(traj2[0], traj2[1], color = 'm', linewidth = 1)
    ax3.plot(traj3[0], traj3[1], color = 'm', linewidth = 1)

    # saving the paths
    coordinates = traj1.T
    if os.path.exists("Results/final_traj1.txt"):
        os.remove("Results/final_traj1.txt")
    final_traj1 = open("Results/final_traj1.txt", "a")

    for i in range(len(coordinates)):
        np.savetxt(final_traj1, coordinates[i], fmt="%s", newline=' ')
        final_traj1.write("\n")

    coordinates = traj2.T
    if os.path.exists("Results/final_traj2.txt"):
        os.remove("Results/final_traj2.txt")
    final_traj2 = open("Results/final_traj2.txt", "a")

    for i in range(len(coordinates)):
        np.savetxt(final_traj2, coordinates[i], fmt="%s", newline=' ')
        final_traj2.write("\n")

    coordinates = traj3.T
    if os.path.exists("Results/final_traj3.txt"):
        os.remove("Results/final_traj3.txt")
    final_traj3 = open("Results/final_traj3.txt", "a")

    for i in range(len(coordinates)):
        np.savetxt(final_traj3, coordinates[i], fmt="%s", newline=' ')
        final_traj3.write("\n")

    plt.savefig("Results/final_all_trajectories.png")
    plt.show()
    plt.pause(30)
    plt.close()

if __name__ == "__main__":
    main()
