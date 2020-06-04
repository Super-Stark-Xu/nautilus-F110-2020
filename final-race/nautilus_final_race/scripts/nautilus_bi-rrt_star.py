import numpy as np 
from scipy.interpolate import CubicSpline
from scipy.interpolate import interp1d
import os
from PIL import Image 
from matplotlib import pyplot as plt
from math import sqrt,cos,sin,atan2
import random, time
import csv

EPSILON = 5.0 # In cells
RADIUS = 10.0 # In cells
TARGET_RADIUS = 50.0 # In cells
MAX_ITER = 800000
CLEARANCE = 10 # unit cells

INTERPOLATION = 2000
WINDOW = 50

class Node:
    def __init__(self, xcoord, ycoord): #xcoord, ycoord --> grid coordinates
        self.x = xcoord
        self.y = ycoord
        self.cost = 0.0
        self.parent = None

class RRT_solver:
    def __init__(self):
        self.map, self.XDIM, self.YDIM = self.read_pgm()
        self.valid_points = []

    def convert_fromPose_toCell(self, pos): #pos_x, pos_y in meters
        (pos_x, pos_y) = pos
        cell_x = int((pos_x + 34.03)/0.05)
        cell_y = int((pos_y + 35.33)/0.05)
        return (cell_x, cell_y)

    def convert_fromCell_toPose(self, cell): #cell_x, cell_y in grid position
        (cell_x, cell_y) = cell
        pos_x = cell_x*0.05 - 34.03
        pos_y = cell_y*0.05 - 35.33 
        return (pos_x, pos_y)

    def read_pgm(self):
        dirname = os.path.dirname(__file__)
        filename = os.path.join(dirname, '../maps/race_track_f110.pgm')
        im = Image.open(filename)
        width, height = im.size 
        im = np.array(im.getdata(), dtype=np.float64)
        im = im.reshape((height, width))
        im = np.rot90(im, 3) # Rotate thrice to match the map offeset --> X := [0, 998]; Y:= [0, 778]
        # plt.imshow(im)
        # plt.show()
        return im, width, height 

    def getClearanceSpace(self, x1, y1, x2, y2):
        if x1>x2:
            xmax = x1
            xmin = x2
        else:
            xmax = x2
            xmin = x1

        if y1>y2:
            ymax = y1
            ymin = y2
        else:
            ymax = y2
            ymin = y1

        X1 = xmin - CLEARANCE
        X2 = xmax + CLEARANCE

        Y1 = ymin - CLEARANCE
        Y2 = ymax + CLEARANCE

        if X1 < 0:
            X1 = 0

        if X2 > self.XDIM-1:
            X2 = self.XDIM-1

        if Y1 < 0:
            Y1 = 0

        if Y2 > self.YDIM-1:
            Y2 = self.YDIM-1
        checkSpace = self.map[X1:X2, Y1:Y2]
        return checkSpace

    def checkObstacle(self, nn, newNode):
        # TODO: check the map to see if there's an obstacle in between nn, newNode
        # if self.map[x,y] == 255 (cell is obstacle free space); if self.map[x,y] != 255 (obstacle or outside track)
        # returns True if obstacle not present /False
        x1, y1 = nn.x, nn.y
        x2, y2 = newNode.x, newNode.y
        checkSpace = self.getClearanceSpace(x1, y1, x2, y2)
        checkSpace = checkSpace.ravel()
        if (len(checkSpace) > 0) and np.all(checkSpace == 255): # All space should be white
            return True
        else:
            return False

    def dist(self, p1, p2):
        return sqrt((p1.x-p2.x)**2 + (p1.y-p2.y)**2)
        
    def step_from_to(self, p1, p2):
        if self.dist(p1, p2) < EPSILON:
            px = int(p2.x)
            py = int(p2.y)
            return (px, py)
        else:
            theta = atan2(p2.y-p1.y, p2.x-p1.x)
            px = int(p1.x + EPSILON*cos(theta))
            py = int(p1.y + EPSILON*sin(theta))
            return (px, py)

    def chooseParent(self, nn, newNode, nodes):
        for p in nodes:
            cond1 = self.checkObstacle(p, newNode)
            cond2 = self.dist(p, newNode) < RADIUS
            cond3 = p.cost + self.dist(p, newNode) < nn.cost + self.dist(nn, newNode)
            if cond1 and cond2 and cond3:
                nn = p
        newNode.cost = nn.cost + self.dist(nn, newNode)
        newNode.parent = nn
        return newNode, nn

    def nearestNeighbor(self, nodes, q_target):
        q_near = nodes[0]
        for p in nodes:
            if self.dist(p, q_target) < self.dist(q_near, q_target):
                q_near = p
        return q_near

    def nodeSelectRandom(self):
        p = 0.7
        if len(self.valid_points) == 0:
            rand = Node(random.randint(0, self.XDIM), random.randint(0, self.YDIM))
            
        else:
            if random.random() < p:
                idx = random.randint(0, len(self.valid_points)-1)
                grid_x = self.valid_points[idx][0]
                rand_x = random.randint(grid_x-50, grid_x+50)
                rand_x = np.clip(rand_x, 0, self.XDIM)

                grid_y = self.valid_points[idx][1]
                rand_y = random.randint(grid_y-50, grid_y+50)
                rand_y = np.clip(rand_y, 0, self.YDIM)
                rand = Node(rand_x, rand_y)
            else:
                rand = Node(random.randint(0, self.XDIM), random.randint(0, self.YDIM))
        return rand

    def extend(self, nodes):
        rand = Node(random.randint(0, self.XDIM), random.randint(0, self.YDIM))
        # rand = self.nodeSelectRandom()
        nn = self.nearestNeighbor(nodes, rand)
        interpolateNode = self.step_from_to(nn, rand)
        newNode = Node(interpolateNode[0], interpolateNode[1])
        if self.checkObstacle(nn, newNode):
            newNode, nn = self.chooseParent(nn, newNode, nodes)
            nodes.append(newNode)
            self.valid_points.append([newNode.x, newNode.y])
        return nodes

    def checkGoal(self, newNode, goal):
        if self.dist(newNode, goal) < RADIUS and self.checkObstacle(newNode, goal):
            return True

    def back_trace(self, nodes, reverse):
        path_nodes = []
        path_nodes.append(nodes[-1])
        cur_parent = nodes[-1].parent
        while cur_parent != None:
           path_nodes.append(cur_parent)
           cur_parent = cur_parent.parent
        if reverse == True:
            path_nodes.reverse()
        return path_nodes

    def plot_grid(self, path_nodes):
        for node in path_nodes:
            x,y = node.x, node.y
            self.map[x:x+2,y:y+2] = np.ones((2,2))*100.0
        plt.imshow(self.map)
        plt.show()

    def calculate_theta(self, p1, p2):
        theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
        return theta

    def get_waypoints(self, nodes):
        waypoints = []
        for i in range(len(nodes)):
            if i == 0:
                node_cell = (nodes[i].x, nodes[i].y)
                node_pos = self.convert_fromCell_toPose(node_cell)
                theta = 0.0
            else:
                node_cell = (nodes[i].x, nodes[i].y)
                node_pos = self.convert_fromCell_toPose(node_cell)
                theta = self.calculate_theta(prev_node_pos, node_pos)
            waypoints.append([node_pos[0], node_pos[1], theta])
            prev_node_pos = node_pos
        return waypoints

    def interpolateWaypoints(self, waypoints):
        N = len(waypoints)
        if N > INTERPOLATION:
            interpolated_waypoints = waypoints
        else:
            interpolated_waypoints = []
            K = int(INTERPOLATION/N)
            for i in range(N-1):
                X0 = waypoints[i]
                X1 = waypoints[i+1]
                for k in range(K+1):
                    x = X0[0] + k*(X1[0] - X0[0])/K
                    y = X0[1] + k*(X1[1] - X0[1])/K
                    th = X0[2] + k*(X1[2] - X0[2])/K
                    Xnew = [x, y, th]
                    interpolated_waypoints.append(Xnew)
        return interpolated_waypoints

    def smooth_waypoints(self, waypoints):
        N = len(waypoints)
        smooth_waypoints = []
        waypoints = np.array(waypoints, dtype=np.float64)
        for i in range(N):
            strt_idx = i
            end_idx = i + WINDOW
            if end_idx < N:
                x_win = waypoints[strt_idx:end_idx, 0]
                y_win = waypoints[strt_idx:end_idx, 1]
                th_win = waypoints[strt_idx:end_idx, 2]
            else:
                x_win = waypoints[strt_idx:, 0]
                y_win = waypoints[strt_idx:, 1]
                th_win = waypoints[strt_idx, 2]

            x = np.mean(x_win)
            y = np.mean(y_win)
            th = np.mean(th_win)
            Xnew = [x,y,th]
            smooth_waypoints.append(Xnew)
        return smooth_waypoints

    def plot_waypoints(self, waypoints):
        print("Printing Waypoints")
        X = []
        Y = []
        for i in waypoints:
            X.append(i[0])
            Y.append(i[1])
        plt.plot(X,Y)
        plt.show()

    def RRT_solve(self, start, goal):
        start_cell = self.convert_fromPose_toCell(start)
        goal_cell = self.convert_fromPose_toCell(goal)
        print("Start Cell: ", start_cell)
        print("Goal  Cell: ", goal_cell)
        print("Searching for a Path...")

        start = Node(start_cell[0], start_cell[1])
        goal = Node(goal_cell[0], goal_cell[1])

        start_nodes = []
        start_nodes.append(start)

        goal_nodes = []
        goal_nodes.append(goal)

        i = 0
        flag = False
        start_time = time.time()
        while i < MAX_ITER and flag != True:
            start_nodes = self.extend(start_nodes)
            goal_nodes = self.extend(goal_nodes)

            q_target = goal_nodes[-1]
            # for q_target in goal_nodes:
            q_near = self.nearestNeighbor(start_nodes, q_target)
            if self.dist(q_target, q_near) < TARGET_RADIUS:
                if self.checkObstacle(q_near, q_target):
                    newNode = Node(q_target.x, q_target.y)
                    newNode, nn = self.chooseParent(q_near, newNode, start_nodes)
                    start_nodes.append(newNode)
                    flag = True
                    break
            i += 1
            print("iteration cnt: ", i)

        if flag == True:
            tot_time = str(round(time.time() - start_time, 2))
            print("Path found!")
            print("Time Taken: " + tot_time + 'sec')
            print("Total Iter: " + str(i))

            start_path = self.back_trace(start_nodes, reverse=True)
            goal_path = self.back_trace(goal_nodes, reverse=False)
            path = start_path + goal_path

            self.plot_grid(path)
            pos_waypoints = self.get_waypoints(path)
            pos_waypoints = self.interpolateWaypoints(pos_waypoints)
            pos_waypoints = self.smooth_waypoints(pos_waypoints)
            return pos_waypoints
        else:
            print("Solution not found! Try increasing the number of iterations")
            return None

if __name__ == '__main__':
    start = (0.00, 0.00) # start point C0
    C1 = (13.405, -10.46) # Checkpoint 1
    C2 = (1.395, -11.87)
    C3 = (-18.815, -28.605)
    C4 = (-24.02, -21.135)
    C5 = (-16.82, -4.93)

    waypoints = []

    rrt_obj1 = RRT_solver()
    waypoints1 = rrt_obj1.RRT_solve(start, C2)
    waypoints += waypoints1
    rrt_obj1.plot_waypoints(waypoints1)
    with open('rrt_waypoints_S-C2.csv', 'wb') as file:
        wr = csv.writer(file)
        wr.writerows(waypoints1)
    
    rrt_obj2 = RRT_solver()
    waypoints2 = rrt_obj2.RRT_solve(C2, C4)
    waypoints += waypoints2
    rrt_obj2.plot_waypoints(waypoints2)
    with open('rrt_waypoints_C2-C4.csv', 'wb') as file:
        wr = csv.writer(file)
        wr.writerows(waypoints2)

    rrt_obj3 = RRT_solver()
    waypoints3 = rrt_obj3.RRT_solve(C4, C5)
    waypoints += waypoints3
    rrt_obj3.plot_waypoints(waypoints3)
    with open('rrt_waypoints_C4-C5.csv', 'wb') as file:
        wr = csv.writer(file)
        wr.writerows(waypoints3)

    rrt_obj4 = RRT_solver()
    waypoints4 = rrt_obj4.RRT_solve(C5, start)
    waypoints += waypoints4
    rrt_obj4.plot_waypoints(waypoints4)
    with open('rrt_waypoints_C5-S.csv', 'wb') as file:
        wr = csv.writer(file)
        wr.writerows(waypoints4)

    rrt_obj4.plot_waypoints(waypoints)
    with open('rrt_waypoints_final.csv', 'wb') as file:
        wr = csv.writer(file)
        wr.writerows(waypoints)
    



