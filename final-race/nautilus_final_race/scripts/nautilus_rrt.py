import numpy as np 
import os
from PIL import Image 
from matplotlib import pyplot as plt
from math import sqrt,cos,sin,atan2
import random, time
import csv

EPSILON = 5.0 # In cells
RADIUS = 10.0 # In cells
MAX_ITER = 8000

class Node:
    def __init__(self, xcoord, ycoord): #xcoord, ycoord --> grid coordinates
        self.x = xcoord
        self.y = ycoord
        self.cost = 0.0
        self.parent = None

class RRT_solver:
    def __init__(self, start, goal):
        self.map, self.XDIM, self.YDIM = self.read_pgm()
        start_cell = self.convert_fromPose_toCell(start)
        goal_cell = self.convert_fromPose_toCell(goal)
        print("Start Cell: ", start_cell)
        print("Goal  Cell: ", goal_cell)
        print("Searching for a Path...")
        waypoints = self.RRT_solve(start_cell, goal_cell)
        # self.plot_waypoints(waypoints)

    def plot_waypoints(self, waypoints):
        print("Printing Waypoints")
        X = []
        Y = []
        for i in waypoints:
            X.append(i[0])
            Y.append(i[1])
            print(i[0], i[1], i[2])
        print("")
        plt.plot(X,Y)
        plt.show()

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

    def checkObstacle(self, nn, newNode):
        # TODO: check the map to see if there's an obstacle in between nn, newNode
        # if self.map[x,y] == 255 (cell is obstacle free space); if self.map[x,y] != 255 (obstacle or outside track)
        # returns True if obstacle not present /False

        x1, y1 = nn.x, nn.y
        x2, y2 = newNode.x, newNode.y
        
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

        checkSpace = self.map[xmin:xmax, ymin:ymax]
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

    def checkGoal(self, newNode, goal):
        if self.dist(newNode, goal) < RADIUS and self.checkObstacle(newNode, goal):
            return True

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
            waypoints.append((node_pos[0], node_pos[1], theta))
            prev_node_pos = node_pos
        return waypoints

    def smooth_waypoints(self, waypoints):
        # TODO: smooth the waypoints = [(X1,Y1,th1), (X2,Y2,th2), ..., (Xg, Yg, thg)]
        # Apply cubic spline
        x = 0
        y = 0
        theta = 0
        smooth_waypoints = []
        for idx in range(waypoints):
            if idx < 50 or idx > len(wayponts)-50:
                smooth_point = waypoints[idx]
            else:
                # find mean the closest 100 point for every point(not the first 50 and last 50 points)
        for smooth_idx in range(-50,50):
            point = waypoints[idx + smooth_idx]
            x += point[0]
            y += point[1]
            theta += point[2]

            smooth_point_x = x/100
            smooth_point_y = y/100
            smooth_point_theta = theta/100
            smooth_point = tuple(smooth_point_x, smooth_point_y, smooth_point_theta)
            
            smooth_waypoints.append(smooth_point)
    return smooth_waypoints

    def back_trace(self, nodes):
        path_nodes = []
        path_nodes.append(nodes[-1])
        cur_parent = nodes[-1].parent
        while cur_parent != None:
           path_nodes.append(cur_parent)
           cur_parent = cur_parent.parent
        path_nodes.reverse()
        return path_nodes

    def plot_grid(self, path_nodes):
        for node in path_nodes:
            x,y = node.x, node.y
            self.map[x:x+2,y:y+2] = np.ones((2,2))*100.0
        plt.imshow(self.map)
        plt.show()

    def RRT_solve(self, start, goal):
        start = Node(start[0], start[1])
        goal = Node(goal[0], goal[1])
        nodes = []

        nodes.append(start)
        i = 0
        start_time = time.time()
        while i < MAX_ITER and goal not in nodes:
            rand = Node(random.random()*self.XDIM, random.random()*self.YDIM)
            
            nn = nodes[0]
            for p in nodes:
                if self.dist(p, rand) < self.dist(nn, rand):
                    nn = p

            interpolateNode = self.step_from_to(nn, rand)
            newNode = Node(interpolateNode[0], interpolateNode[1])

            if self.checkObstacle(nn, newNode):
                newNode.cost = nn.cost + self.dist(nn, newNode)
                newNode.parent = nn
                nodes.append(newNode)

            if self.checkGoal(newNode, goal):
                print("Path found")
                goal.cost = newNode.cost + self.dist(newNode, goal)
                goal.parent = newNode
                nodes.append(goal)

        i += 1
        if goal in nodes:
            path_nodes = self.back_trace(nodes)
            self.plot_grid(path_nodes)

            pos_waypoints = self.get_waypoints(path_nodes)
            # pos_waypoints = self.smooth_waypoints(pos_waypoints)
            return pos_waypoints
        else:
            print("Solution not found! Try increasing the number of iterations")
            return None

if __name__ == '__main__':
    start = (0.0, 0.0) # start point C0
    C1 = (13.47, -10.33)# Checkpoint 1
    rrt_obj = RRT_solver(start, C1)





