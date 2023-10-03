from hbird_msgs.msg import Waypoint
from queue import PriorityQueue
import math

class Node:
    def __init__(self, x, y, g, h, f, parent):
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.f = f
        self.parent = parent

    def __lt__(self, other):
        return self.f < other.f

class PathPlanner():
    """
    This class computes a waypoint-based path from the start to the goal poses.

    You are to implement the __init__ and plan methods. Feel free to add any more methods.
    """

    def __init__(self, env):
        """
        Inputs:
        - env (dict):    dictionary storing relevant environment parameters 
                         (see Environment class in planner_utils.py)
        """
        self.env = env
        
        self.open = PriorityQueue() ### FIND WAY OF SORTING QUEUE
        
        start_node = Node(self.env.start_pose.position.x, self.env.start_pose.position.y, 0, self.heuristic(self.env.start_pose.position.x, self.env.start_pose.position.y), self.heuristic(self.env.start_pose.position.x, self.env.start_pose.position.y), None)
        self.open.put(start_node.f, start_node)

        self.closed = []
        
    def heuristic(self, x, y):
        
        x1 = self.env.goal_pose.position.x - x
        y1 = self.env.goal_pose.position.y - y
        heuristic = math.sqrt(x1 **2 + y1 **2)
        
        return heuristic

    def get_neighbours(self, node):
        x = node.x
        y = node.y 
        neighbours = []
        
        neighbours_coordinates = [[x-self.env.grid_size, y - self.env.grid_size], [x-self.env.grid_size, y], [x-self.env.grid_size, y + self.env.grid_size],
                      [x, y - self.env.grid_size], [x, y + self.env.grid_size],
                      [x+self.env.grid_size, y - self.env.grid_size], [x+self.env.grid_size, y], [x+self.env.grid_size, y + self.env.grid_size]]
        
        for each in neighbours_coordinates:
            temp = Node(each[0], each[1], node.g + 1, self.heuristic(each[0], each[1]), node.g + 1 + self.heuristic(each[0], each[1]), node)
            neighbours.append(temp)
        
        return neighbours
    
    def check_bounds(self, node):

        check = False

        if node.x >= self.env.x_min and node.x <= self.env.x_max:
            if node.y >= self.env.y_min and node.y <= self.env.y_max:
                check = True
        
        return check
        
    def check_obstacles(self, node):
        check = False

        corner1 = [node.x - self.env.robot_radius, node.y - self.env.robot_radius]
        corner2 = [node.x + self.env.robot_radius, node.y - self.env.robot_radius]
        corner3 = [node.x - self.env.robot_radius, node.y + self.env.robot_radius]
        corner4 = [node.x + self.env.robot_radius, node.y + self.env.robot_radius]

        if corner1 not in self.env.obstacles["x"]["y"] and corner2 not in self.env.obstacles and corner3 not in self.env.obstacles and corner4 not in self.env.obstacles:
            check = True            ### NEED TO FIND WAY TO COMPARE LIST TO DICTIONARY VALUES BY PAIR

        return check
    
    def plan(self):
        """
        Main method that computes and returns the path

        Returns:
        - path (list of Waypoint objects)      
        """
        distance = []
        
        while self.open:
            node = self.open.get()      # this is a node object
            self.closed.append(node)

            if node.x == self.env.goal_pose.position.x and node.y == self.env.goal_pose.position.y:
                
                path = []
                
                while node:
                    waypoint = Waypoint()
                    waypoint.position.x = node.x
                    waypoint.position.y = node.y
                    
                    path.append(waypoint)
                    node = node.parent
                
                path.reverse()

                return path
            else:
                neighbours = self.get_neighbours(node) 

            for element in neighbours:

                bounds = self.check_bounds(element)
                obstacles = self.check_obstacles(element)
                
                comparison =  self.open

                if bounds == True and obstacles == True:
                    if element in self.closed: 
                        continue
                    if element in self.open:
                        if element.f > self.open[].f: ### need to fidn out how to find element in dict
                            continue
                    
                    self.open.get(old one)
                    self.open.put(element)
   
        return None

            


        '''
        while current_pos != self.env.goal_pos:
            node = self.open.f.index(min(self.open.f))
            
            if node == self.env.goal_pos:
                ## DONE
            else:
                self.closed.append(node)
                neighbours = get_neighbours(node)
            for element in neighbours:
                if element.g < node.g and element in self.closed:
                    self.closed
        
        
        # for each square

        
        for x in range(self.env.x_min + (self.env.grid_size/2), self.env.x_max, self.env.grid_size):
            for y in range(self.env.y_min + (self.env.grid_size/2), self.env.y_max, self.env.grid_size):
        

        for x in range(current_pos[x] - self.env.grid_size, current_pos[x] + self.env.grid_size, self.env.grid_size):
            for y in range(current_pos[y] - self.env.grid_size, current_pos[y] + self.env.grid_size, self.env.grid_size):
            
            # ensure there are no obstacles in this square
            if x in self.env.obstacles and y in self.env.obstacles:
                
                # array1[] = list(np.arange(pair[current_pos[x - self.env.robot_radius]][current_pos[y - self.env.robot_radius]], pair[current_pos[x + self.env.robot_radius]][current_pos[y + self.env.robot_radius]], 0.1
                
                x_radius_max = current_pos[x] + self.env.robot_radius
                x_radius_min = current_pos[x] - self.env.robot_radius
                y_radius_max = current_pos[y] + self.env.robot_radius
                y_radius_min = current_pos[y] - self.env.robot_radius
                
                if any(self.env.obstacles['x'] < x_radius_max and self.env.obstacles['x'] > x_radius_min:
                    if self.env.obstacles['y'] < y_radius_max and self.env.obstacles['y'] > y_radius_min:
                        grid[x][y] = 999

                break
            

            # find distance to goal
            distance.x = self.env.goal_pose.position.x - x
            distance.y = self.env.goal_pose.position.y - y
            distance.sum = math.sqrt(distance.x **2 + distance.y **2)

            grid[x][y] = distance.sum

            # find distance to drone
            distance.x = x - self.env.start_pose.position.x
            distance.y = y - self.env.start_pose.position.y
            distance.sum = math.sqrt(distance.x **2 + distance.y **2)

            # add distance to target + distance to drone
            grid[x][y] += distance.sum
        '''


        # path = 
        
        # return path


