# Carlo Colizzi, Lulu de la Pena
# Oct 3, 2023

from hbird_msgs.msg import Waypoint
from queue import PriorityQueue
import math

class Node:
    def __init__(self, env, x, y, g, h, f, parent):

        self.x = x              # X coordinate of node
        self.y = y              # Y coordinate of node
        self.g = g              # cost so far
        self.h = h              # heuristic - aka cost to get to target
        self.f = f              # total cost = g + h
        self.parent = parent    # node previous to this in the path

        # calculating ID based on coordinates
        max_cols = (env.x_max - env.x_min) // env.grid_size
        row = math.ceil(y / env.grid_size)
        column = math.ceil(x / env.grid_size)

        self.id = ((row - 1) * max_cols) + column



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
        
        self.open = {'x': [], 'y': [], 'g': [], 'h': [], 'f': [], 'parent': [], 'id': []}              # this is a dict with x y g h f parent id as keys and lists as values
        self.closed = {'x': [], 'y': [], 'g': [], 'h': [], 'f': [], 'parent': [], 'id': []}            # this is a dict with x y g h f parent id as keys and lists as values

        start_node = Node(self.env, self.env.start_pose.position.x, self.env.start_pose.position.y, 0, self.heuristic(self.env.start_pose.position.x, self.env.start_pose.position.y), self.heuristic(self.env.start_pose.position.x, self.env.start_pose.position.y), None)
        self.append_node(self.open, start_node)


    def append_node(self, list, node):
        list['x'].append(node.x)
        list['y'].append(node.y)
        list['g'].append(node.g)
        list['h'].append(node.h)
        list['f'].append(node.f)
        list['parent'].append(node.parent)
        list['id'].append(node.id)
       
    def heuristic(self, x, y):
        
        x1 = self.env.goal_pose.position.x - x
        y1 = self.env.goal_pose.position.y - y
        heuristic = math.sqrt(x1 **2 + y1 **2)
        
        return heuristic
    
    def get_distance(self, x1, y1, x2, y2):
    
        x =  x1 - x2
        y =  y1 - y2
        distance = math.sqrt(x **2 + y **2)
    
        return distance

    def get_neighbours(self, node):
        x = node.x
        y = node.y 
        neighbours = []
        
        neighbours_coordinates = [[x-self.env.grid_size, y - self.env.grid_size], [x-self.env.grid_size, y], [x-self.env.grid_size, y + self.env.grid_size],
                                  [x, y - self.env.grid_size], [x, y + self.env.grid_size],
                                  [x+self.env.grid_size, y - self.env.grid_size], [x+self.env.grid_size, y], [x+self.env.grid_size, y + self.env.grid_size]]
        

        # neighbours_coordinates = [[x-self.env.grid_size, y],
        #                           [x, y - self.env.grid_size], [x, y + self.env.grid_size], 
        #                           [x+self.env.grid_size, y]]
        
        for each in neighbours_coordinates:
            temp = Node(self.env, each[0], each[1], node.g + 1, self.heuristic(each[0], each[1]), node.g + 1 + self.heuristic(each[0], each[1]), node)
            neighbours.append(temp)
        
        return neighbours
    
    def check_bounds(self, node):

        check = False

        if (node.x -  self.env.robot_col_radius) >= self.env.x_min and (node.x +  self.env.robot_col_radius) <= self.env.x_max:
            if (node.y -  self.env.robot_col_radius) >= self.env.y_min and (node.y +  self.env.robot_col_radius) <= self.env.y_max:
                check = True
        
        return check
        
    def check_obstacles(self, node):
        '''
        LOOP obstacles BY INDEX and check if coordinate (paired by index) is less than robot_radius away from node coordinates
        '''
        check = True

        for i in range(0, len(self.env.obstacles['x'])):
            x = self.env.obstacles['x'][i]
            y = self.env.obstacles['y'][i]
            
            if self.get_distance(x, y, node.x, node.y) < self.env.robot_col_radius:
                check = False
        
        return check    
    
    def get_lowest(self, list, value):
        '''
        Returns node from list with lowest 'value'. Also removes node from the list
        '''
        
        index = list[value].index(min(list[value]))

        x = list['x'].pop(index)
        y = list['y'].pop(index)
        g = list['g'].pop(index)
        h = list['h'].pop(index)
        f = list['f'].pop(index)
        parent = list['parent'].pop(index)
        id = list['id'].pop(index)

        node = Node(self.env, x, y, g, h, f, parent)
        
        return node



    def plan(self):
        """
        Main method that computes and returns the path

        Returns:
        - path (list of Waypoint objects)      
        """
        path = []

        while len(self.open['id']) > 0:
            #print("###########WHILE", self.open)
            node = self.get_lowest(self.open, 'f')        # this is a node object - automatically removed from open

            if abs( (node.x-self.env.goal_pose.position.x) + (node.y-self.env.goal_pose.position.y)) < 0.5:
            #if node.x == self.env.goal_pose.position.x and node.y == self.env.goal_pose.position.y:     # check if we have reached the goal
                                
                while node:
                    waypoint = Waypoint()
                    waypoint.position.x = node.x
                    waypoint.position.y = node.y
                    waypoint.position.z = 1.0
                    
                    path.append(waypoint)
                    node = node.parent
                
                path.reverse()      # work your way backwards for the path

                return path
            
            
            
            self.append_node(self.closed, node)
            neighbours = self.get_neighbours(node) 

            for element in neighbours:

                bounds = self.check_bounds(element)         # check if node is inside grid
                obstacles = self.check_obstacles(element)   # check if any obstacles are overlapping
            
                if bounds == True and obstacles == True:

                    if element.id in self.open['id']:


                        #print("in OPEN")

                        index = self.open['id'].index(element.id)

                        if element.f > self.open['f'][index]: ### need to fidn out how to find element in dict
                            continue
                        
                        
                        self.open['x'].pop(index)           # delete istance from dictionary
                        self.open['y'].pop(index)
                        self.open['g'].pop(index)
                        self.open['h'].pop(index)
                        self.open['f'].pop(index)
                        self.open['parent'].pop(index)
                        self.open['id'].pop(index)
                        
                        self.append_node(self.open, element)  # add to open list


                    elif element.id in self.closed['id']: 
                        #print("in CLOSED")
                        #print("$$$$$$$$$$", self.closed)

                        index = self.closed['id'].index(element.id)

                        if element.f > self.closed['f'][index]:
                            continue

                        self.closed['x'].pop(index)           # delete istance from dictionary
                        self.closed['y'].pop(index)
                        self.closed['g'].pop(index)
                        self.closed['h'].pop(index)
                        self.closed['f'].pop(index)
                        self.closed['parent'].pop(index)
                        self.closed['id'].pop(index)

                        self.append_node(self.closed, element)  # add to open list

                    else:
                        #print("in ELSE")
                        self.append_node(self.open, element)  # add to open list
   
        return None

