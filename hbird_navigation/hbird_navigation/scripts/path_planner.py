#from hbird_msgs.msg import Waypoint
from queue import PriorityQueue
import math

class Node:
    def __init__(self, x, y, f, g, h, parent):
        self.x = x
        self.y = y
        self.f = f
        self.g = g
        self.h = h
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

        self.open.put(self.env.start_pos)

        self.closed = []
        

    def get_neighbours(self, node):
        x = node.x
        y = node.y 

        neighbours = [[x-self.env.grid_size, y - self.env.grid_size], [x-self.env.grid_size, y], [x-self.env.grid_size, y + self.env.grid_size],
                      [x, y - self.env.grid_size], [x, y + self.env.grid_size],
                      [x+self.env.grid_size, y - self.env.grid_size], [x+self.env.grid_size, y], [x+self.env.grid_size, y + self.env.grid_size],]
        
        return neighbours


    def plan(self):
        """
        Main method that computes and returns the path

        Returns:
        - path (list of Waypoint objects)      
        """
        distance = []
        
        while self.open:
            node = self.open.get()
            self.closed.append(node)
        
            if node == self.env.goal_pos:
                return self.open 
            else:
                neighbours = get_neighbours(node) 

    
                for element in neighbours:
                    element.parent = node
                    distance.x = self.env.goal_pos.position.x - element.x
                    distance.y = self.env.goal_pos.position.y - element.y
                    element.h = math.sqrt(distance.x **2 + distance.y **2)
                    element.g = node.g + math.sqrt(node.x-element.x ** 2 + node.y - element.y ** 2)
                    element.f = element.g + element.h
                
                    if element in self.open.queue:
                        existing_node = None   # find out if element is in open set and lower f 
                        for open_node in self.open.queue:
                            if open_node.x == element.x and open_node.y == element.y: 
                                existing_node = open_node
                                break
                        if existing_node is not None and element.f < existing_node.f:  # if lower f, update existing node in open set 
                            existing_node.g = element.g
                            existing_node.f = element.f
                            existing_node.parent = element.parent 
                    else:
                        if element in self.closed:
                            existing_node = self.closed[element]
                        if element.f < existing_node.f:  # if lower f, update existing node in closed set
                            self.closed.remove(element)
                            self.open.put(element)
                        else:
                            self.open.put(element)
                '''
                if element in self.open.queue and element.f > self.open.queue[element].f: ### need to fidn out how to find element in queue ## added slef.open.queue to get elements in queue 
                    continue
                if element in self.closed and element.f > self.closed[element].f: ### need to fidn out how to find element in queue
                    continue
                
                self.open.get(element)
                self.closed.get(element)
                self.open.put(element)
                '''
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


