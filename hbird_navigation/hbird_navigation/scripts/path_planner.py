from hbird_msgs.msg import Waypoint
import math

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

        # your code here

        
    def plan(self):
        """
        Main method that computes and returns the path

        Returns:
        - path (list of Waypoint objects)      
        """
        
        # for each square
        for x in range(self.env.x_min + (self.env.grid_size/2), self.env.x_max, self.env.grid_size):
            for y in range(self.env.y_min + (self.env.grid_size/2), self.env.y_max, self.env.grid_size):
            
            # ensure there are no obstacles in this square
            if x in self.env.obstacles and y in self.env.obstacles:
                
                array1[] = list(np.arange(pair[current_pos[x - self.env.robot_radius]][current_pos[y - self.env.robot_radius]], pair[current_pos[x + self.env.robot_radius]][current_pos[y + self.env.robot_radius]], 0.1
                
                x_radius_max = current_pos[x] + robot_radius
                x_radius_min = current_pos[x] - robot_radius
                y_radius_max = current_pos[y] + robot_radius
                y_radius_min = current_pos[y] - robot_radius
                
                if self.env.obstacles['x'] < x_radius_max and self.env.obstacles['x'] > x_radius_min:
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



        path = []

        # your code here

        
        return path


