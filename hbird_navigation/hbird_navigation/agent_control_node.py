
import rclpy
import yaml
import math
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
#from .planner_utils import Environment, Visualizer, PathPlanner
from hbird_msgs.msg import Waypoint, State
from time import perf_counter

from scripts.planner_utils import Environment, Visualizer, PathPlanner


class AgentControlNode(Node):
    
    def __init__(self):
        super().__init__('agent_control_node')

        self.get_logger().info('Starting up the Agent Control Node...')

        # Add the parameter getter and the parameter declaration
        param_descriptor = ParameterDescriptor(description='Defines agent ID.')
        self.declare_parameter('agent_id', 'HB1', param_descriptor)
        self._agent_id = self.get_parameter('agent_id')._value
        
        # define ROS2 topics
        vehicle_state_topic = '/'+self._agent_id+'/agent_state'
        pos_setpoint_topic = '/'+self._agent_id+'/position_setpoint'

        # initialize subscriber and publisher
        self._state_subscriber = self.create_subscription(State, 
                                                          vehicle_state_topic,
                                                          self.state_update_callback, 10)

        self._pos_setpoint_publisher = self.create_publisher(Waypoint, 
                                                             pos_setpoint_topic, 10)
        
        # initialize timer
        self._publish_rate = 0.5  # sec/cycle
        self._publish_timer = self.create_timer(self._publish_rate, self.control_cycle)


        # planner config  
        config_path = "/home/robotics/robotics_ws/src/hbird_common/hbird_navigation/hbird_navigation/config/planner_config.yaml"
        config = self.load_config(config_path)
        self.env = Environment(config)
        self.path = self.plan_path()

        # define state and waypoint 
        self.current_target_index = 0
        self.target_height = 1.2 
        self.state_time = perf_counter()
        self.stage = "ground"

        self.start_x = self.env.start_pose.position.x - 4.5
        self.start_y = self.env.start_pose.position.y - 5.0 
        self.goal_x = self.env.goal_pose.x - 4.5
        self.goal_y = self.env.goal_pose.y - 5.0 
       
 
        # set desired position setpoints
        self.x_des = 0 #1.5
        self.y_des = 0 #1.5
        self.z_des = 0 #2.5
        self.psi_des = 6.283
        self.z_ground = 0.26


        # set thresholds
        self.pos_threshold = 0.1
        self.orient_threshold = 0.05

    def load_config(self, config_path): 
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        return config

    def get_current_time(self):
        return perf_counter()
    
    def angle_wrap(self, angle : float):
        return (angle + np.pi) % (2 * np.pi) - np.pi  #still confused about this 
    
    def state_update_callback(self, state_msg):
        self._state = state_msg # TODO: This is in ROS message format

    def reached_pos(self, target_pos) -> bool: #checking if state met target pos 
        pos_error = math.sqrt(
            (self._state.position.x - target_pos.position.x)**2 +
            (self._state.position.y - target_pos.position.y)**2 +
            (self._state.position.z - target_pos.position.z)**2
        ) 

        orient_error = abs(self.angle_wrap(self._state.orientation.z - target_pos.heading)) #check if orientation is met target
        return pos_error < self.pos_threshold and orient_error < self.orient_threshold


    def control_cycle(self):
        pos_setpoint = Waypoint()

        self.get_logger().info( 
            "State: {}, Time: {: .2f}, Current Target: {}".format(
               self.stage,
               perf_counter() - self.state_time,
               self.current_target_index
           )
        )
        if self.stage == "ground":
            pos_setpoint.position.x = self.x_des #position setpoint for ground stage
            pos_setpoint.position.y = self.y_des
            pos_setpoint.position.z = self.z_des
            pos_setpoint.heading = self.psi_des
           
            if self.get_current_time() - self.state_time > 10: # checks if time in stage passes 10 seconds
                self.state_time = self.get_current_time()
                self.stage = "takeoff"
        
        elif self.stage == "takeoff":
            pos_setpoint.position.x = self.start_x
            pos_setpoint.position.y = self.start_y
            pos_setpoint.position.z = self.target_height
            pos_setpoint.heading = self.psi_des  

            if self.reached_pos(pos_setpoint):
                self.state_time = self.get_current_time()
                self.stage = "follow_path"
        
        elif self.stage == "follow_path":
           current_target_waypoint = self.path[self.current_target_index] #waypoint current target 
           pos_setpoint = self.map_to_webots_transform(current_target_waypoint) # move waypoint to webots coordinates 

           pos_setpoint.position.z = self.target_height #height to target height 
           pos_setpoint.heading = self.psi_des

           current_pos = (self._state.position.x, self._state.position.y)
           target_pos = (pos_setpoint.position.x, pos_setpoint.position.y)
           path_pos = (current_target_waypoint.position.x, current_target_waypoint.position.y)

           self.get_logger().info(f"Current Position: {current_pos}, Target Position: {target_pos}, Path position: {path_pos}") # go back to 

        if self.reached_pos(pos_setpoint):
            self.current_target_index += 1

            if self.current_target_index == len(self.path):
                self.state_time = self.get_current_time()
                self.stage = "land"
        #elif land 
        # publish the setpoint
        self._pos_setpoint_publisher.publish(pos_setpoint)

    def map_to_webots_transform(self,waypoint): 
        webots_wp = Waypoint()
        webots_wp.position.x = waypoint.position.y - 4.5
        webots_wp.position.y = waypoint.position.x - 5.0
        return webots_wp

def main(args=None):
    rclpy.init(args=args)

    agent_control = AgentControlNode()

    rclpy.spin(agent_control)

    # Destroy the node explicitly
    agent_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()