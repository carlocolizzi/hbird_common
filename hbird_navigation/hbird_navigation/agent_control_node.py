import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

import pathlib
import yaml

from hbird_msgs.msg import Waypoint, State
from .scripts.path_planner import PathPlanner
from .scripts.planner_utils import Environment, Visualizer

import math

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
        self._publish_rate = 0.2  # sec/cycle
        self._publish_timer = self.create_timer(self._publish_rate, self.control_cycle)

        # get planner config file
        config_path = str(pathlib.Path().parent.resolve())+"/src/hbird_common/hbird_navigation/hbird_navigation/config/"
        config_file = "planner_config.yaml"
        try:
            with open(config_path + config_file, "r") as file:
                config = yaml.load(file, Loader=yaml.FullLoader)
        except FileNotFoundError:
            print(f"The file '{config_file}' does not exist.")
        except yaml.YAMLError as e:
            print(f"Error parsing the YAML file: {e}")

        # set up environment and obtain path
        env = Environment(config)

        # initialize planner
        path_planner = PathPlanner(env)

        # call planner
        self.path = path_planner.plan()
        
        # initialize and call the visualizer
        viz = Visualizer(env)
        viz.plot(self.path)

        self.index = 0
        self.travel_height = 1.2

        # initialise state - this is the current status of the sim
        self.stage = "ground"
        self._state = State()

        # convert coordinates to webots coordinates
        self.start_x = env.start_pose.position.x - 4.5
        self.start_y = env.start_pose.position.y - 5.0
        self.goal_x = env.goal_pose.position.x - 4.5
        self.goal_y = env.goal_pose.position.y - 5.0

    def state_update_callback(self, state_msg):
        self._state = state_msg

    def reached_target_pose(self, target_pos):
        pos_error = math.sqrt((self._state.position.x - target_pos.position.x) ** 2
                            + (self._state.position.y - target_pos.position.y) ** 2)
                            
        if pos_error < 0.5:         # error threshold to go to next point
            return True
        else:
            return False
    
    
    def control_cycle(self):

        pos_setpoint = Waypoint()
        pos_setpoint.heading = 0.0

        if self.stage == "ground":
            # self.get_logger().info('Ground')

            pos_setpoint.position.x = self.start_x
            pos_setpoint.position.y = self.start_y
            pos_setpoint.position.z = 0.0

            self.stage = "takeoff"

        elif self.stage == "takeoff":
            # self.get_logger().info('takeoff')

            pos_setpoint.position.x = self.start_x
            pos_setpoint.position.y = self.start_y
            pos_setpoint.position.z = self.travel_height

            if self.reached_target_pose(pos_setpoint):
                self.stage = "follow"

        elif self.stage == "follow":
            # self.get_logger().info('follow')

            pos_setpoint = self.map_to_webots_transform(self.path[self.index])
            pos_setpoint.position.z = self.travel_height

            if self.reached_target_pose(pos_setpoint):
                self.index += 1
                if self.index == len(self.path):
                    self.stage = "land"

        elif self.stage == "land":
            # self.get_logger().info('land')

            pos_setpoint.position.x = self.goal_x
            pos_setpoint.position.y = self.goal_y
            # self.get_logger().info('switching to z land')

            pos_setpoint.position.z = -1.2
            self.stage = "end"

        elif self.stage == "end":
            while True:
                pass
    
        # publish the setpoint
        self._pos_setpoint_publisher.publish(pos_setpoint)

    def map_to_webots_transform(self, waypoint):
        """Helper function that returns a Waypoint adjusted to the webots reference frame"""
        webots_wp = Waypoint()
        webots_wp.position.x = waypoint.position.x - 4.5
        webots_wp.position.y = waypoint.position.y - 5.0
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