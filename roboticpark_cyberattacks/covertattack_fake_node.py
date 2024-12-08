import rclpy, sys, os
from rclpy.node import Node, NodeNameNonExistentError
from roboticpark_cyberattacks.utils import rslg, printargs
from rcl_interfaces.srv import *
from webots_ros2_driver.webots_controller import WebotsController
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchService
from asyncio_run_in_process import run_in_process,open_in_process
from multiprocessing import Pool


class covertnodeFake(Node):

    impersonateNodeName = ""
    impersonateNodeDescription = ""


    def __init__(self):
        super().__init__("covertnodefakenode")
        self.declare_parameter('reply_node_impersonate_name', "Unset")
        self.impersonateNodeName = self.get_parameter('reply_node_impersonate_name').get_parameter_value().string_value
        printargs(self)

        self.launchImpersonatedNode()

    def launchImpersonatedNode(self):
        """This function  launchs a node that mimics the original one.
        In this case we know exactly the type of the node we are going to launch. 

        Parameters:

        request: Dict, the request
        response:  Dict, the response

        """
        rslg(self,f'Launching node...')

        self.launch_service = LaunchService()
        package_path = get_package_share_directory('roboticpark_cyberattacks')
        robot_description = os.path.join(package_path , 'config', 'reply.crazyflie.urdf')
        config_path = os.path.join(get_package_share_directory('roboticpark_cyberattacks'), 'config', 'reply.node.config.yaml')
        with open(robot_description, 'r') as infp:
            robot_description = infp.read()
        robot_description = robot_description.replace("my_config_file", config_path)
        rslg(self,f'{robot_description}')

        self.impersonateNodeDescription = WebotsController(
                            robot_name=self.impersonateNodeName,
                            parameters=[
                                {'robot_description': robot_description,
                                'use_sim_time': False,
                                'set_robot_state_publisher': True},
                            ],
                            remappings= [ ('asdfasdf', 'adsfasdfasd'),('/dron08/local_pose', '/dron08/myypose'),('local_pose', 'myypose')],
                            respawn=True
                        )

        self.launch_service.include_launch_description(
            launch.LaunchDescription([self.impersonateNodeDescription])
        )
        with Pool(processes=2) as pool:
            res = pool.apply_async(self.launch_aux_node())

    def launch_aux_node(self):

        self.get_logger().info("Launching node")
        self.launch_service.run()

def main():
    rclpy.init()
    mycovertnodeFake = covertnodeFake()

    rclpy.spin(mycovertnodeFake)

    rclpy.shutdown()
    