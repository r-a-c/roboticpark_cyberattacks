import rclpy, sys
from rclpy.node import Node, NodeNameNonExistentError
from roboticpark_cyberattacks.utils import rslg, printargs, import_message_type
from rcl_interfaces.srv import *
from std_srvs.srv import Trigger

class replynode(Node):

    replyTopic = None
    replyTopicType = None
    messaggesList = []
    replyAmount = 0

    def __init__(self):
        super().__init__("replynode")
        self.declare_parameter('reply_topic', "Unset")
        self.declare_parameter('reply_topic_type', "Unset")
        self.declare_parameter('reply_amount', "Unset")
        self.replyTopic = self.get_parameter('reply_topic').get_parameter_value().string_value
        replyTopicTypeValue = self.get_parameter('reply_topic_type').get_parameter_value().string_value
        self.replyAmount = int(self.get_parameter('reply_amount').get_parameter_value().string_value)
        printargs(self)

        # Create replication service
        serviceName = f'{self.get_name()}/data_replicate'
        self.srv = self.create_service(Trigger, serviceName, self.publishStoredData)

        if self.replyAmount == 'Unset' or self.replyTopic == 'Unset' or replyTopicTypeValue == 'Unset':
            rslg(self,f'No reply_topic, reply_amount or reply_topic_type passed as parameter')
            sys.exit()

        self.replyTopicType = import_message_type(self,replyTopicTypeValue)
        self.subscription = self.create_subscription(self.replyTopicType,self.replyTopic,self.save_messagges,10)

    def save_messagges(self, msg):
        """This function  save a requested amount of messagges to a list

        Parameters:

        self: Node, the node used to perform the activities.
        msg: String, the string we are goint to store

        """
        if not len(self.messaggesList) >= self.replyAmount:
            rslg(self,f'Mensaje recibido: {msg}')
            self.messaggesList.append(msg)

    def publishStoredData(self,request,response):
        """This function  publish the data stored into the topic

        Parameters:

        self: Node, the node used to perform the activities.
        request: Dict, the request
        response:  Dict, the response
        """
        
        response.message = "Data replied succesfully in this execution"
        rslg(self,f'Replying Data')
        self.publisher_ = self.create_publisher(self.replyTopicType, self.replyTopic, 10)
        for i in self.messaggesList:
            rslg(self,f'Publishsing {i}')
            self.publisher_.publish(i)

        response.success = True
        response.message = response.message
        return response



def main():
    rclpy.init()
    myreplynode = replynode()

    rclpy.spin(myreplynode)

    rclpy.shutdown()
    