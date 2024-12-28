import rclpy, sys, os
from rclpy.node import Node, NodeNameNonExistentError
from roboticpark_cyberattacks.utils import rslg, printargs, import_message_type
from rcl_interfaces.srv import *
from std_srvs.srv import Trigger
import pickle
from asyncio_run_in_process import run_in_process,open_in_process
from multiprocessing import Pool


class replynode(Node):

    replyTopic = None
    replyTopicType = None
    replyTopicTypeValue = None
    messaggesList = []
    replyAmount = 0
    logFile = ""


    def __init__(self):
        super().__init__("replynode")
        self.declare_parameter('reply_topic', "Unset")
        self.declare_parameter('reply_topic_type', "Unset")
        self.declare_parameter('reply_amount', "Unset")
        self.declare_parameter('reply_log_file_path', "reply_log_file.txt")
        self.replyTopic = self.get_parameter('reply_topic').get_parameter_value().string_value
        self.replyTopicTypeValue = self.get_parameter('reply_topic_type').get_parameter_value().string_value
        self.replyAmount = int(self.get_parameter('reply_amount').get_parameter_value().string_value)
        self.logFile = self.get_parameter('reply_log_file_path').get_parameter_value().string_value
        printargs(self)
        
        # Create replication service
        serviceName = f'{self.get_name()}/data_save'
        self.srv = self.create_service(Trigger, serviceName, self.save_messagges)

        # Create replication service
        serviceName = f'{self.get_name()}/data_replicate'
        self.srv = self.create_service(Trigger, serviceName, self.publishStoredData)

        # Create replication service from file
        serviceName = f'{self.get_name()}/data_replicate_from_file'
        self.srv = self.create_service(Trigger, serviceName, self.publishStoredDataFromFile)

        # Create save to file service
        serviceName = f'{self.get_name()}/data_save_to_file'
        self.srv = self.create_service(Trigger, serviceName, self.save_to_file)

        if self.replyAmount == 'Unset' or self.replyTopic == 'Unset' or self.replyTopicTypeValue == 'Unset':
            rslg(self,f'No reply_topic, reply_amount or reply_topic_type passed as parameter')
            sys.exit()

        self.replyTopicType = import_message_type(self,self.replyTopicTypeValue)


    def save_messagges(self, request,response):
        """This function  save messagess from the selected topic to a list in memory
        It creates a subscription to a topic, and then stores the needed amount of messagges

        Parameters:
            request: Dict, the request
            response:  Dict, the response

        """
        response.message = f"Data copied to list"
        rslg(self,f'Copying messagges to list ')
        self.subscription = self.create_subscription(self.replyTopicType,self.replyTopic,self.save_messagges_aux,10)
        response.success = True
        response.message = response.message
        return response

    def save_messagges_aux(self, msg):
        """This function  save a requested amount of messagges to a list

        Parameters:
            msg: String, the string we are goint to store

        """
        if not len(self.messaggesList) >= self.replyAmount:
            rslg(self,f'Mensaje recibido: {msg}')
            self.messaggesList.append(msg)

    def save_to_file(self, request,response):
        """This function  save a the list messaggesList to a file

        Parameters:
            request: Dict, the request
            response:  Dict, the response

        """
        response.message = f"Data copied to file {self.logFile}"
        rslg(self,f'Copying to file')
        with open(self.logFile, "wb") as archivo:
            pickle.dump(self.messaggesList, archivo)
        
        response.success = True
        response.message = response.message
        return response


    def publishStoredData(self,request,response):
        """This function  publish the data stored into the topic

        Parameters:
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
    
    def publishStoredDataFromFile(self,request,response):
        """This function  publish the data stored into the topic, using a file as a source

        Parameters:
            request: Dict, the request
            response:  Dict, the response

        """
        
        response.message = "Data replied succesfully in this execution, source is a file"
        rslg(self,f'Replying Data from a file')
        self.publisher_ = self.create_publisher(self.replyTopicType, self.replyTopic, 10)

        with open(self.logFile, "rb") as archivo:
            localLogFileList= pickle.load(archivo)

        for i in localLogFileList:
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
    