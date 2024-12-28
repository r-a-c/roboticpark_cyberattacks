import rclpy, sys, os
from rclpy.node import Node
import importlib
from std_srvs.srv import Empty
from rcl_interfaces.srv import GetParameters
from roboticpark_cyberattacks.utils import rslg,import_message_type
from rcl_interfaces.srv import *
from datetime import datetime


class logcapturer(Node):
    def __init__(self):
        super().__init__("logcapturer")
        self.declare_parameter('topic_to_log', "Unset")
        self.declare_parameter('topic_to_log_type', "Unset")

        # Create Log File
        topic_name = self.get_parameter('topic_to_log')._value
        log_file_name = f"{topic_name}_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
        log_file_name = log_file_name.replace("/","_")
        self.log_file_path = os.path.join(os.getcwd(), log_file_name)
        rslg(self,f" Writing info from '{topic_name}' in {self.log_file_path}")
    
    def loggerToFile(self,msg):
        with open(self.log_file_path, 'a') as log_file:
            log_file.write(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] {msg}\n")
    #        rslg(self,f"Mensaje recibido: {msg}")

def subscribe_to_topic(node,topic,msg_type):
    """This function  subscribes a node to a topic

       Parameters:
        node: Node, the node used to perform the activities.
        topic: String, the topic to subscribe.
        msg_type: Type of the messagge will be subscribed.

    """
    node.subscription = node.create_subscription( msg_type, topic, node.loggerToFile, 10)

def main():
    rclpy.init()
    mylogcapturer = logcapturer()

    # Obtain values passed by parameters.
    topicToLog = mylogcapturer.get_parameter('topic_to_log')._value
    topicToLogType = mylogcapturer.get_parameter('topic_to_log_type')._value

    if topicToLog == 'Unset' or topicToLogType == 'Unset' :
        rslg(mylogcapturer,'No topic or topic Type specified')
        sys.exit()

    topicType = import_message_type(mylogcapturer,topicToLogType)
    
    if topicType == None:
        rslg(mylogcapturer,'Problem obtaining the type of the topic')
        sys.exit()

    try:
        subscribe_to_topic(mylogcapturer,topicToLog,topicType)
    except Exception as e:
        rslg(logcapturer,f"Some kind of problem subscribing the topic {e}")
        sys.exit()



    rclpy.spin(mylogcapturer)
    rclpy.shutdown()

    
