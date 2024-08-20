import rclpy, sys
from rclpy.node import Node
from rclpy import Parameter
from std_srvs.srv import Empty
from rcl_interfaces.srv import GetParameters
from ros2node.api import get_node_names

class dosnode(Node):
    def __init__(self):
        super().__init__("dosnode")
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

def checkobjective(node):
    """Check the objective of the attack, passed by parameter by Ros
    ros2 run roboticpark_cyberattacks dosattack  --ros-args -p dosobjective:=loquesea
    It does not work to check the existance of a node because a Node is not able to check the full graph.
    It only sees a group of nodes in any interaction. 

    Parameters
    ----------
    node: Node
        The node 
    """
    #nodeNameList = node.get_node_names()
    nodeNameList = get_node_list2(node)
    print(nodeNameList)
    objectiveParam = node.get_parameter('dosobjective')._value
    if  objectiveParam == 'Unset':
        node.get_logger().info('dosobjective node has not been set')
        sys.exit()
    if objectiveParam not in nodeNameList:
        node.get_logger().info(f'Required node {objectiveParam} not present')
        sys.exit()

def get_node_list2(node):
    """ See checkobjective, this function could be a method for that function """
    node_names = get_node_names(node=node,include_hidden_nodes=False)
    return node_names

def get_node_list(node):
    """ See checkobjective, this function could be a method for that function """
    node_names = node.get_node_names_and_namespaces()
    return [name[0] for name in node_names]

def getNodeList(node):
    """ See checkobjective, this function could be a method for that function """
    auxList = []
    client = node.create_client(Empty, '/ros2/node/list')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting service /ros2/node/list...')
    
    request = Empty.Request()
    future = client.call_async(request)
    
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None:
        auxList = node.node_list 
    else:
        node.get_logger().error('No nodes in this graph!!')
        sys.exit()
    return auxList

def checkService(node):
    objectiveParam = node.get_parameter('dosobjective')._value
    client = node.create_client(GetParameters, f'/{objectiveParam}/get_parameters')
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().info(f'Not able to connect that node {objectiveParam}')
        sys.exit()

def main():
    rclpy.init()
    mydosnode = dosnode()
    printargs(mydosnode)
    # checkobjective(mydosnode)
    checkService(mydosnode)
    rclpy.spin(mydosnode)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
