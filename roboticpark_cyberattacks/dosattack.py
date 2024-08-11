import rclpy, sys
from rclpy.node import Node
from rclpy import Parameter

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

    Parameters
    ----------
    node: Node
        The node 
    """
    nodeNameList = node.get_node_names()
    print(nodeNameList)
    objectiveParam = node.get_parameter('dosobjective')._value
    if  objectiveParam == 'Unset':
        node.get_logger().info('dosobjective node has not been set')
        sys.exit()
    if objectiveParam not in nodeNameList:
        node.get_logger().info(f'Required node {objectiveParam} not present')
        sys.exit()

def main():
    rclpy.init()
    mydosnode = dosnode()
    printargs(mydosnode)
    checkobjective(mydosnode)
    rclpy.spin(mydosnode)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
