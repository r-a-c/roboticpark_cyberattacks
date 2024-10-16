import importlib

def printargs(node):
    """Prints every param present in the node and its values

       Parameters
       ----------
       node: Node The node 
    """
    
    for i in node.get_parameters_by_prefix(''):
        pamname=i
        pamvalue=node.get_parameter(i)._value
        rslg(node,f'Name of parameter:{pamname}   Value of the parameter: {pamvalue}')

def rslg(node,msg):
    """Prints info into node's log

       Parameters
       ----------
       node: Node The node 
       msg: String, the message
    """
    node.get_logger().info(msg)

def import_message_type(node,msg_type_str):
    """This function searchs for the class object needed accoding to the params. 

       Parameters:

       node: Node, the node used to perform the activities.
       msg_type_str: String, the type of the messagge, as a string. 

       Returns:

       None if no class if found else the objective class.
    """
    try:
        package_name, msg_name = msg_type_str.split("/",1)
        msg, msg_name = msg_name.split("/",1)
        module = importlib.import_module(f"{package_name}.msg")
        return getattr(module, msg_name)
    except Exception as e:
        rslg(node,f"Problem with the type '{msg_type_str}': {e}")
        return None