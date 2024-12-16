import importlib, time, random

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

def fill_msg_with_random_values(msg,double_range_start,double_range_end,int_range_start,int_range_end,frame_id_string):
    """
    Finds numeric values in a msg and edits them according to params's range, randomly. Also sets frame to "map"

    Parameters:
    - msg: ROS2 message, any
    - double_range_start,double_range_end: range for double values
    - int_range_start,int_range_end: range for int values
    - frame_id_string: frame id string
    """
    def is_numeric_simple(ros_type):
        numeric_types = {
            'float32', 'float64', 'int8', 'int16', 'int32', 'int64', 
            'uint8', 'uint16', 'uint32', 'uint64'
        }
        return ros_type in numeric_types
    
    def is_numeric_complex(ros_type):
        numeric_types = {
            'double'
        }
        return ros_type in numeric_types


    for field_name, field_type in msg._fields_and_field_types.items():
        field_value = getattr(msg, field_name)
        # with open("test.txt", "a") as myfile:
        #         myfile.write(f"Types {field_name}: {field_type} \n")
        #         myfile.flush()
        
        if field_name == "frame_id":
            setattr(msg, field_name, frame_id_string)
        if is_numeric_simple(field_type):
            setattr(msg, field_name, random.randint(int_range_start,int_range_end))  
        if is_numeric_complex(field_type):
            setattr(msg, field_name, random.uniform(double_range_start,double_range_end))   
        if field_name == "sec" or field_name == "nanosec":
            setattr(msg,field_name,int(time.time()))
        
        elif '/' in field_type:
            setattr(msg, field_name, fill_msg_with_random_values(field_value,double_range_start,double_range_end,int_range_start,int_range_end,frame_id_string))
    
    return msg

