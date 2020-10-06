import os
import sys
import yaml
from tempfile import NamedTemporaryFile
import launch.actions
import launch_ros.actions
import lifecycle_msgs.msg

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def _create_params_file_from_dict(params):
    with NamedTemporaryFile(mode='w', prefix='launch_params_', delete=False) as h:
        param_file_path = h.name
        yaml.dump(params, h, default_flow_style=False)
        return param_file_path

def get_robot_name_in_arg():
    robot_name = ''
    # find ros param setting template
    for idx in range(4, len(sys.argv)):
        tmp_arg = sys.argv[idx]
        tmp_sp = tmp_arg.split(':=')
        # if argument is ros param template
        if len(tmp_sp) == 2:
            tmp_key = tmp_sp[0]
            tmp_value = tmp_sp[1]
            # check param name is robot_name
            if str(tmp_key) == 'robot_name' or str(tmp_key) == '_robot_name':
                # set namespace with robot_name
                robot_name = tmp_value
    return robot_name

def get_robot_name_in_env():
    robot_name = ''
    # check environment param
    if 'ROBOT_NAME' in os.environ.keys():
        # if environment param name has ROBOT_NAME
        env_robot_name = os.environ['ROBOT_NAME']
        if env_robot_name != None:
        # set namespace with ROBOT_NAME
            robot_name = env_robot_name
    return robot_name

def find_robot_name():
    robot_name = ''
    argument_count = len(sys.argv)
    # if exe argument count over 4
    if argument_count > 4:
        robot_name = get_robot_name_in_arg()
    # if robot_name still not exist 
    if robot_name == '':
        robot_name = get_robot_name_in_env()
    return robot_name

def set_remapping_list(namespace, remap_topic_list):
    result_remapping_list = []
    # set topic remapping prefix
    ns_for_remap = namespace
    if namespace != '':
        ns_for_remap = '/' + namespace

    # set topic remap list with prefix
    for remap_topic in remap_topic_list:
        result_remapping_list.append(tuple([remap_topic, ns_for_remap+remap_topic]))
    return result_remapping_list

def get_modify_config_dict(namespace, config_filename, modify_model_node_list):
    # load config yaml
    config_dict = {}
    with open(config_filename, 'r') as stream:
        config_dict = yaml.safe_load(stream)

    # find need to modify model tuple in dict
    for node_name in modify_model_node_list:
        # search correct tuple in config_dict
        for node_key in config_dict.keys():
            if node_name == node_key:
                if not 'ros__parameters' in config_dict[node_key].keys():
                    config_dict[node_key]['ros__parameters'] = {}
                if not 'sim' in config_dict[node_key]['ros__parameters']:
                    config_dict[node_key]['ros__parameters']['sim'] = {}
                if not 'model' in config_dict[node_key]['ros__parameters']['sim']:
                    config_dict[node_key]['ros__parameters']['sim']['model'] = {}
                config_dict[node_key]['ros__parameters']['sim']['model'] = namespace
                config_dict[node_key]['ros__parameters']['use_sim_time'] = True
    return config_dict

def modify_config_param_with_ns(namespace, config_filename, modify_model_node_list):
    result_config_param = config_filename
    # modify config for namespace
    if namespace != '':
        # get_modify_config_dict
        config_dict = get_modify_config_dict(namespace, config_filename, modify_model_node_list)
        config_dict_with_ns = {}
        # capsule config with namespace
        config_dict_with_ns[namespace] = config_dict
        # create temp file for config load
        result_config_param = _create_params_file_from_dict(config_dict_with_ns)
    return result_config_param

def generate_launch_description():
    # Set remapping topic list
    remap_topic_list = ['/tf', '/tf_static']

    # Set nodename need to modify model with robot_name
    modify_model_node_list = []

    # Get the launch directory
    config_dir = os.path.join(get_package_share_directory("cloud_bridge"), 'config')

    # Get config filename
    # config_filename = os.path.join(config_dir, 'params.omnimapper.yaml')
    config_filename = os.path.join(config_dir, 'server.yaml')
    param_filename = os.path.join(config_dir, 'params.yaml')
    
    # Get namespace in argument
    namespace = find_robot_name()
    
    print ('========================================')
    print ('RobotName11 : '+ str(namespace))
    print ('config_filename : '+ str(config_filename))
    print ('========================================')
    # Set remapping topic tuple list
    remapping_list = set_remapping_list(namespace, remap_topic_list)

    # modify config param with namespace
    config_params = modify_config_param_with_ns(namespace, config_filename, modify_model_node_list)
    
    # Create environment variables
    stdout_linebuf_envvar = launch.actions.SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    # Create actions nodes
    # Create micom_driver_sim node
    cloud_trans_cloud = launch_ros.actions.Node(
        package='cloud_bridge',
        node_executable='cloud_bridge_server',
        node_namespace=namespace,
        remappings=remapping_list,
        parameters=[config_filename, param_filename],
        output='screen')

    # Create the launch description and populate
    ld = launch.LaunchDescription()

    ld.add_action(cloud_trans_cloud)


    return ld
