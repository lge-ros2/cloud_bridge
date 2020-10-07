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
    env_robot_name = get_robot_name_in_env()
    robot_name = LaunchConfiguration('robot_name', default=env_robot_name)
    return robot_name

def set_remapping_list(remap_topic_list):
    result_remapping_list = []

    # set topic remap list with prefix
    for remap_topic in remap_topic_list:
        result_remapping_list.append(tuple(['/'+remap_topic, remap_topic]))
    return result_remapping_list

def generate_launch_description():
    # Set remapping topic list
    remap_topic_list = ['tf', 'tf_static']

    # Get the launch directory
    config_dir = os.path.join(get_package_share_directory("cloud_bridge"), 'config')

    # Get config filename
    # config_filename = os.path.join(config_dir, 'params.omnimapper.yaml')
    config_filename = os.path.join(config_dir, 'server.yaml')
    param_filename = os.path.join(config_dir, 'params.yaml')
    
    # Get namespace in argument
    namespace = find_robot_name()
    
    # Set remapping topic tuple list
    remapping_list = set_remapping_list(remap_topic_list)
  
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
