import os
import yaml
import sys
import codecs
from tempfile import NamedTemporaryFile
from ament_index_python.packages import get_package_share_directory

import xml.etree.ElementTree as ET


def GetPkgXMLPath():
  return  os.path.dirname(os.path.abspath(os.path.dirname(__file__)))+\
    "/package.xml"
##
# @brief Get the string of the current package name
#
# @return the current pack name
def GetModelPackageName():
  doc = ET.parse(GetPkgXMLPath())
  root = doc.getroot()
  # get & return of the package name ; by using the tag "name"
  return root.find("name").text

##
# @brief get Model Pkg yaml file parser instance
#
# @param ModelPkgName ex) "porter_seocho"
#
# @return Model Pkg yaml file parser instance
def GetModelConfig(ModelPkgName):
  ModelConfigFile=ModelPkgName+".yaml"
  ModelConfigPath=os.path.join(get_package_share_directory(ModelPkgName), 
          'config', ModelConfigFile)
  ModelConfig=yaml.load(codecs.open(ModelConfigPath,"r","utf8"),Loader=yaml.FullLoader)
  #print (":x: Model : "+ ModelConfig["model"])
  #print (":x: Robot : "+ ModelConfig["robot"])
  #print (":x: Map ; Init map : "+ ModelConfig["map"]["init"])
  #print (":x: Map ; Init pos : "+ str(ModelConfig["map"]["pos"]))
  #print (":x: Map ; available maps : "+ str(ModelConfig["map"]["list"]))
  return ModelConfig



##
# @brief Get RobotConfig pkg name ; ex) "cloi_robot_config"
#
# @param ModelPkgName ex) "porter_seocho"
#
# @return RobotConfig pkg name ex) "cloi_robot_config"
def GetRobotConfigPkg(ModelPkgName):
  ModelConfig        = GetModelConfig(ModelPkgName)
  RobotName          = ModelConfig["robot"] 
  return ModelConfig["RobotConfigPkg"]


##
# @brief get Robot context yaml parser instance
#
# @param ModelPkgName ex) "porter_seocho"
#
# @return Robot context yaml parser instance
def GetRobotConfig(ModelPkgName):
  RobotConfigPkgName = GetRobotConfigPkg(ModelPkgName)

  ModelConfig        = GetModelConfig(ModelPkgName)
  RobotName          = ModelConfig["robot"] 

  RobotConfigFile = RobotName+"_config.yaml"
  RobotConfigPath = os.path.join(get_package_share_directory(RobotConfigPkgName), 
                     'robot',RobotName, RobotConfigFile)
  RobotConfig     =yaml.load(codecs.open(RobotConfigPath,"r","utf8"),Loader=yaml.FullLoader)
  return RobotConfig


##
# @brief Get the abs path of the robot config path
#  ex) cloi2_ws/install/cloi_robot_config/share/cloi_robot_config/robot/porter_tx2/config
#
# @param ModelPkgName ex) "cloi_robot_config"
# @param RobotName    ex) "porter_tx2"
#
# @return absolute robot config path
def GetRobotConfigPath(ModelPkgName,RobotName):
  RobotConfigPackage = GetRobotConfigPkg(ModelPkgName) # cloi_robot_config

  return os.path.join(get_package_share_directory(RobotConfigPackage),\
          'robot',RobotName,'config/')

##
# @brief Get the abs path of the robot launch path
#  ex) cloi2_ws/install/cloi_robot_config/share/cloi_robot_config/robot/porter_tx2/launch
#
# @param ModelPkgName ex) "cloi_robot_config"
# @param RobotName    ex) "porter_tx2"
#
# @return absolute robot launch path
def GetRobotLaunchPath(ModelPkgName,RobotName):
  RobotConfigPackage = GetRobotConfigPkg(ModelPkgName) # cloi_robot_config

  return os.path.join(get_package_share_directory(RobotConfigPackage),\
          'robot',RobotName,'launch/')

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

def recursive_remap(config_dic, param_dic):
    for key in config_dic.keys():
        node_dic = config_dic[key]
        if 'ros__parameters' in node_dic.keys():
            rp_dic = node_dic['ros__parameters']
            for sub_key in param_dic.keys():
                for rp_key in rp_dic.keys():
                    if rp_key == sub_key:
                        rp_dic[rp_key] = param_dic[sub_key]
        else:
            recursive_remap(node_dic, param_dic)

def get_config_dict_with_remap(config_filename, param_dict):
    config_dict = {}
    # load config yaml
    with open(config_filename, 'r') as stream:
        config_dict = yaml.safe_load(stream)
    # remap parameters with param_dict
    recursive_remap(config_dict, param_dict)
    return config_dict

def modify_config_param_with_ns(namespace, config_filename, substitutions, param_dict):
    # modify config for namespace
    config_dict = get_config_dict_with_remap(config_filename, param_dict)
    return get_configured_params_with_ns(namespace, config_dict)

def get_configured_params_with_ns(namespace, config_dict):
    config_dict_with_ns = {}
    if namespace != '':
        # capsule config with namespace
        config_dict_with_ns[namespace] = config_dict
    else:
        config_dict_with_ns = config_dict
    # create temp file for config load
    return _create_params_file_from_dict(config_dict_with_ns)