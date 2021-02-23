#
# LGE Advanced Robotics Laboratory
# Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
# All Rights are Reserved.
#
# SPDX-License-Identifier: MIT
#
import os
import sys

from collections import OrderedDict
from typing import List
from typing import Text
from typing import Tuple
from typing import Dict

from launch.substitutions import LaunchConfiguration

import yaml
import tempfile
import launch

class DictItemReference:
    def __init__(self, dictionary, key):
        self.dictionary = dictionary
        self.dictKey = key

    def key(self):
        return self.dictKey

    def setValue(self, value):
        self.dictionary[self.dictKey] = value

class RewrittenYaml(launch.Substitution):
    """
    Substitution that modifies the given Yaml file.
    """

    def __init__(self,
        source_file: launch.SomeSubstitutionsType,
        rewrites: Dict,
        convert_types = False) -> None:
        super().__init__()

        from launch.utilities import normalize_to_list_of_substitutions  # import here to avoid loop
        self.__source_file = normalize_to_list_of_substitutions(source_file)
        self.__rewrites = {}
        self.__convert_types = convert_types
        for key in rewrites:
            self.__rewrites[key] = normalize_to_list_of_substitutions(rewrites[key])

    @property
    def name(self) -> List[launch.Substitution]:
        """Getter for name."""
        return self.__source_file

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return ''

    def perform(self, context: launch.LaunchContext) -> Text:
        yaml_filename = launch.utilities.perform_substitutions(context, self.name)
        rewritten_yaml = tempfile.NamedTemporaryFile(mode='w', delete=False)
        resolved_rewrites = self.resolve_rewrites(context)
        data = yaml.safe_load(open(yaml_filename, 'r'))
        self.substitute_values(data, resolved_rewrites)
        yaml.dump(data, rewritten_yaml)
        rewritten_yaml.close()
        return rewritten_yaml.name

    def resolve_rewrites(self, context):
        resolved = {}
        for key in self.__rewrites:
            resolved[key] = launch.utilities.perform_substitutions(context, self.__rewrites[key])
        return resolved

    def substitute_values(self, yaml, rewrites):
        for key in self.getYamlKeys(yaml):
            if key.key() in rewrites:
                raw_value = rewrites[key.key()]
                key.setValue(self.convert(raw_value))

    def getYamlKeys(self, yamlData):
        try:
            for key in yamlData.keys():
                for k in self.getYamlKeys(yamlData[key]):
                    yield k
                yield DictItemReference(yamlData, key)
        except AttributeError:
            return

    def convert(self, text_value):
        if self.__convert_types:
            # try converting to int
            try:
                return int(text_value)
            except ValueError:
                pass

            # try converting to float
            try:
                return float(text_value)
            except ValueError:
                pass

            # try converting to bool
            if text_value.lower() == "true":
                return True
            if text_value.lower() == "false":
                return False

            #nothing else worked so fall through and return text
        return text_value

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

def parse_launch_arguments(launch_arguments: List[Text]) -> List[Tuple[Text, Text]]:
    """Parse the given launch arguments from the command line, into list of tuples for launch."""
    parsed_launch_arguments = OrderedDict()  # type: ignore
    for argument in launch_arguments:
        count = argument.count(':=')
        if count == 0 or argument.startswith(':=') or (count == 1 and argument.endswith(':=')):
            pass
        else:
            name, value = argument.split(':=', maxsplit=1)
            parsed_launch_arguments[name] = value  # last one wins is intentional
    return parsed_launch_arguments

def get_env_values_dict(rewritten_list):
    result_env_dict = {}
    for key in rewritten_list:
        if key in os.environ.keys():
            env_value = os.environ[key]
            if env_value != None:
                result_env_dict[key] = env_value
        if key.upper() in os.environ.keys():
            env_value = os.environ[key.upper()]
            if env_value != None:
                result_env_dict[key] = env_value
    return result_env_dict


def get_configured_params(config_filename, rewritten_list):
    parsed_args_dict = parse_launch_arguments(sys.argv)
    env_dict = get_env_values_dict(rewritten_list)
    param_substitutions = {}

    for param_name in rewritten_list:
        if param_name in env_dict.keys():
            param_substitutions[param_name] = env_dict[param_name]
        if param_name in parsed_args_dict.keys():
            param_substitutions[param_name] = parsed_args_dict[param_name]

    return RewrittenYaml(
        source_file=config_filename, rewrites=param_substitutions,
        convert_types=True)

    