import os
from typing import Optional

from .constants import RobotHardware, RobotType, RobotConfiguration


def get_robot_name() -> Optional[str]:
    return os.environ.get('VEHICLE_NAME', None)


def get_robot_hardware() -> RobotHardware:
    rhardware = os.environ.get('ROBOT_HARDWARE', '__NOTSET__')
    return RobotHardware.from_string(rhardware)


def get_robot_type() -> RobotType:
    rtype = os.environ.get('ROBOT_TYPE', '__NOTSET__')
    return RobotType.from_string(rtype)


def get_robot_configuration() -> RobotConfiguration:
    rcfg = os.environ.get('ROBOT_CONFIGURATION', 'UNKNOWN')
    return RobotConfiguration.from_string(rcfg)
