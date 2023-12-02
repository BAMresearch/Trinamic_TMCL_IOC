
import attr
from typing import Dict, Optional
from src.axis_parameters import AxisParameters
import socket
from typing import List

def validate_ip_address(instance, attribute, value):
    try:
        socket.inet_aton(value)
    except socket.error:
        raise ValueError(f"Invalid IP address: {value}")

def validate_port_number(instance, attribute, value):
    if not (0 <= value <= 65535):
        raise ValueError(f"Port number must be between 0 and 65535, got {value}")

def validate_board_module_id(instance, attribute, value):
    if not (0 <= value <= 255):
        raise ValueError(f"Board serial ID must be between 0 and 255, got {value}")

@attr.define
class BoardParameters:
    # Board-level configurable parameters
    board_configurable_parameters: Dict[int, int] = attr.field(factory=dict)

    # List of attribute names to be converted into PVs
    pv_attributes: List[str] = ['ip_address', 'port_number', 'board_module_id', 'board_configurable_parameters']

    # IP address of the board
    ip_address: str = attr.field(default = "192.168.0.253", validator=validate_ip_address, converter=str)

    # Port number for communication
    port_number: int = attr.field(default = "4016", validator=validate_port_number, converter=int)

    # Board serial ID
    board_module_id: int = attr.field(default="0", validator=validate_board_module_id, converter=int)

    # Store axis parameters in a list
    # axes_parameters: List[AxisParameters] = attr.Factory(lambda: [AxisParameters() for _ in range(6)])
    axes_parameters: List[AxisParameters] = attr.field(factory=list)
    # axes_parameters = attr.Factory(lambda: [AxisParameters() for _ in range(6)])
