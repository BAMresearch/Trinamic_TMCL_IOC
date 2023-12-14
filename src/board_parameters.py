
import logging
from pathlib import Path
import attr
from typing import Dict, Optional, Union
from src.axis_parameters import AxisParameters
import socket
from typing import List
import pytrinamic.modules
from attr import validators

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

def pytrinamic_module_converter(module_or_str:Union[None, str, pytrinamic.modules.TMCLModule]) -> Union[None, pytrinamic.modules.TMCLModule]:
    """Converts an input to a PyTrinamic module if it exists, or throw an error."""
    if module_or_str is None:
        return module_or_str # nothing that can be done
    # bug in pytrinamic makes this fail:
    elif isinstance(module_or_str, pytrinamic.modules.TMCLModule): 
        return module_or_str
    # but this works...
    elif isinstance(module_or_str, type): 
        return module_or_str
    elif isinstance(module_or_str, str):
        module = getattr(pytrinamic.modules, module_or_str, None)
        assert module is not None, logging.error(f"Board module type {module_or_str} not found in PyTrinamic modules library (must be e.g. 'TMCM6214')")
        return module
    else:
        logging.warning(f"input to pytrinamic_module_converter must be either None, str, or pytrinamic module. Got {module_or_str} which is type {type(module_or_str)}")
        return None

@attr.define
class BoardParameters:
    """
    Some basic configuration parameters pertaining to the board. This contains the axes parameters for this board as well.
    """
    # configuration file
    board_configuration_file: Path = attr.field(factory=Path)

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

    # Board model, or rather which module to load from PyTrinamic, needs to be set on config load. Due to bug in pytrinamic, modules report as instance of type...
    pytrinamic_module: Optional[pytrinamic.modules.TMCLModule] = attr.field(default=None, validator=validators.optional(validators.instance_of((type, pytrinamic.modules.TMCLModule))), converter=pytrinamic_module_converter)

    # Store axis parameters in a list
    # axes_parameters: List[AxisParameters] = attr.Factory(lambda: [AxisParameters() for _ in range(6)])
    axes_parameters: List[AxisParameters] = attr.field(factory=list)
    # axes_parameters = attr.Factory(lambda: [AxisParameters() for _ in range(6)])
