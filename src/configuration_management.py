import logging
from pathlib import Path
import yaml
import attr
from src.axis_parameters import AxisParameters
from src.board_parameters import BoardParameters
from typing import Any

class ConfigurationManagement:
    @staticmethod
    def load_configuration(config_file: Path, board_parameters: BoardParameters) -> None:
        """
        Load the configuration from a YAML file and update the BoardParameters instance.

        :param config_file: Path to the YAML configuration file.
        :param board_parameters: Instance of BoardParameters to be updated.
        """
        with open(config_file, 'r') as file:
            config = yaml.safe_load(file)

        logging.debug(f'Loading board configuration: {config.get("board", {})}')
        ConfigurationManagement._update_board_parameters(config.get('board', {}), board_parameters)
        logging.debug(f'Loading axis configuration: {config.get("axes", {})}')
        ConfigurationManagement._update_axes_parameters(config.get('axes', []), board_parameters)

    @staticmethod
    def _update_board_parameters(board_config, board_parameters) -> None:
        """
        Sets the board parameters based on the board_config dict loaded from the YAML file.
        """
        for key, value in board_config.items():
            if key=='board_configuration_file':
                continue # don't load this one
            if hasattr(board_parameters, key):
                setattr(board_parameters, key, value)

    @staticmethod
    def _update_axes_parameters(axes_config, board_parameters) -> None:
        """
        Updates the axes parameters for all axes based on the axes_config dict loaded from the YAML file.
        """
        for axis_config in axes_config:
            axis_number = axis_config.get('axis_number') 
            if (axis_number is not None) and (axis_number >= 0) and len(board_parameters.axes_parameters) < (axis_number+1): # axis_number can start at 0
                board_parameters.axes_parameters += [AxisParameters()]
            if axis_number is not None and 0 <= axis_number < len(board_parameters.axes_parameters):
                ConfigurationManagement._update_axis_parameters(axis_config, board_parameters.axes_parameters[axis_number])

    @staticmethod
    def _update_axis_parameters(axis_config, axis_parameters: AxisParameters) -> None:
        """
        Updates the axisparameters with the values in the axis_config dict loaded from the YAML file.
        """
        for key, value in axis_config.items():
            if hasattr(axis_parameters, key):
                setattr(axis_parameters, key, value)
                
    @staticmethod
    def save_configuration(config_file: Path, board_parameters: BoardParameters) -> None:
        """
        Save the current state of BoardParameters to a YAML configuration file.

        :param config_file: Path to the YAML configuration file to be written.
        :param board_parameters: Instance of BoardParameters to be saved.
        """
        config = {
            'board': attr.asdict(board_parameters, filter=lambda attr, value: attr.name != 'axes_parameters'),
            'axes': [attr.asdict(axis_param) for axis_param in board_parameters.axes_parameters]
        }
        with open(config_file, 'w') as file:
            yaml.dump(config, file)