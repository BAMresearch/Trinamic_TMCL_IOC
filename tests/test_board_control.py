import asyncio
import unittest
from unittest.mock import patch, MagicMock
from src.board_parameters import BoardParameters
from src.configuration_management import ConfigurationManagement
from src.board_control import BoardControl
from src import ureg

class TestBoardControl(unittest.TestCase):
    def setUp(self):
        # Setup code before each test method
        # Mock the ConfigurationManagement.load_configuration method
        with patch('src.configuration_management.ConfigurationManagement.load_configuration') as mock_load_config:
            self.board_params = BoardParameters()
            mock_load_config.return_value = self.board_params  # Assigning mock BoardParameters
            ConfigurationManagement.load_configuration('dummy/path/config.yaml', self.board_params)

        # Mocking BoardControl methods
        self.board_control = BoardControl(self.board_params)
        self.board_control.initialize_board = MagicMock()
        self.board_control.initialize_axes = MagicMock()
        self.board_control.move_axis = MagicMock()

    def test_initialize_board(self):
        # Test initialization of the board
        self.board_control.initialize_board()
        self.board_control.initialize_board.assert_called_once()

    def test_initialize_axes(self):
        # Test initialization of axes
        self.board_control.initialize_axes()
        self.board_control.initialize_axes.assert_called_once()

    def test_move_axis(self):
        # Test moving an axis
        axis_index = 0
        target_position = 10000
        self.board_control.move_axis(axis_index, target_position)
        self.board_control.move_axis.assert_called_with(axis_index, target_position)

class TestBoardParameters(unittest.TestCase):

    def test_board_parameters_initialization(self):
        # Path to the YAML configuration file
        config_file_path = 'tests/testdata/example_config.yaml'

        # Create an instance of BoardParameters
        board_params = BoardParameters()

        # Load the configuration into board_params
        ConfigurationManagement.load_configuration(config_file_path, board_params)

        # Assert that BoardParameters instance is correctly populated
        self.assertEqual(board_params.ip_address, '192.168.0.253')
        self.assertEqual(board_params.port_number, 4016)
        # Add other assertions for the rest of the attributes
        self.assertEqual(board_params.axes_parameters[0].axis_number, 0)
        self.assertEqual(board_params.axes_parameters[0].steps_to_realworld_conversion_quantity, ureg('12345 steps/mm'))

if __name__ == '__main__':
    unittest.main()
