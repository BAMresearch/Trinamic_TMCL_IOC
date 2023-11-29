from src.axis_parameters import AxisParameters
from src.board_parameters import BoardParameters
import pytrinamic
from pytrinamic.connections import ConnectionManager
from pytrinamic.modules import TMCM6214
import time
from . import pint
from . import ureg

class BoardControl:
    def __init__(self, boardpar:BoardParameters): # , connection_string:str = "--interface socket_serial_tmcl --port 192.168.0.253:4016 --host-id 3 --module-id 0"):
        connection_string = f"--interface socket_serial_tmcl --port {boardpar.ip_address}:{boardpar.port_number} --host-id 3 --module-id {boardpar.board_module_id}"
        self.connection_manager = ConnectionManager(connection_string)
        self.boardpar = boardpar
        self.module = boardpar.board_module_id

    def initialize_board(self):
        with self.connection_manager.connect() as myInterface:
            self.module = TMCM6214(myInterface, module_id=self.boardpar.board_module_id)
            for key, value in self.boardpar.board_configurable_parameters.items():
                self.module.set_global_parameter(key, value)
    
    def initialize_axis(self, axis_index:int):
        with self.connection_manager.connect() as myInterface:
            self.module = TMCM6214(myInterface)
            axpar=self.boardpar.axes_parameters[axis_index]
            for key, value in axpar.configurable_parameters.items():
                self.module.set_axis_parameter(axis_index, key, value)

    def initialize_axes(self):
        for axpar in self.boardpar.axes_parameters:
            self.initialize_axis(axpar.axis_number)

    def home_axis(self, axis_index:int):     
        with self.connection_manager.connect() as myInterface:
            self.module.reference_search(0, axis_index, self.module_id)
    
    def await_move_completion(self, axis_index:int):
        self.update_axis_parameters(axis_index)
        while (not self.axis_parameters[axis_index].is_position_reached_RBV) and (self.axis_parameters[axis_index].is_moving_RBV):
            time.sleep(0.5)
            self.update_axis_parameters(axis_index)

    def stop_axis(self, axis:int):
        """
        Stops the motor immediately on the given axis.

        Parameters:
        axis: Axis index.

        Returns: None
        """
        with self.connection_manager.connect() as myInterface:
            myInterface.stop(axis, self.module_id)
    
    def stop_all(self):
        """
        Stops all motors immediately.

        Returns: None
        """
        with self.connection_manager.connect() as myInterface:
            for axis in self.boardpar.axes_parameters:
                self.stop_axis(axis.axis_number)

    def move_axis(self, axis_index:int, position_steps:int):
        """
        Moves the motor on the given axis to the given target position. 
        Does not account for backlash or convert real-world units to steps. 

        Parameters:
        axis: Axis index.
        position: Target position to move the motor to. Units are module specific.
        Returns: None
        """
        with self.connection_manager.connect() as myInterface:
            myInterface.move_to(axis_index, position_steps, self.module_id)

    def update_axis_parameters(self, axis_index:int, axpars: AxisParameters):
        with self.connection_manager.connect() as myInterface:
            module = TMCM6214(myInterface, module_id=self.boardpar.board_module_id)
            axis = module.motors[axis_index]

            actual_step_coordinate_RBV = int(axis.get_axis_parameter(axis.AP.ActualPosition))
            target_step_coordinate_RBV = int(axis.get_axis_parameter(axis.AP.TargetPosition))

            is_moving_RBV = bool(axis.get_axis_parameter(axis.AP.ActualVelocity)!=0)
            is_position_reached_RBV = bool(axis.get_axis_parameter(axis.AP.PositionReachedFlag))

    # Add other necessary motor control functions