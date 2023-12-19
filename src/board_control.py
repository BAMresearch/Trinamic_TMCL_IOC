import asyncio
import logging
from typing import List, Tuple, Union
from src.axis_parameters import AxisParameters
from src.board_parameters import BoardParameters
import pytrinamic
from pytrinamic.connections import ConnectionManager
# module already loaded in board_params.pytrinamic_module
# from pytrinamic.modules import TMCM6214 
from caproto.server.records import MotorFields, pvproperty

from src.epics_utils import update_epics_motorfields_instance

class BoardControl:
    """low-level commands to communicate with the board, addressing basic board funccionalities"""
    last_board_tick_timer:int = 0

    def __init__(self, boardpar:BoardParameters) -> None: # , connection_string:str = "--interface socket_serial_tmcl --port 192.168.0.253:4016 --host-id 3 --module-id 0"):
        connection_string = f"--interface socket_serial_tmcl --port {boardpar.ip_address}:{boardpar.port_number} --host-id 3 --module-id {boardpar.board_module_id}"
        self.connection_manager = ConnectionManager(connection_string)
        self.boardpar = boardpar
        self.module_id = boardpar.board_module_id
        self.module = None

    def initialize_board(self) -> None:
        """
        Initializes the board with the parameters from the BoardParameters instance. Sets the global parameters on the board.
        """
        with self.connection_manager.connect() as myInterface:
            self.module = self.boardpar.pytrinamic_module(myInterface, module_id=self.boardpar.board_module_id)
            for key, value in self.boardpar.board_configurable_parameters.items():
                logging.info(f"setting board {key=} {value=}")
                self.module.set_global_parameter(key, 0, value) # these are automatically stored
    
    def initialize_axis(self, axis_index:int) -> None:
        """
        Initializes a single axis with the parameters from the AxisParameters instance. Sets the axis parameters on the board.
        """
        axpar=self.boardpar.axes_parameters[axis_index]
        with self.connection_manager.connect() as myInterface:
            self.module = self.boardpar.pytrinamic_module(myInterface, module_id=self.boardpar.board_module_id)
            for key, value in axpar.configurable_parameters.items():
                logging.info(f"setting {axis_index=} {key=} {value=}")
                self.module.set_axis_parameter(key, axis_index, value)
        self.update_board_parameters_from_axis_parameters(axis_index)

    def update_board_parameters_from_axis_parameters(self, axis_index:int) -> None:
        """
        Updates the board parameters from the axis parameters, useful for example after getting updated parameters from EPICS. Sets the global parameters on the board.
        """
        axpar=self.boardpar.axes_parameters[axis_index]
        self.set_velocity_in_microsteps_per_second_on_board(axis_index, axpar.velocity_in_microsteps_per_second())
        self.set_acceleration_in_microsteps_per_second_squared_on_board(axis_index, axpar.acceleration_in_microsteps_per_second_squared())
        # self.set_axis_inversion_on_board(axis_index) 
        # not sure we need to also swap limit switches, but probably... if not, fix the logic in this method:
        # self.set_swapped_limit_switches_on_board(axis_index)

    def initialize_axes(self) -> None:
        """
        Initializes all axes on the board with the parameters from the BoardParameters instance.
        """
        for axpar in self.boardpar.axes_parameters:
            self.initialize_axis(axpar.axis_number)

    async def check_if_powercycle_occurred(self) -> None:
        # check the tick timer and see if its value is lower than the previous one. 
        with self.connection_manager.connect() as myInterface:
            self.module = self.boardpar.pytrinamic_module(myInterface, module_id=self.boardpar.board_module_id)
            new_board_tick_timer = self.module.get_global_parameter(self.module.GP0.TickTimer, 0)
        if new_board_tick_timer < self.last_board_tick_timer:
            raise RuntimeError("Board tick-timer didn't move forwards anymore, probably power reset occurred.")
        if new_board_tick_timer > 1500000000: # reset, we've only a few days left before it rolls over
            self.last_board_tick_timer = 0
            self.module.set_global_parameter(self.module.GP0.TickTimer, 0, 0)
        self.last_board_tick_timer = new_board_tick_timer

    def set_axis_single_parameter(self, axis_index:int, parameter_string:str, value: int) -> None:
        self.set_axis_parameters(axis_index, [(parameter_string, value)])

    def get_axis_single_parameter(self, axis_index:int, parameter_string:str) -> None:
        with self.connection_manager.connect() as myInterface:
            self.module = self.boardpar.pytrinamic_module(myInterface, module_id=self.boardpar.board_module_id)
            axis = self.module.motors[axis_index]
            parameter = getattr(axis.AP, parameter_string, None)
            if parameter is None: 
                logging.warning(f'Tried to set axis parameter with name {parameter_string}, but could not find it in the Trinamic axis parameter (axis.AP) model')
            else:
                axis.get_axis_parameter(parameter)

    def set_axis_parameters(self, axis_index:int, parval_list: List[Tuple[str, int]]) -> None:
        """
        Convenience function when you have to set a single axis parameter from somewhere else. 
        parval_list is a list of (parameter_string, value) tuples. 
        parameters_string should be an existing axis parameter name, such as MaxVelocity. 
        value must be an int.
        """
        with self.connection_manager.connect() as myInterface:
            self.module = self.boardpar.pytrinamic_module(myInterface, module_id=self.boardpar.board_module_id)
            axis = self.module.motors[axis_index]
            for parval in parval_list:
                parameter_string, value = parval # unpack
                assert isinstance(value, int), logging.error(f'calls to board_control.set_axis_parameter should have a parameter value that is an int. Got {type(value)=} instead for {parameter_string=}')
                parameter = getattr(axis.AP, parameter_string, None)
                if parameter is None: 
                    logging.warning(f'Tried to set axis parameter with name {parameter_string}, but could not find it in the Trinamic axis parameter (axis.AP) model')
                else:
                    axis.set_axis_parameter(parameter, value)

    def set_velocity_in_microsteps_per_second_on_board(self, axis_index:int, velocity_in_microsteps_per_second:int) -> None:
        """
        sets the velocity in microsteps per second for the given axis. Sends it to the board.
        """
        self.set_axis_single_parameter(axis_index, 'MaxVelocity', velocity_in_microsteps_per_second)
        # with self.connection_manager.connect() as myInterface:
        #     self.module = self.boardpar.pytrinamic_module(myInterface, module_id=self.boardpar.board_module_id)
        #     axis = self.module.motors[axis_index]
        #     axis.set_axis_parameter(axis.AP.MaxVelocity, velocity_in_microsteps_per_second)

    def set_acceleration_in_microsteps_per_second_squared_on_board(self, axis_index:int, acceleration_in_microsteps_per_second_squared:int) -> None:
        """
        sets the acceleration in microsteps per second squared for the given axis. Sends it to the board.
        """
        self.set_axis_parameters(
            axis_index, [
                ('MaxAcceleration', acceleration_in_microsteps_per_second_squared),
                ('MaxDeceleration', acceleration_in_microsteps_per_second_squared)
            ]
        )
        # with self.connection_manager.connect() as myInterface:
        #     self.module = self.boardpar.pytrinamic_module(myInterface, module_id=self.boardpar.board_module_id)
        #     axis = self.module.motors[axis_index]
        #     axis.set_axis_parameter(axis.AP.MaxAcceleration, acceleration_in_microsteps_per_second_squared)
        #     # decelerate as quick as acceleration
        #     axis.set_axis_parameter(axis.AP.MaxDeceleration, acceleration_in_microsteps_per_second_squared)

    def get_end_switch_distance(self, axis_index:int) -> int:
        """
        Returns the distance between the end switches in steps.
        """
        with self.connection_manager.connect() as myInterface:
            self.module = self.boardpar.pytrinamic_module(myInterface, module_id=self.boardpar.board_module_id)
            return self.module.get_axis_parameter(self.module.motors[axis_index].AP.RightLimitSwitchPosition, axis_index) # limit switch distance in steps. 

    def home_axis(self, axis_index:int) -> None:
        """
        Homes the motor on the given axis. 
        """
        with self.connection_manager.connect() as myInterface:
            self.module = self.boardpar.pytrinamic_module(myInterface, module_id=self.boardpar.board_module_id)
            # self.module.reference_search(0, axis_index, self.boardpar.board_module_id)
            myInterface.reference_search(0, axis_index, self.boardpar.board_module_id)
    
    def check_if_moving(self, axis_index:int) -> bool:
        """
        Checks if the motor on the given axis is moving by checking the (ideally updated) axis parameters.
        """
        self.update_axis_parameters(axis_index)
        return bool((not self.boardpar.axes_parameters[axis_index].is_position_reached_RBV) and (self.boardpar.axes_parameters[axis_index].is_moving_RBV))

    async def await_move_completion(self, axis_index:int, instance:Union[pvproperty, None]=None) -> None:
        """
        Waits until the motor on the given axis has completed its motion. Updates the axis parameters and the EPICS fields.
        """
        axpar = self.boardpar.axes_parameters[axis_index]
        doublecheck = 0 # doublecheck that the motor is not moving anymore.
        if instance is not None:
            EPICS_fields: MotorFields = instance.field_inst

        while doublecheck < 2:
            doublecheck += int(not(self.check_if_moving(axis_index)))
            await asyncio.sleep(self.boardpar.axes_parameters[axis_index].update_interval_moving)
            self.update_axis_parameters(axis_index)
            if instance is not None:
                if EPICS_fields.stop.value == 1 or EPICS_fields.stop_pause_move_go.value == 'Stop':
                    self.stop_axis(axis_index)
                    axpar.is_move_interrupted = True
                    logging.warning(f"Motion interrupted by {EPICS_fields.stop.value=} and/or {EPICS_fields.stop_pause_move_go.value=}.")
                    break
                await update_epics_motorfields_instance(axpar, instance, moving_or_nonmoving='moving')

            if axpar.is_move_interrupted:
                self.stop_axis(axis_index) # stop the motor motion immediately
                logging.warning("Motion interrupted by limit switch or stop command.")
                break

        if instance is not None:
            # we can reset the stop flag. 
            EPICS_fields = instance.field_inst
            await EPICS_fields.stop.write(0)
            # await EPICS_fields.stop_pause_move_go.write('Go')

        # if we didn't break out of the loop, the motion is complete. in case of imperfect movement, update target position to actual. 
        # if instance is not None:
        #     await update_epics_motorfields_instance(axpar, instance, moving_or_nonmoving='nonmoving')
            # await EPICS_fields.user_readback_value.write(axpar.actual_coordinate_RBV.to(axpar.base_realworld_unit).magnitude)
            
    def stop_axis(self, axis:int):
        """
        Stops the motor immediately on the given axis.

        Parameters:
        axis: Axis index.

        Returns: None
        """
        with self.connection_manager.connect() as myInterface:
            myInterface.stop(axis, self.boardpar.board_module_id)
    
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
            myInterface.move_to(axis_index, position_steps, self.boardpar.board_module_id)

    def update_axis_parameters(self, axis_index:int):
        axpars=self.boardpar.axes_parameters[axis_index]
        with self.connection_manager.connect() as myInterface:
            module = self.boardpar.pytrinamic_module(myInterface, module_id=self.boardpar.board_module_id)
            axis = module.motors[axis_index]

            axpars.actual_coordinate_RBV = axpars.raw_to_user(int(axis.get_axis_parameter(axis.AP.ActualPosition, signed=True)))
            # don't think I need this, but it won't hurt.:
            axpars.target_coordinate_RBV = axpars.raw_to_user(int(axis.get_axis_parameter(axis.AP.TargetPosition, signed=True)))

            axpars.is_moving_RBV = bool(axis.get_axis_parameter(axis.AP.ActualVelocity)!=0)
            axpars.is_position_reached_RBV = bool(axis.get_axis_parameter(axis.AP.PositionReachedFlag))
            if axpars.invert_limit_values:
                # not sure right=negative and left=positive. TODO: needs checking - nope, reverse. is fixed now. 
                axpars.negative_limit_switch_status_RBV = bool(1-axis.get_axis_parameter(axis.AP.LeftEndstop))
                axpars.positive_limit_switch_status_RBV = bool(1-axis.get_axis_parameter(axis.AP.RightEndstop))
            else:
                axpars.negative_limit_switch_status_RBV = bool(axis.get_axis_parameter(axis.AP.LeftEndstop))
                axpars.positive_limit_switch_status_RBV = bool(axis.get_axis_parameter(axis.AP.RightEndstop))

    # Add other necessary motor control functions