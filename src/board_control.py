import asyncio
import logging
from typing import Union
from src.axis_parameters import AxisParameters
from src.board_parameters import BoardParameters
import pytrinamic
from pytrinamic.connections import ConnectionManager
from pytrinamic.modules import TMCM6214
import time
from . import pint
from . import ureg
from caproto.server.records import MotorFields, pvproperty

class BoardControl:
    """low-level commands to communicate with the board, addressing basic board funccionalities"""
    def __init__(self, boardpar:BoardParameters): # , connection_string:str = "--interface socket_serial_tmcl --port 192.168.0.253:4016 --host-id 3 --module-id 0"):
        connection_string = f"--interface socket_serial_tmcl --port {boardpar.ip_address}:{boardpar.port_number} --host-id 3 --module-id {boardpar.board_module_id}"
        self.connection_manager = ConnectionManager(connection_string)
        self.boardpar = boardpar
        self.module = boardpar.board_module_id

    def initialize_board(self):
        with self.connection_manager.connect() as myInterface:
            self.module = TMCM6214(myInterface, module_id=self.boardpar.board_module_id)
            for key, value in self.boardpar.board_configurable_parameters.items():
                print(f"setting board {key=} {value=}")
                self.module.set_global_parameter(key, 0, value) # these are automatically stored
    
    def initialize_axis(self, axis_index:int):
        with self.connection_manager.connect() as myInterface:
            self.module = TMCM6214(myInterface)
            axpar=self.boardpar.axes_parameters[axis_index]
            for key, value in axpar.configurable_parameters.items():
                print(f"setting {axis_index=} {key=} {value=}")
                self.module.set_axis_parameter(key, axis_index, value)

    def initialize_axes(self):
        for axpar in self.boardpar.axes_parameters:
            self.initialize_axis(axpar.axis_number)

    def get_end_switch_distance(self, axis_index:int):
        with self.connection_manager.connect() as myInterface:
            self.module = TMCM6214(myInterface)
            return self.module.get_axis_parameter(self.module.motors[axis_index].AP.RightLimitSwitchPosition, axis_index) # limit switch distance in steps. 

    def home_axis(self, axis_index:int):     
        with self.connection_manager.connect() as myInterface:
            self.module = TMCM6214(myInterface)
            # self.module.reference_search(0, axis_index, self.boardpar.board_module_id)
            myInterface.reference_search(0, axis_index, self.boardpar.board_module_id)
    
    def check_if_moving(self, axis_index:int):
        self.update_axis_parameters(axis_index)
        return bool((not self.boardpar.axes_parameters[axis_index].is_position_reached_RBV) and (self.boardpar.axes_parameters[axis_index].is_moving_RBV))

    async def await_move_completion(self, axis_index:int, EPICS_fields:Union[MotorFields, None]=None, instance:Union[pvproperty, None]=None):
        self.update_axis_parameters(axis_index)
        axpar = self.boardpar.axes_parameters[axis_index]
        doublecheck = 0 # doublecheck that the motor is not moving anymore.
        while doublecheck < 2:
            doublecheck += int(not(self.check_if_moving(axis_index)))
            await asyncio.sleep(self.boardpar.axes_parameters[axis_index].update_interval_moving)
            self.update_axis_parameters(axis_index)
            # weirdness 6; not sure I should be continually writing this
            # if instance is not None: 
            #     await instance.write(axpar.actual_coordinate_RBV.to(axpar.base_realworld_unit).magnitude)
            if EPICS_fields is not None:
                await EPICS_fields.user_readback_value.write(axpar.actual_coordinate_RBV.to(axpar.base_realworld_unit).magnitude)
                await EPICS_fields.dial_readback_value.write((axpar.actual_coordinate_RBV+axpar.user_offset).to(axpar.base_realworld_unit).magnitude)
                await EPICS_fields.raw_readback_value.write(axpar.real_world_to_steps(axpar.actual_coordinate_RBV))
                if EPICS_fields.stop.value == 1 or EPICS_fields.stop_pause_move_go.value == 'Stop':
                    self.stop_axis(axis_index)
                    await EPICS_fields.stop.write(0)
                    axpar.is_move_interrupted = True
                    logging.warning(f"Motion interrupted by {EPICS_fields.stop.value=} and/or {EPICS_fields.stop_pause_move_go.value=}.")
                    break
            if axpar.is_move_interrupted:
                self.stop_axis(axis_index) # stop the motor motion immediately
                logging.warning("Motion interrupted by limit switch or stop command.")
                break
        # if we didn't break out of the loop, the motion is complete. in case of imperfect movement, update target position to actual. 
        if EPICS_fields is not None:
            await EPICS_fields.user_readback_value.write(axpar.actual_coordinate_RBV.to(axpar.base_realworld_unit).magnitude)
            
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
            module = TMCM6214(myInterface, module_id=self.boardpar.board_module_id)
            axis = module.motors[axis_index]

            axpars.set_actual_coordinate_RBV_by_steps(int(axis.get_axis_parameter(axis.AP.ActualPosition)))
            axpars.immediate_target_coordinate_RBV = axpars.steps_to_real_world(int(axis.get_axis_parameter(axis.AP.TargetPosition)))

            axpars.is_moving_RBV = bool(axis.get_axis_parameter(axis.AP.ActualVelocity)!=0)
            axpars.is_position_reached_RBV = bool(axis.get_axis_parameter(axis.AP.PositionReachedFlag))
            if axpars.invert_limit_values:
                axpars.negative_limit_switch_status_RBV = ~bool(axis.get_axis_parameter(axis.AP.LeftLimitSwitch))
                axpars.positive_limit_switch_status_RBV = ~bool(axis.get_axis_parameter(axis.AP.RightLimitSwitch))
            else:
                axpars.negative_limit_switch_status_RBV = bool(axis.get_axis_parameter(axis.AP.LeftLimitSwitch))
                axpars.positive_limit_switch_status_RBV = bool(axis.get_axis_parameter(axis.AP.RightLimitSwitch))

    # Add other necessary motor control functions