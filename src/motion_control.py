from typing import Union

import numpy as np
from .board_control import BoardControl
from .axis_parameters import AxisParameters, quantity_converter
from . import ureg
import logging
from caproto.server.records import pvproperty

class MotionControl:
    """high-level interface for controlling motion of the motors"""

    def __init__(self, board_control: BoardControl) -> None:
        self.board_control = board_control

    async def check_for_move_interrupt(self, axis_index: int, instance:Union[pvproperty, None]=None) -> None:
        """ checks if the move was interrupted by a limit switch or a stop command. """
        axpar = self.board_control.boardpar.axes_parameters[axis_index]
        if axpar.is_move_interrupted:
            logging.error("Motion was interrupted by a limit switch or a stop command.")
            
        if instance is not None:
            # check the EPICS values whether we should stop:
            fields = instance.field_inst
            if fields.stop.value == 1 or fields.stop_pause_move_go.value == 'Stop':
                logging.error(f"Motion was interrupted by EPICS {fields.stop.value=} and/or {fields.stop_pause_move_go.value=}.")
                axpar.is_move_interrupted = True
            
    def user_coordinate_change(self, axis_index_or_name: Union[int, str], new_actual_coordinate: Union[ureg.Quantity, float]) -> None:
        """ 
        changes the user offset for the specified axis so that the requested value becomes the new_actual_coordinate.
        Parameters:
        axis_index_or_name: Axis index or its short_id.
        new_actual_coordinate: New actual coordinate as a pint.Quantity with unit, or pint-interpretable string. If no unit is supplied (as string or quantity), the base_realworld_unit of the axis is assumed (e.g., mm for linear axes, radian for rotational axes), or steps if no base_realworld_unit is set.
        """
        axis_index = self._resolve_axis_index(axis_index_or_name)
        axis_params = self.board_control.boardpar.axes_parameters[axis_index]
        new_actual_coordinate = quantity_converter(new_actual_coordinate, axis_params.base_realworld_unit)
        delta = new_actual_coordinate - axis_params.actual_coordinate_RBV
        self.user_coordinate_change_by_delta(axis_index, delta)

    def user_coordinate_change_by_delta(self, axis_index_or_name: Union[int, str], delta: Union[ureg.Quantity, float], adjust_user_limits:bool=True) -> None:
        """Changes the user coordinate by adjustment of the offset. For EPICS-dictated changes, adjust_user_limits should be set to False, as EPICS already updates the lower limit..."""
        axis_index = self._resolve_axis_index(axis_index_or_name)
        axis_params = self.board_control.boardpar.axes_parameters[axis_index]
        delta = quantity_converter(delta)
        axis_params.user_offset += delta
        axis_params.actual_coordinate_RBV += delta
        if adjust_user_limits: # do not do this for EPICS-directed offset changes. 
            axis_params.negative_user_limit += delta
            axis_params.positive_user_limit += delta
        self.board_control.update_axis_parameters(axis_index)
        logging.info(f"User offset for axis {axis_index} changed to {axis_params.user_offset}.")
    
    def user_coordinate_zero(self, axis_index_or_name: Union[int, str]) -> None:
        """ sets the user offset for the specified axis to the current coordinate, effectively setting the current position to zero. """
        axis_index = self._resolve_axis_index(axis_index_or_name)
        axis_params = self.board_control.boardpar.axes_parameters[axis_index]
        delta = axis_params.actual_coordinate_RBV
        self.user_coordinate_change_by_delta(axis_index, delta)
    
    async def home_await_and_set_limits(self, axis_index: int, EPICS_fields_instance:Union[pvproperty, None]=None) -> None:
        """ kicks off the homing process, waits for it to complete, and sets the stage motion range limit to the end switch distance. """
        # indicate the stage is not homed.
        axpar = self.board_control.boardpar.axes_parameters[axis_index]
        axpar.is_homed_RBV = False
        # haven't started moving yet, so nothing is interrupted yet. 
        axpar.is_move_interrupted = False
        # check if we should move
        await self.check_for_move_interrupt(axis_index, instance=EPICS_fields_instance)
        if axpar.is_move_interrupted:
            # don't do anything else. 
            logging.info('Homing interrupted.')
            return
        
        # good to go, home the axis
        logging.info(f"Homing axis {axis_index}...")
        self.board_control.home_axis(axis_index)
        # wait for the moves to complete
        await self.board_control.await_move_completion(axis_index, instance=EPICS_fields_instance)
        await self.check_for_move_interrupt(axis_index, instance=EPICS_fields_instance)
        if axpar.is_move_interrupted:
            # don't do anything else. 
            logging.info('Homing interrupted.')
            return
        logging.info(f"Axis {axis_index} homed, setting parameters.")
        # set the stage motion range limit to the end switch distance
        range_steps = self.board_control.get_end_switch_distance(axis_index)
        range_realworld = axpar.steps_to_real_world(range_steps)
        axpar.stage_motion_limit_RBV = range_realworld
        # now we re-set the user limits to re-validate that they lie within the stage motion limit
        axpar.negative_user_limit = self.board_control.boardpar.axes_parameters[axis_index].negative_user_limit
        axpar.positive_user_limit = self.board_control.boardpar.axes_parameters[axis_index].positive_user_limit
        logging.info(f"Axis {axis_index} homed, stage motion range set to {range_realworld}. Moving to center of range.")
        # move out of limit range
        self.board_control.move_axis(axis_index, int(range_steps/2))
        await self.board_control.await_move_completion(axis_index, instance=EPICS_fields_instance)
        await self.check_for_move_interrupt(axis_index, instance=EPICS_fields_instance)
        if axpar.is_move_interrupted:
            # don't do anything else. 
            logging.info('Homing interrupted.')
            return

        # indicate the stage is now homed.
        logging.info(f"Axis {axis_index} homing complete.")
        axpar.is_homed_RBV = True # should be finished now. 

    def direct_target_outside_motion_limits(self, axis_params: AxisParameters, direct_target: ureg.Quantity) -> bool:
        """
        Check if a direct (backlash-adjusted) target is outside of the motion limits.
        """
        return not (axis_params.negative_user_limit <= direct_target <= axis_params.positive_user_limit)

    def add_backlash_if_needed(self, axis_params: AxisParameters, adjusted_target: ureg.Quantity) -> ureg.Quantity:
        """
        Check if backlash correction is needed for a given absolute adjusted_target position.
        Returns a tuple of (backlash_needed, adjusted_backlashed_target).
        """
        backlash_needed = axis_params.backlash_direction * (adjusted_target - axis_params.actual_coordinate_RBV).magnitude > 0
        if backlash_needed:
            adjusted_backlashed_target = adjusted_target + axis_params.backlash * axis_params.backlash_direction
            return adjusted_backlashed_target
        else:
            return adjusted_target

    def reset_move_interrupt(self, axis_params:AxisParameters):
        if axis_params.is_move_interrupted:
            logging.info(f'is_move_interrupted is True, but reset requested. Setting to False.')
        axis_params.is_move_interrupted = False
        return

    def are_we_there_yet(self, axis_params:AxisParameters, target_coordinate:ureg.Quantity):
        """Checks whether we are within one step of the target_coordinate (user), returns True if so"""
        self.board_control.update_axis_parameters(axis_params.axis_number)
        target_steps = axis_params.real_world_to_steps(target_coordinate + axis_params.user_offset)
        actual_steps = axis_params.real_world_to_steps(axis_params.actual_coordinate_RBV + axis_params.user_offset)
        return np.isclose(target_steps, actual_steps, atol=1.5)

    async def kickoff_move_to_coordinate(self, axis_index_or_name: Union[int, str], target_coordinate: Union[ureg.Quantity, str, float, int], include_backlash_when_required:bool=True, EPICS_fields_instance:Union[pvproperty, None]=None  ) -> None:
        '''
        Kick off a motion command on the motor stage. This function returns immediately, before the motion is complete. Use await_move_completion to wait for the motion to complete and handle possible movement interrupts.

        :param axis_index_or_name: Axis index or its short_id.
        :param target_coordinate: Target position as a pint.Quantity with unit, or pint-interpretable string. If no unit is supplied (as string or quantity), the base_realworld_unit of the axis is assumed (e.g., mm for linear axes, radian for rotational axes), or steps if no base_realworld_unit is set.
        :param include_backlash_when_required: If true, a backlash distance is added when the motor is moved in the backlash direction, otherwise omitted. Set to false for the (second) backlash move itself. 
        '''
        axis_index = self._resolve_axis_index(axis_index_or_name)
        axis_params = self.board_control.boardpar.axes_parameters[axis_index]
        # get the latest hot goss off of the board. 
        self.board_control.update_axis_parameters(axis_params.axis_number)
        # conversion of the target coordinate to a pint.Quantity
        target_coordinate = quantity_converter(target_coordinate)
        # Apply backlash correction if needed
        adjusted_backlashed_target = target_coordinate
        if include_backlash_when_required:
            adjusted_backlashed_target = self.add_backlash_if_needed(axis_params, target_coordinate)
        # check if within limits, otherwise set axis_params.is_move_interrupted
        if self.direct_target_outside_motion_limits(axis_params, adjusted_backlashed_target):
            logging.error(f"Target position {target_coordinate} is outside of the axis user motion limit: {(axis_params.negative_user_limit)}, {(axis_params.positive_user_limit)} with backlash {axis_params.backlash}.")
            axis_params.is_move_interrupted = True

        # ensure that the axis is homed before moving
        if not axis_params.is_homed_RBV:
            logging.warning("Axis should ideally be homed before moving.")

        # ensure that the axis is not moving before moving
        if axis_params.is_moving_RBV:
            logging.error("Axis should ideally be stopped before moving.")

        # check if there is anything telling us not to move...        
        await self.check_for_move_interrupt(axis_index, instance=EPICS_fields_instance)
        if axis_params.is_move_interrupted:
            logging.error('Not allowed to move due to flags telling us not to.')
        else:
            steps = axis_params.real_world_to_steps(adjusted_backlashed_target + axis_params.user_offset)
            self.board_control.move_axis(axis_index, steps)

    async def move_to_coordinate_with_backlash(self, axis_index_or_name: Union[int, str], target_coordinate: Union[ureg.Quantity, str, float, int], absolute_or_relative: str = 'absolute', EPICS_fields_instance:Union[pvproperty, None]=None  ) -> None:
        """
        Move a motor axis to a specified position in real space, considering backlash.

        :param axis_index_or_name: Axis index or its short_id.
        :param target_coordinate: Target position as a pint.Quantity with unit, or pint-interpretable string. If no unit is supplied (as string or quantity), the base_realworld_unit of the axis is assumed (e.g., mm for linear axes, radian for rotational axes), or steps if no base_realworld_unit is set.
        :param absolute_or_relative: 'absolute' for absolute position, 'relative' for relative movement.
        """
        axis_index = self._resolve_axis_index(axis_index_or_name)
        axis_params = self.board_control.boardpar.axes_parameters[axis_index]
        # get the latest hot goss off of the board. 
        self.board_control.update_axis_parameters(axis_params.axis_number)
        if absolute_or_relative.lower() != 'absolute':
            abs_target_coordinate = target_coordinate + axis_params.actual_coordinate_RBV
        else: 
            abs_target_coordinate = target_coordinate
        
        # kick-off the move, plenty of checks to make sure we're interrupted if needed:
        if not axis_params.is_move_interrupted:
            self.kickoff_move_to_coordinate(axis_index_or_name, abs_target_coordinate, include_backlash_when_required=True, EPICS_fields_instance=EPICS_fields_instance)
            # wait for the move to complete
            await self.board_control.await_move_completion(axis_index, EPICS_fields_instance)
        # do the backlash move if needed 
        while not(self.are_we_there_yet(axis_params, abs_target_coordinate)) and not(axis_params.is_move_interrupted):
            self.kickoff_move_to_coordinate(axis_index_or_name, abs_target_coordinate, include_backlash_when_required=False, EPICS_fields_instance=EPICS_fields_instance)
        await self.board_control.await_move_completion(axis_index, EPICS_fields_instance)

    def _resolve_axis_index(self, axis: Union[int, str]) -> int:
        if isinstance(axis, str):
            # Resolve axis index from short_id
            for i, ap in enumerate(self.board_parameters.axes_parameters):
                if ap.short_id == axis:
                    return i
            raise ValueError(f"No axis found with short_id '{axis}'")
        elif isinstance(axis, int):
            return axis
        else:
            raise TypeError("Axis must be an int or str")

    # Additional advanced motion control methods