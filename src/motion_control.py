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
            # if instance is not None:
            #     await instance.fields_inst.stop.write(1)
                # await instance.fields_inst.stop_pause_move_go.write('Stop')

        if instance is not None:
            # check the EPICS values whether we should stop:
            fields = instance.field_inst
            if fields.stop.value == 1 or fields.stop_pause_move_go.value == 'Stop':
                logging.error("Motion was interrupted by an EPICS stop command.")
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

    def user_coordinate_change_by_delta(self, axis_index_or_name: Union[int, str], delta: Union[ureg.Quantity, float]) -> None:
        axis_index = self._resolve_axis_index(axis_index_or_name)
        axis_params = self.board_control.boardpar.axes_parameters[axis_index]
        delta = quantity_converter(delta)
        axis_params.user_offset += delta
        axis_params.actual_coordinate_RBV += delta
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
    
    async def home_await_and_set_limits(self, axis_index: int) -> None:
        """ kicks off the homing process, waits for it to complete, and sets the stage motion range limit to the end switch distance. """
        # indicate the stage is not homed.
        axpar = self.board_control.boardpar.axes_parameters[axis_index]
        axpar.is_homed_RBV = False
        # haven't started moving yet, so nothing is interrupted yet. 
        axpar.is_move_interrupted = False
        # home the axis
        logging.info(f"Homing axis {axis_index}...")
        self.board_control.home_axis(axis_index)
        # wait for the moves to complete
        await self.board_control.await_move_completion(axis_index)
        await self.check_for_move_interrupt(axis_index)
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
        await self.board_control.await_move_completion(axis_index)
        await self.check_for_move_interrupt(axis_index)
        if axpar.is_move_interrupted:
            # don't do anything else. 
            logging.info('Homing interrupted.')
            return

        # indicate the stage is now homed.
        logging.info(f"Axis {axis_index} homing complete.")
        axpar.is_homed_RBV = True # should be finished now. 

    def _prepare_axis_for_motion(self, axis_params: AxisParameters, reset_move_interrupt:bool = False) -> None:
        # make sure we have an updated state of the axis:
        self.board_control.update_axis_parameters(axis_params.axis_number)

        # ensure that the axis is homed before moving
        if not axis_params.is_homed_RBV:
            logging.error("Axis must be homed before moving.")

        # ensure that the axis is not moving before moving
        if axis_params.is_moving_RBV:
            logging.error("Axis must be stopped before moving.")

        if axis_params.is_move_interrupted:
            if reset_move_interrupt:
                axis_params.is_move_interrupted=False
                logging.error("Axis status cannot be in interrupted state before move, resetting flag")
            else:
                logging.error("Axis status cannot be in interrupted state before move")



    def _calculate_adjusted_target(self, axis_params: AxisParameters, target_coordinate: ureg.Quantity, absolute_or_relative: str = 'absolute') -> ureg.Quantity:
        # if not isinstance(target_coordinate, ureg.Quantity):
        #     raise TypeError("target_coordinate must be a pint.Quantity")
        
        if absolute_or_relative == 'absolute':
            adjusted_target = target_coordinate
        elif absolute_or_relative == 'relative':
            adjusted_target = axis_params.actual_coordinate_RBV + target_coordinate
        else:
            logging.error("absolute_or_relative must be 'absolute' or 'relative'.")
        return adjusted_target

    def direct_target_outside_motion_limits(self, axis_params: AxisParameters, direct_target: ureg.Quantity) -> bool:
        """
        Check if a direct (backlash-adjusted) target is outside of the motion limits.
        """
        return not (axis_params.negative_user_limit <= direct_target <= axis_params.positive_user_limit)

    async def kickoff_move_to_coordinate(self, axis_index_or_name: Union[int, str], target_coordinate: Union[ureg.Quantity, str, float, int], absolute_or_relative: str = 'absolute') -> None:
        '''
        Kick off a motion command on the motor stage. This function returns immediately, before the motion is complete. Use await_move_completion to wait for the motion to complete and handle possible movement interrupts.

        :param axis_index_or_name: Axis index or its short_id.
        :param target_coordinate: Target position as a pint.Quantity with unit, or pint-interpretable string. If no unit is supplied (as string or quantity), the base_realworld_unit of the axis is assumed (e.g., mm for linear axes, radian for rotational axes), or steps if no base_realworld_unit is set.
        :param absolute_or_relative: 'absolute' for absolute position, 'relative' for relative movement.

        '''
        axis_index = self._resolve_axis_index(axis_index_or_name)
        axis_params = self.board_control.boardpar.axes_parameters[axis_index]
        absolute_or_relative = absolute_or_relative.lower()

        # conversion of the target coordinate to a pint.Quantity
        target_coordinate = quantity_converter(target_coordinate)

        # Prepare the axis for motion, also resets interrupt here since we've not done anything yet. 
        self._prepare_axis_for_motion(axis_params, reset_move_interrupt = True)
        await self.check_for_move_interrupt(axis_index)
        # Validate target coordinate
        adjusted_target = self._calculate_adjusted_target(axis_params, target_coordinate, absolute_or_relative)

        # Apply backlash correction if needed
        backlash_needed, adjusted_backlashed_target = self.is_backlash_needed(axis_params, adjusted_target)
        # Check motion limits, taking bidirectional backlash into account
        if self.direct_target_outside_motion_limits(axis_params, adjusted_target):
            logging.error(f"Target position {adjusted_target} is outside of the axis user motion limit: {(axis_params.negative_user_limit)}, {(axis_params.positive_user_limit)} with backlash {axis_params.backlash}.")
        else:          
            print(f"backlash_needed={backlash_needed}, adjusted_backlashed_target={adjusted_backlashed_target}")
            # Perform the movement
            steps = axis_params.real_world_to_steps(adjusted_backlashed_target + axis_params.user_offset)
            self.board_control.move_axis(axis_index, steps)

    def is_backlash_needed(self, axis_params: AxisParameters, adjusted_target: ureg.Quantity) -> (bool, ureg.Quantity):
        """
        Check if backlash correction is needed for a given absolute adjusted_target position.
        Returns a tuple of (backlash_needed, adjusted_backlashed_target).
        """
        adjusted_backlashed_target = adjusted_target
        backlash_needed = axis_params.backlash_direction * (adjusted_target - axis_params.actual_coordinate_RBV).magnitude > 0
        if backlash_needed:
            adjusted_backlashed_target += axis_params.backlash * axis_params.backlash_direction
        return backlash_needed, adjusted_backlashed_target
    
    async def apply_optional_backlash_move(self, axis_index_or_name: Union[int, str], target_coordinate: Union[ureg.Quantity, str, float, int], absolute_or_relative: str = 'absolute') -> None:
        """
        Apply backlash move if needed. First part is similar to kickoff_move_to_coordinate, but we don't check the motion limits.
        """
        axis_index = self._resolve_axis_index(axis_index_or_name)
        axis_params = self.board_control.boardpar.axes_parameters[axis_index]
        absolute_or_relative = absolute_or_relative.lower()

        # conversion of the target coordinate to a pint.Quantity
        target_coordinate = quantity_converter(target_coordinate)

        # Prepare the axis for motion
        self._prepare_axis_for_motion(axis_params, reset_move_interrupt = False)
        await self.check_for_move_interrupt(axis_index)
        if axis_params.is_move_interrupted:
            # don't do anything else. 
            logging.info('Homing interrupted.')
            return
        # Validate target coordinate
        adjusted_target = self._calculate_adjusted_target(axis_params, target_coordinate, absolute_or_relative)

        backlash_needed, adjusted_target = self.is_backlash_needed(axis_params, adjusted_target)
        # of course it will think backlash is not needed... it's in the opposite direction!
        if np.isclose(abs(axis_params.actual_coordinate_RBV - adjusted_target), axis_params.backlash, rtol=0.01):
            backlash_needed = True

        print(f"backlash_needed={backlash_needed}, adjusted_backlashed_target={adjusted_target}")
        # apply backlash move if needed and possible
        if self.direct_target_outside_motion_limits(axis_params, adjusted_target):
            logging.error(f"Target position {adjusted_target} is outside of the axis user motion limit: {(axis_params.negative_user_limit)}, {(axis_params.positive_user_limit)} with backlash {axis_params.backlash}.")

        if backlash_needed:
            steps = axis_params.real_world_to_steps(adjusted_target + axis_params.user_offset)
            self.board_control.move_axis(axis_index, steps)

    async def move_to_coordinate_with_backlash(self, axis_index_or_name: Union[int, str], target_coordinate: Union[ureg.Quantity, str, float, int], absolute_or_relative: str = 'absolute', instance: Union[None, pvproperty] = None) -> None:
        """
        Move a motor axis to a specified position in real space, considering backlash.

        :param axis_index_or_name: Axis index or its short_id.
        :param target_coordinate: Target position as a pint.Quantity with unit, or pint-interpretable string. If no unit is supplied (as string or quantity), the base_realworld_unit of the axis is assumed (e.g., mm for linear axes, radian for rotational axes), or steps if no base_realworld_unit is set.
        :param absolute_or_relative: 'absolute' for absolute position, 'relative' for relative movement.
        """
        axis_index = self._resolve_axis_index(axis_index_or_name)

        # kick-off the move, plenty of checks to make sure we're interrupted if needed:
        await self.check_for_move_interrupt(axis_index, instance)
        self.kickoff_move_to_coordinate(axis_index_or_name, target_coordinate, absolute_or_relative)
        # wait for the move to complete
        await self.check_for_move_interrupt(axis_index, instance)
        await self.board_control.await_move_completion(axis_index, instance)

        await self.check_for_move_interrupt(axis_index, instance)
        await self.apply_optional_backlash_move(axis_index_or_name, target_coordinate, absolute_or_relative)
        await self.check_for_move_interrupt(axis_index, instance)

        await self.board_control.await_move_completion(axis_index, instance)
        await self.check_for_move_interrupt(axis_index, instance)

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