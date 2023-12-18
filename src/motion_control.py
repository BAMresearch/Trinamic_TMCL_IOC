from typing import Union

import numpy as np

from src.epics_utils import update_epics_motorfields_instance
from .board_control import BoardControl
from .axis_parameters import AxisParameters, quantity_converter
from . import ureg
import logging
from caproto.server.records import pvproperty
from caproto.server.records import MotorFields

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


    def find_mismatched_calibration_field(self, axpar: AxisParameters, fields: MotorFields, valuevalue:Union[None, float] = None) -> (str, Union[float, int]):
        """
        Helper method that finds out which field has been changed in EPICS, so that the calibration can update the relevant axis parameter settings.
        Returns a string that indicates which parameter changed, and a delta by how much it changed delta (new - old), in EGU as float or int. 
        We check the following fields:
        VAL, DVAL, RVAL, OFF.
        Since I don't know where to get the value for the main fields instance (VAL), that can be supplied as optional parameter. from value_write_hook
        """
        rtol = 1e-5 
        atol = 1.5

        # value
        if valuevalue is not None:
            delta = valuevalue - axpar.actual_coordinate_RBV.to(ureg.Unit(fields.engineering_units.value)).magnitude
            if not np.isclose(delta, 0, rtol=rtol):
                return "VAL", delta

        # relative val: RLV:
        delta = fields.relative_value.value
        if not np.isclose(delta, 0, rtol=rtol):
            fields.relative_value.write(0)
            return "RLV", delta

        # dval
        delta = fields.dial_desired_value.value - axpar.user_to_dial(axpar.actual_coordinate_RBV).to(ureg.Unit(fields.engineering_units.value)).magnitude
        if not np.isclose(delta, 0, rtol = rtol):
            return "DVAL", delta

        # rval
        delta = fields.raw_desired_value.value - axpar.user_to_raw(axpar.actual_coordinate_RBV).to(ureg.Unit(fields.engineering_units.value)).magnitude
        if not np.isclose(delta, 0, atol = atol):
            return "RVAL", delta

        # offset
        delta = fields.user_offset.value - axpar.user_offset.to(ureg.Unit(fields.engineering_units.value)).magnitude
        if not np.isclose(delta, 0, rtol = rtol):
            return "OFF", delta

        logging.warning("trying to find calibration field mismatch but VAL, DVAL, RVAL or OFF are not different")
        return "NotFound", 0


    async def coordinate_change_through_epics(self, axis_index_or_name: Union[int, str], EPICS_motorfields_instance:pvproperty, valuevalue:Union[float, None]=None):
        """
        This is a directing supermethod that is called when "set_use_switch" is set to "Set". 
        It can be called by board_pv_group upon value_write_hook when VAL is changed, or update_axpar_from_epics_and_take_action when either of them notice the Set flag.
        It finds out which EPICS motorfield has been changed, and calls the appropriate method to adapt the remaining motor calibration fields to the new settings.
        """
        fields: MotorFields = EPICS_motorfields_instance.fields_inst
        # check our assumptions:
        assert fields.set_use_switch.value == 'Set', logging.error('coordinate_change_through_epics called, but the EPICS motorparameter SET field is not "Set"')
        # find out what changed:
        axis_index = self._resolve_axis_index(axis_index_or_name)
        axpar = self.board_control.boardpar.axes_parameters[axis_index]

        changed_field, delta = self.find_mismatched_calibration_field(axpar, fields, valuevalue)

        # find out if the fixed offset FOFF is set to Fixed or Variable:
        logging.info(f'{fields.offset_freeze_switch.value=}')
        if fields.offset_freeze_switch.value=='Variable':
            self.coordinate_change_through_epics_set_no_foff(axis_index_or_name, EPICS_motorfields_instance, changed_field, delta)
        else:
            self.coordinate_change_through_epics_set_fixed_foff(axis_index_or_name, EPICS_motorfields_instance, changed_field, delta)
        # after we're done with these, we update the EPICS fields: 
        await update_epics_motorfields_instance(axpar, EPICS_motorfields_instance)

    def coordinate_change_through_epics_set_no_foff(self, axis_index_or_name: Union[int, str], EPICS_motorfields_instance:pvproperty, changed_field:str, delta:Union[float, int]):
        """
        When the "SET" field is 'Set' (not 'Use'), we have to update the links between the user, dial, and raw settings, as well as the high and low limits without moving the motor.
        This is used for calibration between the user and dial positions. Details in the EPICS motor record definition for the calibration-related fields. 
        Its behaviour is different depending whether "FOFF" (Fix Offset) is on or off.
        This is the method for setting when FOFF is not Fixed but Variable.
        This method is expected to be called from coordinate_change_through_epics
        changed_field must be one of: "VAL", "DVAL", "OFF" or "RVAL"
        """
        fields: MotorFields = EPICS_motorfields_instance.field_inst
        axis_index = self._resolve_axis_index(axis_index_or_name)
        axpar = self.board_control.boardpar.axes_parameters[axis_index]
        assert fields.offset_freeze_switch == 'Variable', 'FOFF switch must be "Variable" to use the coordinate_change_through_epics_set_no_foff method'
        # find out which field has changed:
        logging.info(f"Request for calibration change on {axis_index=} received. Will try changing {changed_field=} by {delta=}.")
        if changed_field == "VAL" or changed_field=="OFF" or changed_field=="RLV":
            # change offset so that the current VAL is equal to the requested VAL. 
            delta = quantity_converter(delta, ureg.Unit(fields.engineering_units.value))
            axpar.user_offset += delta
            axpar.negative_user_limit += delta
            axpar.positive_user_limit += delta
            # update the relevant fields, this updates the thing too.. 
            self.board_control.update_axis_parameters(axis_index)
            return # things might go squiffy if we now also do the below...
        elif changed_field == "DVAL": 
            # update RVAL without moving. Also change the offset so VAL stays the same. 
            delta = quantity_converter(delta, ureg.Unit(fields.engineering_units.value))
            axpar.user_offset -= delta # VAL should not change, neither the associated limits
            # send update to the board with updated hardware raw position. This can now be calculated from actual_coordinate_RBV since the offset is changed. 
            self.board_control.set_axis_single_parameter(axis_index, 'ActualPosition', axpar.user_to_raw(axpar.actual_coordinate_RBV))
            # update the relevant fields, this updates the thing too.. 
            self.board_control.update_axis_parameters(axis_index)
            return 
        elif changed_field == "RVAL":
            # update DVAL, then the offset so VAL stays the same. Pretty much the same procedure as above:
            # update RVAL without moving. Also change the offset so VAL stays the same. 
            assert isinstance(delta, int), logging.error(f'Change in calibration requested due to change in RAW, but delta provided is not int. {delta=} is of type {type(delta)=}')
            axpar.user_offset -= axpar.steps_to_real_world(delta) # VAL should not change, neither the associated limits
            # send update to the board with updated hardware raw position. This can now be calculated from actual_coordinate_RBV since the offset is changed. 
            self.board_control.set_axis_single_parameter(axis_index, 'ActualPosition', axpar.user_to_raw(axpar.actual_coordinate_RBV))
            # update the relevant fields, this updates the thing too.. 
            self.board_control.update_axis_parameters(axis_index)
            return 
        else:
            logging.warning(f'Set field changes for changes in {changed_field=} with {delta=} are not supported yet.')


    def coordinate_change_through_epics_set_fixed_foff(self, axis_index_or_name: Union[int, str], EPICS_motorfields_instance:pvproperty, changed_field:str, delta:Union[float, int]):
        """
        When the "SET" field is 'Set' (not 'Use'), we have to update the links between the user, dial, and raw settings, as well as the high and low limits without moving the motor.
        This is used for calibration between the user and dial positions. Details in the EPICS motor record definition for the calibration-related fields. 
        Its behaviour is different depending whether "FOFF" (Fix Offset) is on or off. 
        This is the method for setting when FOFF is not Variable but Fixed.
        This method is expected to be called from coordinate_change_through_epics
        changed_field must be one of: "VAL", "DVAL", "OFF" or "RVAL"
        """
        fields: MotorFields = EPICS_motorfields_instance.field_inst
        axis_index = self._resolve_axis_index(axis_index_or_name)
        axpar = self.board_control.boardpar.axes_parameters[axis_index]
        assert fields.offset_freeze_switch == 'Fixed', 'FOFF switch must be "Fixed" to use the coordinate_change_through_epics_set_fixed_foff method'
        # find out which field has changed:
        logging.info(f"Request for calibration change on {axis_index=} received. Will try changing {changed_field=} by {delta=}.")
        if changed_field == "VAL" or changed_field=='DVAL' or changed_field=="RLV":
            # change motor board value so that the current VAL is equal to the requested VAL. 
            delta = quantity_converter(delta, ureg.Unit(fields.engineering_units.value))
            self.board_control.set_axis_single_parameter(axis_index, 'ActualPosition', axpar.user_to_raw(axpar.actual_coordinate_RBV + delta))
            # and now we let nature take its course 
            self.board_control.update_axis_parameters(axis_index)
            return # things might go squiffy if we now also do the below...
        elif changed_field == "RVAL":
            # update DVAL, then the offset so VAL stays the same. Pretty much the same procedure as above:
            # update RVAL without moving. Also change the offset so VAL stays the same. 
            assert isinstance(delta, int), logging.error(f'Change in calibration requested due to change in RAW, but delta provided is not int. {delta=} is of type {type(delta)=}')
            # send update to the board with updated hardware raw position. This can now be calculated from actual_coordinate_RBV since the offset is changed. 
            self.board_control.set_axis_single_parameter(axis_index, 'ActualPosition', axpar.user_to_raw(axpar.actual_coordinate_RBV) + delta)
            # let nature take its course.
            self.board_control.update_axis_parameters(axis_index)
            return 
        # Add the changed_field OFF thingie, although with fixed offset, should anything happen really? let's not for now...
        else:
            logging.warning(f'Set field with fixed offset changes for changes in {changed_field=} with {delta=} are not supported yet.')


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
        range_realworld = axpar.raw_to_dial(range_steps)
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
        target_steps = axis_params.user_to_raw(target_coordinate)
        actual_steps = axis_params.user_to_raw(axis_params.actual_coordinate_RBV)
        logging.info(f'Are we there yet? {target_steps=}, {actual_steps=}')
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
            steps = axis_params.user_to_raw(adjusted_backlashed_target)
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