import asyncio
import logging
from caproto.server.records import MotorFields, pvproperty

from src.motion_control import MotionControl
from . import ureg
from src.axis_parameters import AxisParameters

async def update_axpar_from_epics_and_take_action(mc: MotionControl, axis_index:int ,  instance:pvproperty) -> None:
    """
    Updates the AxisParameters instance from the EPICS IOC and takes action if necessary. This should be done as part of a synchronization before writing back the axis parameters to epics values, but after the initial axis and epics parameters initialization.
    To be used when motor is not moving, this can be used to update either:
    1) the user offset (OFF)
    2) the dial lower limit (DLLM) and user lower limit (LLM)
    3) the dial upper limit (DHLM) and user upper limit (HLM)
    4) the velocity
    5) the acceleration duration
    6) the backlash distance
    7) the axis direction
    8) the motor step size
    9) a homing operation start (HOMR, HOMF), both will kick off the same home action
    """
    fields: MotorFields = instance.field_inst # get the motor record fields
    axpar = mc.board_control.board_params.axes_parameters[axis_index] # get the axis parameters for this axis
    bc = mc.board_control
    # 1) check if the user offset has been changed from EPICS
    if fields.user_offset.value != axpar.user_offset.to(axpar.base_realworld_unit).magnitude:
        mc.user_coordinate_changed(axis_index, (fields.user_offset.value - axpar.user_offset.to(axpar.base_realworld_unit).magnitude)*axpar.base_realworld_unit)

    # 2) check if the user lower limit has been changed from EPICS
    if fields.user_low_limit.value != axpar.negative_user_limit.to(axpar.base_realworld_unit).magnitude:
        axpar.negative_user_limit = fields.user_low_limit.value*axpar.base_realworld_unit
        # also update the dial low limit
        fields.dial_low_limit.write((axpar.negative_user_limit+axpar.user_offset).to(axpar.base_realworld_unit).magnitude)
    
    if fields.dial_low_limit.value != (axpar.negative_user_limit+axpar.user_offset).to(axpar.base_realworld_unit).magnitude:
        axpar.negative_user_limit = (fields.dial_low_limit.value*axpar.base_realworld_unit-axpar.user_offset)
        # also update the user low limit
        fields.user_low_limit.write(axpar.negative_user_limit.to(axpar.base_realworld_unit).magnitude)

    # 3) check if the user upper limit has been changed from EPICS
    if fields.user_high_limit.value != axpar.positive_user_limit.to(axpar.base_realworld_unit).magnitude:
        axpar.positive_user_limit = fields.user_high_limit.value*axpar.base_realworld_unit
        # also update the dial high limit
        fields.dial_high_limit.write((axpar.positive_user_limit+axpar.user_offset).to(axpar.base_realworld_unit).magnitude)
    
    if fields.dial_high_limit.value != (axpar.positive_user_limit+axpar.user_offset).to(axpar.base_realworld_unit).magnitude:
        axpar.positive_user_limit = (fields.dial_high_limit.value*axpar.base_realworld_unit-axpar.user_offset)
        # also update the user high limit
        fields.user_high_limit.write(axpar.positive_user_limit.to(axpar.base_realworld_unit).magnitude)
    
    # 4) check if the velocity has been changed from EPICS
    if fields.velocity.value != axpar.velocity.to(axpar.base_realworld_unit/ureg.s).magnitude:
        axpar.velocity = fields.velocity.value*axpar.base_realworld_unit/ureg.s

    # 5) check if the acceleration duration has been changed from EPICS
    if fields.seconds_to_velocity.value != axpar.acceleration_duration.to(ureg.s).magnitude:
        axpar.acceleration_duration = fields.seconds_to_velocity.value*ureg.s
    
    # 6) check if the backlash distance has been changed from EPICS
    if fields.bl_distance.value != axpar.backlash.to(axpar.base_realworld_unit).magnitude:
        axpar.backlash = fields.bl_distance.value*axpar.base_realworld_unit
    
    # 7) check if the axis direction has been changed from EPICS
    if fields.user_direction.value == 'Neg' and not axpar.invert_axis_direction:
        axpar.invert_axis_direction = True
    elif fields.user_direction.value == 'Pos' and axpar.invert_axis_direction:
        axpar.invert_axis_direction = False
    
    # 8) check if the motor step size has been changed from EPICS
    if fields.motor_step_size.value != 1./(axpar.steps_to_realworld_conversion_quantity.to(ureg.Unit('steps')/axpar.base_realworld_unit).magnitude):
        axpar.steps_to_realworld_conversion_quantity = 1./(fields.motor_step_size.value*ureg.Unit('steps')/axpar.base_realworld_unit)

    # 9) check if a homing operation has been started from EPICS
    if fields.home_forward.value == 1 or fields.home_reverse.value == 1:
        await mc.home_await_and_set_limits(axis_index)
        await fields.home_forward.write(0)
        await fields.home_reverse.write(0)
    
    # now we update the board parameters from the axis parameters
    await bc.update_board_parameters_from_axis_parameters(axis_index)

    await asyncio.sleep(0) # this is needed to allow the IOC to process the changes to the motor record fields

async def update_epics_motorfields_instance(axpar: AxisParameters, instance:pvproperty, moving_or_nonmoving:str='') -> None:
    """
    Updates the motor record fields in the EPICS IOC with the values from the AxisParameters instance.
    If moving_or_nonmoving is 'moving', the fields that are only relevant when the motor is moving are updated.
    if set to 'nonmoving', the fields that are only relevant when the motor is not moving are updated.
    if left empty, only the base fields are updated

    """
    fields: MotorFields = instance.field_inst
    await fields.velocity.write(axpar.velocity.to(axpar.base_realworld_unit/ureg.s).magnitude)
    await fields.seconds_to_velocity.write(axpar.acceleration_duration.to(ureg.s).magnitude)
    await fields.user_low_limit.write(axpar.negative_user_limit.to(axpar.base_realworld_unit).magnitude) 
    await fields.user_high_limit.write(axpar.positive_user_limit.to(axpar.base_realworld_unit).magnitude)
    await fields.user_readback_value.write(axpar.actual_coordinate_RBV.to(axpar.base_realworld_unit).magnitude)
    await fields.dial_readback_value.write((axpar.actual_coordinate_RBV+axpar.user_offset).to(axpar.base_realworld_unit).magnitude)
    await fields.raw_readback_value.write(axpar.real_world_to_steps(axpar.actual_coordinate_RBV))
    await fields.engineering_units.write(format(axpar.base_realworld_unit, '~')) # this is the base unit, e.g. 'mm'
    await fields.user_offset.write(axpar.user_offset.to(axpar.base_realworld_unit).magnitude)
    # inverted value to get the motor resolutiion. 
    await fields.motor_step_size.write(1./(axpar.steps_to_realworld_conversion_quantity.to(ureg.Unit('steps')/axpar.base_realworld_unit).magnitude))
    await fields.base_velocity.write(0) # dummy value
    await fields.user_high_limit_switch.write(axpar.positive_limit_switch_status_RBV)
    await fields.raw_high_limit_switch.write(axpar.positive_limit_switch_status_RBV)
    await fields.user_low_limit_switch.write(axpar.negative_limit_switch_status_RBV)
    await fields.raw_low_limit_switch.write(axpar.negative_limit_switch_status_RBV)
    await fields.bl_distance.write(axpar.backlash.to(axpar.base_realworld_unit).magnitude)
    await fields.bl_velocity.write(axpar.velocity.to(axpar.base_realworld_unit/ureg.s).magnitude)
    await fields.dial_high_limit.write((axpar.positive_user_limit+axpar.user_offset).to(axpar.base_realworld_unit).magnitude)
    await fields.dial_low_limit.write((axpar.negative_user_limit+axpar.user_offset).to(axpar.base_realworld_unit).magnitude)
    await fields.disable_putfield.write(0) # dummy value
    if axpar.invert_axis_direction:
        await fields.user_direction.write('Neg')
    else:
        await fields.user_direction.write('Pos')
    
    if moving_or_nonmoving == 'nonmoving':
        await update_epics_motorfields_instance_nonmoving(axpar, instance)
    elif moving_or_nonmoving == 'moving':
        await update_epics_motorfields_instance_moving(axpar, instance)

async def update_epics_motorfields_instance_nonmoving(axpar: AxisParameters, instance:pvproperty) -> None:
    """special fields in addition to update_epics_motorfields_instance to (re)set when the motor stage is not moving"""
    fields: MotorFields = instance.field_inst
    await fields.done_moving_to_value.write(1)
    await fields.stop_pause_move_go.write('Stop')
    await fields.motor_is_moving.write(0)

async def update_epics_motorfields_instance_moving(axpar: AxisParameters, instance:pvproperty) -> None:
    """special fields in addition to update_epics_motorfields_instance to (re)set when the motor stage is moving"""
    fields: MotorFields = instance.field_inst
    await fields.raw_desired_value.write(axpar.real_world_to_steps(axpar.target_coordinate+axpar.user_offset))
    await fields.dial_desired_value.write((axpar.target_coordinate+axpar.user_offset).to(axpar.base_realworld_unit).magnitude)
    await fields.stop_pause_move_go.write('Go')
    await fields.motor_is_moving.write(1)
    await fields.done_moving_to_value.write(0)
    await fields.dial_desired_value.write((axpar.target_coordinate+axpar.user_offset).to(axpar.base_realworld_unit).magnitude)
    # await fields.user_readback_value.write(axpar.actual_coordinate_RBV.to(axpar.base_realworld_unit).magnitude)
    # await fields.dial_readback_value.write((axpar.actual_coordinate_RBV+axpar.user_offset).to(axpar.base_realworld_unit).magnitude)

async def epics_reset_stop_flag(fields: MotorFields) -> None:
    """resets the stop flag to 0"""
    if fields.stop.value != 0:
        fields.stop.write(0)