import asyncio
import logging
from caproto.server.records import MotorFields, pvproperty

# can't import these here, because of circular imports
# from src.motion_control import MotionControl
from . import ureg
from src.axis_parameters import AxisParameters

async def update_epics_motorfields_instance(axpar: AxisParameters, instance:pvproperty, moving_or_nonmoving:str='') -> None:
    """
    Updates the motor record fields in the EPICS IOC with the values from the AxisParameters instance.
    If moving_or_nonmoving is 'moving', the fields that are only relevant when the motor is moving are updated.
    if set to 'nonmoving', the fields that are only relevant when the motor is not moving are updated.
    if left empty, only the base fields are updated

    """
    fields: MotorFields = instance.field_inst
    await fields.engineering_units.write(format(axpar.base_realworld_unit, '~')) # this is the base unit, e.g. 'mm'
    await fields.velocity.write(axpar.velocity.to(axpar.base_realworld_unit/ureg.s).magnitude)
    await fields.seconds_to_velocity.write(axpar.acceleration_duration.to(ureg.s).magnitude)
    await fields.bl_distance.write(axpar.backlash.to(ureg.Unit(fields.engineering_units.value)).magnitude)
    # not fully implemented, just take on the values of velocity:
    await fields.bl_velocity.write(axpar.velocity.to(ureg.Unit(fields.engineering_units.value)/ureg.s).magnitude)
    await fields.bl_seconds_to_velocity.write(axpar.acceleration_duration.to(ureg.s).magnitude)
    print(f'At this point, {axpar.target_coordinate=}, {axpar.actual_coordinate_RBV=}')
    await fields.difference_dval_drbv.write(axpar.user_to_dial(axpar.target_coordinate-axpar.actual_coordinate_RBV).to(ureg.Unit(fields.engineering_units.value)).magnitude)
    await fields.difference_rval_rrbv.write(axpar.dial_to_raw(axpar.target_coordinate-axpar.actual_coordinate_RBV))
    await fields.user_low_limit.write(axpar.negative_user_limit.to(ureg.Unit(fields.engineering_units.value)).magnitude) 
    await fields.user_high_limit.write(axpar.positive_user_limit.to(ureg.Unit(fields.engineering_units.value)).magnitude)
    await fields.user_readback_value.write(axpar.actual_coordinate_RBV.to(ureg.Unit(fields.engineering_units.value)).magnitude)
    await fields.dial_readback_value.write(axpar.user_to_dial(axpar.actual_coordinate_RBV).to(ureg.Unit(fields.engineering_units.value)).magnitude)
    await fields.raw_readback_value.write(axpar.user_to_raw(axpar.actual_coordinate_RBV))
    await fields.user_offset.write(axpar.user_offset.to(ureg.Unit(fields.engineering_units.value)).magnitude)
    # inverted value to get the motor resolutiion. 
    await fields.motor_step_size.write(1./(axpar.steps_to_realworld_conversion_quantity.to(ureg.Unit('steps')/ureg.Unit(fields.engineering_units.value)).magnitude))
    await fields.base_velocity.write(0) # dummy value
    await fields.user_high_limit_switch.write(axpar.positive_limit_switch_status_RBV)
    await fields.raw_high_limit_switch.write(axpar.positive_limit_switch_status_RBV)
    await fields.user_low_limit_switch.write(axpar.negative_limit_switch_status_RBV)
    await fields.raw_low_limit_switch.write(axpar.negative_limit_switch_status_RBV)
    await fields.dial_high_limit.write(axpar.user_to_dial(axpar.positive_user_limit).to(ureg.Unit(fields.engineering_units.value)).magnitude)
    await fields.dial_low_limit.write(axpar.user_to_dial(axpar.negative_user_limit).to(ureg.Unit(fields.engineering_units.value)).magnitude)
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
    # SPMG not supposed to be set by code.. default Go. 
    # await fields.stop_pause_move_go.write('Stop')
    await fields.motor_is_moving.write(0)

async def update_epics_motorfields_instance_moving(axpar: AxisParameters, instance:pvproperty) -> None:
    """special fields in addition to update_epics_motorfields_instance to (re)set when the motor stage is moving"""
    fields: MotorFields = instance.field_inst
    await fields.raw_desired_value.write(axpar.user_to_raw(axpar.target_coordinate))
    await fields.dial_desired_value.write(axpar.user_to_dial(axpar.target_coordinate).to(ureg.Unit(fields.engineering_units.value)).magnitude)
    # await fields.stop_pause_move_go.write('Go')
    await fields.motor_is_moving.write(1)
    await fields.done_moving_to_value.write(0)
    # await fields.dial_desired_value.write(axpar.user_to_dial(axpar.target_coordinate).to(ureg.Unit(fields.engineering_units.value)).magnitude)
    
async def epics_reset_stop_flag(fields: MotorFields) -> None:
    """resets the stop flag to 0"""
    if fields.stop.value != 0:
        await fields.stop.write(0)