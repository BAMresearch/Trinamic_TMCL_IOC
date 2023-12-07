#!/usr/bin/env python3

# this started out as a straight copy of caproto's fake_motor_record and should be gradually modified to fit our needs.
# can be run from the command line using: 
# python /usr/bpauw/Code/Trinamic_TMCM6214_TMCL_IOC/src/board_pv_group.py --list-pvs -v
# later using: 
# python /usr/bpauw/Code/Trinamic_TMCM6214_TMCL_IOC --list-pvs -v 
# once I have the main code in place.

import asyncio
from pathlib import Path
from textwrap import dedent

from caproto.server import PVGroup, pvproperty
from caproto.server.records import MotorFields
from src.board_control import BoardControl
from src.motion_control import MotionControl
from src.configuration_management import ConfigurationManagement
from .board_parameters import BoardParameters
from . import ureg
import logging
from src.epics_utils import epics_reset_stop_flag, update_epics_motorfields_instance

async def broadcast_precision_to_fields(record):
    """Update precision of all fields to that of the given record."""

    precision = record.precision
    for field, prop in record.field_inst.pvdb.items():
        if hasattr(prop, 'precision'):
            await prop.write_metadata(precision=precision)

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
    10) the SPMG flag (Stop-Pause-Move-Go)
    """
    fields: MotorFields = instance.field_inst # get the motor record fields
    axpar = mc.board_control.boardpar.axes_parameters[axis_index] # get the axis parameters for this axis
    bc = mc.board_control
    change = False
    # 1) check if the user offset has been changed from EPICS
    if fields.user_offset.value != axpar.user_offset.to(axpar.base_realworld_unit).magnitude:
        mc.user_coordinate_change_by_delta(axis_index, (fields.user_offset.value - axpar.user_offset.to(axpar.base_realworld_unit).magnitude)*axpar.base_realworld_unit, adjust_user_limits=False)
        logging.info("user offset changed")
        change = True

    # 2) check if the user lower limit has been changed from EPICS
    if fields.user_low_limit.value != axpar.negative_user_limit.to(axpar.base_realworld_unit).magnitude:
        axpar.negative_user_limit = fields.user_low_limit.value*axpar.base_realworld_unit
        # also update the dial low limit
        logging.info("negative user limit changed")
        await fields.dial_low_limit.write((axpar.negative_user_limit+axpar.user_offset).to(axpar.base_realworld_unit).magnitude)
        change = True
            
    if fields.dial_low_limit.value != (axpar.negative_user_limit+axpar.user_offset).to(axpar.base_realworld_unit).magnitude:
        axpar.negative_user_limit = (fields.dial_low_limit.value*axpar.base_realworld_unit-axpar.user_offset)
        # also update the user low limit
        logging.info("dial low limit changed")
        await fields.user_low_limit.write(axpar.negative_user_limit.to(axpar.base_realworld_unit).magnitude)
        change = True

    # 3) check if the user upper limit has been changed from EPICS
    if fields.user_high_limit.value != axpar.positive_user_limit.to(axpar.base_realworld_unit).magnitude:
        axpar.positive_user_limit = fields.user_high_limit.value*axpar.base_realworld_unit
        # also update the dial high limit
        await fields.dial_high_limit.write((axpar.positive_user_limit+axpar.user_offset).to(axpar.base_realworld_unit).magnitude)
        change = True
    
    if fields.dial_high_limit.value != (axpar.positive_user_limit+axpar.user_offset).to(axpar.base_realworld_unit).magnitude:
        axpar.positive_user_limit = (fields.dial_high_limit.value*axpar.base_realworld_unit-axpar.user_offset)
        # also update the user high limit
        await fields.user_high_limit.write(axpar.positive_user_limit.to(axpar.base_realworld_unit).magnitude)
        change = True
    
    # 4) check if the velocity has been changed from EPICS
    if fields.velocity.value != axpar.velocity.to(axpar.base_realworld_unit/ureg.s).magnitude:
        axpar.velocity = fields.velocity.value*axpar.base_realworld_unit/ureg.s
        change = True

    # 5) check if the acceleration duration has been changed from EPICS
    if fields.seconds_to_velocity.value != axpar.acceleration_duration.to(ureg.s).magnitude:
        axpar.acceleration_duration = fields.seconds_to_velocity.value*ureg.s
        change = True
    
    # 6) check if the backlash distance has been changed from EPICS
    if fields.bl_distance.value != axpar.backlash.to(axpar.base_realworld_unit).magnitude:
        axpar.backlash = fields.bl_distance.value*axpar.base_realworld_unit
        change = True
    
    # 7) check if the axis direction has been changed from EPICS
    if fields.user_direction.value == 'Neg' and not axpar.invert_axis_direction:
        logging.info(f"Inverting axis direction for axis {axis_index} based on EPICS direction setting")
        axpar.invert_axis_direction = True
        change = True
    elif fields.user_direction.value == 'Pos' and axpar.invert_axis_direction:
        logging.info(f"Uninverting axis directionn (i.e. normal direction) for axis {axis_index} based on EPICS direction setting")
        axpar.invert_axis_direction = False
        change = True
    
    # 8) check if the motor step size has been changed from EPICS
    if fields.motor_step_size.value != 1./(axpar.steps_to_realworld_conversion_quantity.to(ureg.Unit('steps')/axpar.base_realworld_unit).magnitude):
        axpar.steps_to_realworld_conversion_quantity = 1./(fields.motor_step_size.value*ureg.Unit('steps')/axpar.base_realworld_unit)
        change = True

    # 9) check if a homing operation has been started from EPICS
    if fields.home_forward.value == 1 or fields.home_reverse.value == 1: # or both? but that would be weird.
        await mc.home_await_and_set_limits(axis_index)
        await fields.home_forward.write(0)
        await fields.home_reverse.write(0)
        change = True
    
    # now we update the board parameters from the axis parameters if there was a change
    if change:
        bc.update_board_parameters_from_axis_parameters(axis_index)

    await asyncio.sleep(0) # this is needed to allow this to be called as async function

    

async def motor_record(instance, async_lib, defaults=None,
                                 tick_rate_hz=10., motion_control:MotionControl=None, axis_index:int=0):
    """
    A simple motor record for use with caproto.server and the Trinamics board. 

    Parameters
    ----------
    instance : pvproperty (ChannelDouble)
        Ensure you set ``record='motor'`` in your pvproperty first.

    async_lib : AsyncLibraryLayer

    defaults : dict, optional
        Defaults for velocity, precision, acceleration, limits, and resolution.

    tick_rate_hz : float, optional
        Update rate in Hz.
    """
    if defaults is None:
        defaults = dict( # how many of these defaults do I actually need? I don't think I'm using them..
            velocity=0.1,
            precision=3,
            acceleration=1.0,
            resolution=1e-6,
            tick_rate_hz=10.,
            user_limits=(0.0, 100.0),
        )
    board_parameters=motion_control.board_control.boardpar
    axpar = board_parameters.axes_parameters[axis_index]

    fields: MotorFields = instance.field_inst
    have_new_position = False

    async def value_write_hook(instance, value):
        nonlocal have_new_position
        # This happens when a user puts to `motor.VAL`
        logging.info(f"New position {value} requested on axis {axis_index} ")
        axpar.is_move_interrupted = False # reset interrupt flag
        await motion_control.check_for_move_interrupt(axis_index, instance) # will set the is_move_interrupted flag if there's an EPICS no-go
        if axpar.is_move_interrupted:
            # don't do anything else. 
            logging.info('Cannot continue, EPICS STOP and SPMG flags do not allow it.')
            return
        have_new_position = True
        motion_control.do_backlash_move = True # make sure we check that a backlash move is done even if re-moving during a backlash
        # TODO: can we actually move directly from here? Looks like no.. maybe just the first bit tho?
        # if we are here, we are moving.
        axpar.target_coordinate=ureg.Quantity(value, axpar.base_realworld_unit) # this is the target position in real-world units
        motion_control.board_control.update_axis_parameters(axis_index)
        await update_epics_motorfields_instance(axpar, instance, 'moving')
        
        logging.info(f"Moving to {axpar.target_coordinate} on axis {axis_index} from {axpar.actual_coordinate_RBV}")
        # kickoff the move:
        await motion_control.kickoff_move_to_coordinate(axis_index, axpar.target_coordinate, absolute_or_relative='absolute')

    fields.value_write_hook = value_write_hook

    await instance.write_metadata(precision=defaults['precision'])
    await broadcast_precision_to_fields(instance)

    await fields.velocity.write(defaults['velocity']) # we don't have this parameter explicitly in the axis parameters.
    await fields.seconds_to_velocity.write(defaults['acceleration']) # we don't have this parameter explicitly in the axis parameters.
    await fields.motor_step_size.write(defaults['resolution']) # we don't have this parameter explicitly in the axis parameters.
    await update_epics_motorfields_instance(axpar, instance, '') # initial update of the EPICS fields. from this point on we can sync

    while True:
        motion_control.board_control.update_axis_parameters(axis_index)
        # check if settable values have been changed from EPICS. Takes action if needed
        await update_axpar_from_epics_and_take_action(motion_control, axis_index, instance)
        # this updates particular EPICS pvs for an accurate state of the axis. 
        await update_epics_motorfields_instance(axpar, instance)
        
        if not motion_control.board_control.check_if_moving(axis_index) and not have_new_position:
            # we are not moving
            await epics_reset_stop_flag(fields)
            await asyncio.sleep(axpar.update_interval_nonmoving)
            await motion_control.board_control.check_if_powercycle_occurred()
            # update axis state:
            motion_control.board_control.update_axis_parameters(axis_index)
            # check if settable values have been changed from EPICS. Takes action if needed. This is only done when stopped.
            if not axpar.is_moving_RBV:
                await update_axpar_from_epics_and_take_action(motion_control, axis_index, instance)
                await update_epics_motorfields_instance(axpar, instance, 'nonmoving')
            continue

        # if we are here, we should already be moving.
        # motion_control.board_control.update_axis_parameters(axis_index)
        # await update_epics_motorfields_instance(axpar, instance, 'moving')
        # now we await completion
        await motion_control.board_control.await_move_completion(axis_index, instance)

        # backlash if we must
        while motion_control.do_backlash_move: # maybe there's a cleverer move, e.g. by checking if target_coordinate and actual_coordinate_RBV match already
            motion_control.do_backlash_move = False # we're backlash moving only once unless we're starting a new move
            await update_epics_motorfields_instance(axpar, instance, 'moving')
            logging.debug(f"Backlash moving if needed from {axpar.actual_coordinate_RBV} to {axpar.target_coordinate} on axis {axis_index}")
            await motion_control.apply_optional_backlash_move(axis_index, axpar.target_coordinate, absolute_or_relative='absolute')
            # now we await completion again
            await motion_control.board_control.await_move_completion(axis_index, instance)

        # and then we are done.
        await update_epics_motorfields_instance(axpar, instance, 'nonmoving')
        # await instance.write(axpar.actual_coordinate_RBV.to(axpar.base_realworld_unit).magnitude)
        have_new_position = False # we've finished moving to a new position. 
        await epics_reset_stop_flag(fields) # reset stop if needed.



class TrinamicMotor(PVGroup):
    """
    populates a PVGroup with the PVs for a single motor axis, and initiaizes the axis.
    """
    motor = pvproperty(value=0.0, name='', record='motor',
                       precision=3)

    def __init__(self, *args,
                 motion_control:MotionControl,
                 velocity=0.1,
                 precision=3,
                 acceleration=1.0,
                 resolution=1e-6,
                 user_limits=(0.0, 100.0),
                 tick_rate_hz=10.,
                 axis_index:int=0,
                 **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._have_new_position = False
        self.tick_rate_hz = tick_rate_hz
        self.mc = motion_control
        self.bc = motion_control.board_control
        self.axis_index = axis_index
        self.defaults = {
            'velocity': velocity,
            'precision': precision,
            'acceleration': acceleration,
            'resolution': resolution,
            'user_limits': user_limits,
        }
        axpar = self.bc.boardpar.axes_parameters[self.axis_index]
        self.defaults['user_limits']=(
            axpar.negative_user_limit.to(axpar.base_realworld_unit).magnitude, 
            axpar.positive_user_limit.to(axpar.base_realworld_unit).magnitude
        )
        
    @motor.startup
    async def motor(self, instance, async_lib) -> None:
        logging.info(f'Motor instance startup: Initializing axis {self.axis_index}')
        # TODO: uncomment when we're done dev'ing
        self.bc.initialize_axis(self.axis_index)
        # home the axis - this will be improved later to only done on request. 
        # homing on request will be done by setting the home_forward or home_reverse pv to 1.
        # await self.mc.home_await_and_set_limits(self.axis_index)
        # Start the simulator:
        await motor_record(
            self.motor, async_lib, self.defaults,
            tick_rate_hz=self.tick_rate_hz, 
            motion_control=self.mc, 
            axis_index=self.axis_index
        )


class TrinamicIOC(PVGroup):
    'A main Trinamic IOC with several PVGroups created dynamically based on the configuration file.'

    def __init__(self, *args, groups:dict={}, config_file:Path=None, **kwargs) -> None:
        # this does execute...

        super().__init__(*args, **kwargs)
        self.boardpar = BoardParameters(board_configuration_file=config_file)
        ConfigurationManagement.load_configuration(config_file, self.boardpar)
        self.bc = BoardControl(self.boardpar) # low-level comm
        self.mc = MotionControl(self.bc) # high-level motions
        self.bc.initialize_board() # set up comms with the board. 
        self.groups = groups
        for ax_id, axpar in enumerate(self.bc.boardpar.axes_parameters):
            axpar = self.bc.boardpar.axes_parameters[ax_id]
            setattr(self, f'motor{ax_id}', TrinamicMotor( # hopefully this creates a subgroup
                    motion_control= self.mc, 
                    axis_index=axpar.axis_number, 
                    prefix=f'{self.prefix}{axpar.short_id}',
                    )
            )
            self.pvdb.update(**getattr(self, f'motor{ax_id}').pvdb)

    # motor0 = SubGroup(TrinamicMotor, board_control = None, axis_index=0, velocity=1., precision=3, user_limits=(0, 10), prefix='mtr1')
