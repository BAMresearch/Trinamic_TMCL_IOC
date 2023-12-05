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

    async def value_write_hook(fields, value):
        nonlocal have_new_position
        # This happens when a user puts to `motor.VAL`
        print(f"New position {value} requested on axis {axis_index} ")
        have_new_position = True
        # TODO: can we actually move directly from here? Looks like no.. maybe just the first bit tho?
        # if we are here, we are moving.
        motion_control.board_control.update_axis_parameters(axis_index)
        axpar.target_coordinate=ureg.Quantity(instance.value, axpar.base_realworld_unit) # this is the target position in real-world units
        await update_epics_motorfields_instance(axpar, instance, 'moving')
        
        print(f"Moving to {axpar.target_coordinate} on axis {axis_index} from {axpar.actual_coordinate_RBV}")
        # kickoff the move:
        await motion_control.kickoff_move_to_coordinate(axis_index, axpar.target_coordinate, absolute_or_relative='absolute')

    fields.value_write_hook = value_write_hook

    await instance.write_metadata(precision=defaults['precision'])
    await broadcast_precision_to_fields(instance)

    await fields.velocity.write(defaults['velocity']) # we don't have this parameter explicitly in the axis parameters.
    await fields.seconds_to_velocity.write(defaults['acceleration']) # we don't have this parameter explicitly in the axis parameters.
    await fields.motor_step_size.write(defaults['resolution']) # we don't have this parameter explicitly in the axis parameters.
    await update_epics_motorfields_instance(axpar, instance, '') 

    while True:
        motion_control.board_control.update_axis_parameters(axis_index)
        await update_epics_motorfields_instance(axpar, instance)
        if not motion_control.board_control.check_if_moving(axis_index) and not have_new_position:
            # we are not moving
            await update_epics_motorfields_instance(axpar, instance, 'nonmoving')
            await epics_reset_stop_flag(fields)
            await asyncio.sleep(axpar.update_interval_nonmoving)
            motion_control.board_control.update_axis_parameters(axis_index)
            continue

        # if we are here, we are moving.
        motion_control.board_control.update_axis_parameters(axis_index)
        # axpar.target_coordinate=ureg.Quantity(instance.value, axpar.base_realworld_unit) # this is the target position in real-world units
        await update_epics_motorfields_instance(axpar, instance, 'moving')
        
        # print(f"Moving to {axpar.target_coordinate} on axis {axis_index} from {axpar.actual_coordinate_RBV}")
        # # kickoff the move:
        # await motion_control.kickoff_move_to_coordinate(axis_index, axpar.target_coordinate, absolute_or_relative='absolute')

        # we should already be moving...
        # now we await completion
        await motion_control.board_control.await_move_completion(axis_index, instance)

        # backlash if we must
        await update_epics_motorfields_instance(axpar, instance, 'moving')
        print(f"Maybe backlash moving to {axpar.target_coordinate} on axis {axis_index} from {axpar.actual_coordinate_RBV}")
        await motion_control.apply_optional_backlash_move(axis_index, axpar.target_coordinate, absolute_or_relative='absolute')
        # now we await completion again
        await motion_control.board_control.await_move_completion(axis_index, instance)

        await update_epics_motorfields_instance(axpar, instance, 'nonmoving')
        # await instance.write(axpar.actual_coordinate_RBV.to(axpar.base_realworld_unit).magnitude)
        have_new_position = False
        await epics_reset_stop_flag(fields)



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
        print(f'Initializing axis {self.axis_index}')
        # TODO: uncomment when we're done dev'ing
        self.bc.initialize_axis(self.axis_index)
        # home the axis - this will be improved later to only done on request. 
        await self.mc.home_await_and_set_limits(self.axis_index)
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
        self.boardpar = BoardParameters()
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
