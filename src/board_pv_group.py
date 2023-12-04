#!/usr/bin/env python3

# this started out as a straight copy of caproto's fake_motor_record and should be gradually modified to fit our needs.
# can be run from the command line using: 
# python /usr/bpauw/Code/Trinamic_TMCM6214_TMCL_IOC/src/board_pv_group.py --list-pvs -v
# later using: 
# python /usr/bpauw/Code/Trinamic_TMCM6214_TMCL_IOC --list-pvs -v 
# once I have the main code in place.

from pathlib import Path
from textwrap import dedent

from caproto.server import PVGroup, SubGroup, ioc_arg_parser, pvproperty, run
from caproto.server.records import MotorFields
from src.board_control import BoardControl
from src.motion_control import MotionControl
from src.configuration_management import ConfigurationManagement
from .axis_parameters import AxisParameters
from .board_parameters import BoardParameters
from .__init__ import ureg

# class MotorAxisPVGroup(PVGroup):
#     def __init__(self, *args, axis_parameters: AxisParameters, **kwargs):
#         super().__init__(*args, **kwargs)
#         self.axis_parameters = axis_parameters

#         # Dynamically create PVs for each attribute listed in pv_attributes
#         for attr_name in axis_parameters.pv_attributes:
#             attr_value = getattr(axis_parameters, attr_name)
#             create_pvs_for_attribute(attr_name, attr_value, axis_parameters, self)


async def broadcast_precision_to_fields(record):
    """Update precision of all fields to that of the given record."""

    precision = record.precision
    for field, prop in record.field_inst.pvdb.items():
        if hasattr(prop, 'precision'):
            await prop.write_metadata(precision=precision)


async def motor_record(instance, async_lib, defaults=None,
                                 tick_rate_hz=10., motion_control:MotionControl=None, axis_index:int=0):
    """
    A simple motor record.

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
        defaults = dict(
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
        print(f"New position {value} requested on axis {axis_index} ", value)
        have_new_position = True
        # we've a method for this... float or int values are also automatically converted. 
        # but maybe this should be done in the while loop below. 

    fields.value_write_hook = value_write_hook

    await instance.write_metadata(precision=defaults['precision'])
    await broadcast_precision_to_fields(instance)

    await fields.velocity.write(defaults['velocity']) # we don't have this parameter explicitly in the axis parameters.
    await fields.seconds_to_velocity.write(defaults['acceleration']) # we don't have this parameter explicitly in the axis parameters.
    await fields.motor_step_size.write(defaults['resolution']) # we don't have this parameter explicitly in the axis parameters.
    await fields.user_low_limit.write(axpar.negative_user_limit.to(axpar.base_realworld_unit).magnitude) 
    await fields.user_high_limit.write(axpar.positive_user_limit.to(axpar.base_realworld_unit).magnitude)

    while True:
        dwell = axpar.update_interval_nonmoving
        target_pos = instance.value
        diff = (target_pos - axpar.actual_coordinate_RBV.to(axpar.base_realworld_unit).magnitude)
        motion_control.board_control.update_axis_parameters(axis_index)
        if not motion_control.board_control.check_if_moving(axis_index) and not have_new_position:
            fields.stop_pause_move_go.write('Stop')
            if fields.stop.value != 0:
                fields.stop.write(0)
            fields.motor_is_moving.write(0)
            await async_lib.library.sleep(axpar.update_interval_nonmoving)
            motion_control.board_control.update_axis_parameters(axis_index)
            continue

        # if we are here, we are moving.
        if fields.stop.value != 0:
            await fields.stop.write(0) # reset the stop flag

        motion_control.board_control.update_axis_parameters(axis_index)
        axpar.target_coordinate=ureg.Quantity(instance.value, axpar.base_realworld_unit) # this is the target position in real-world units
        await fields.dial_desired_value.write((axpar.target_coordinate+axpar.user_offset).to(axpar.base_realworld_unit).magnitude)
        await fields.done_moving_to_value.write(0)
        await fields.motor_is_moving.write(1)
        readback = axpar.actual_coordinate_RBV.to(axpar.base_realworld_unit).magnitude
        fields.user_readback_value.value = readback
        # kickoff the move:
        await motion_control.kickoff_move_to_coordinate(axis_index, target_pos, absolute_or_relative='absolute')
        # now we await completion
        await motion_control.board_control.await_move_completion(axis_index, fields)

        # backlash if we must
        await motion_control.apply_optional_backlash_move(axis_index, target_pos, absolute_or_relative='absolute')
        # now we await completion again
        await motion_control.board_control.await_move_completion(axis_index, fields)


        # for _ in range(num_steps):
        #     if fields.stop.value != 0:
        #         await fields.stop.write(0)
        #         await instance.write(readback)
        #         break
        #     if fields.stop_pause_move_go.value == 'Stop':
        #         await instance.write(readback)
        #         break

        #     readback = axpar.actual_coordinate_RBV.to(axpar.base_realworld_unit).magnitude
        #     raw_readback = axpar.real_world_to_steps(axpar.actual_coordinate_RBV) # this is the readback in steps
        #     await fields.user_readback_value.write(readback)
        #     await fields.dial_readback_value.write(readback)
        #     await fields.raw_readback_value.write(raw_readback)
        #     await async_lib.library.sleep(dwell)
        # else:
        #     # Only executed if we didn't break
        #     await fields.user_readback_value.write(target_pos)

        await fields.motor_is_moving.write(0)
        await fields.done_moving_to_value.write(1)
        have_new_position = False

class TrinamicMotor(PVGroup):
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
                 **kwargs):
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
    async def motor(self, instance, async_lib):
        print(f'Initializing axis {self.axis_index}')
        # TODO: uncomment when we're done dev'ing
        self.bc.initialize_axis(self.axis_index)
        # Start the simulator:
        await motor_record(
            self.motor, async_lib, self.defaults,
            tick_rate_hz=self.tick_rate_hz, 
            motion_control=self.mc, 
            axis_index=self.axis_index
        )
        
    

# async def motor_record_simulator(instance, async_lib, defaults=None,
#                                  tick_rate_hz=10.):
#     """
#     A simple motor record simulator.

#     Parameters
#     ----------
#     instance : pvproperty (ChannelDouble)
#         Ensure you set ``record='motor'`` in your pvproperty first.

#     async_lib : AsyncLibraryLayer

#     defaults : dict, optional
#         Defaults for velocity, precision, acceleration, limits, and resolution.

#     tick_rate_hz : float, optional
#         Update rate in Hz.
#     """
#     if defaults is None:
#         defaults = dict(
#             velocity=0.1,
#             precision=3,
#             acceleration=1.0,
#             resolution=1e-6,
#             tick_rate_hz=10.,
#             user_limits=(0.0, 100.0),
#         )

#     fields: MotorFields = instance.field_inst
#     have_new_position = False

#     async def value_write_hook(fields, value):
#         nonlocal have_new_position
#         # This happens when a user puts to `motor.VAL`
#         have_new_position = True


#     fields.value_write_hook = value_write_hook

#     await instance.write_metadata(precision=defaults['precision'])
#     await broadcast_precision_to_fields(instance)

#     await fields.velocity.write(defaults['velocity'])
#     await fields.seconds_to_velocity.write(defaults['acceleration'])
#     await fields.motor_step_size.write(defaults['resolution'])
#     await fields.user_low_limit.write(defaults['user_limits'][0])
#     await fields.user_high_limit.write(defaults['user_limits'][1])

#     while True:
#         dwell = 1. / tick_rate_hz
#         target_pos = instance.value
#         diff = (target_pos - fields.user_readback_value.value)
#         # compute the total movement time based an velocity
#         total_time = abs(diff / fields.velocity.value)
#         # compute how many steps, should come up short as there will
#         # be a final write of the return value outside of this call
#         num_steps = int(total_time // dwell)
#         if abs(diff) < 1e-9 and not have_new_position:
#             if fields.stop.value != 0:
#                 await fields.stop.write(0)
#             await async_lib.library.sleep(dwell)
#             continue

#         if fields.stop.value != 0:
#             await fields.stop.write(0)

#         await fields.done_moving_to_value.write(0)
#         await fields.motor_is_moving.write(1)

#         readback = fields.user_readback_value.value
#         step_size = diff / num_steps if num_steps > 0 else 0.0
#         resolution = max((fields.motor_step_size.value, 1e-10))

#         for _ in range(num_steps):
#             if fields.stop.value != 0:
#                 await fields.stop.write(0)
#                 await instance.write(readback)
#                 break
#             if fields.stop_pause_move_go.value == 'Stop':
#                 await instance.write(readback)
#                 break

#             readback += step_size
#             raw_readback = readback / resolution
#             await fields.user_readback_value.write(readback)
#             await fields.dial_readback_value.write(readback)
#             await fields.raw_readback_value.write(raw_readback)
#             await async_lib.library.sleep(dwell)
#         else:
#             # Only executed if we didn't break
#             await fields.user_readback_value.write(target_pos)

#         await fields.motor_is_moving.write(0)
#         await fields.done_moving_to_value.write(1)
#         have_new_position = False



class TrinamicIOC(PVGroup):
    'A main Trinamic IOC with several PVGroups created dynamically based on the configuration file.'

    # motor1 = None
    # motor1 = SubGroup(FakeMotor, velocity=1., precision=3, user_limits=(0, 10), prefix='mtr1')

    def __init__(self, *args, groups:dict={}, config_file:Path=None, **kwargs):
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
