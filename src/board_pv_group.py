#!/usr/bin/env python3

# this started out as a straight copy of caproto's fake_motor_record and should be gradually modified to fit our needs.
# can be run from the command line using: 
# python /usr/bpauw/Code/Trinamic_TMCM6214_TMCL_IOC/src/board_pv_group.py --list-pvs -v
# later using: 
# python /usr/bpauw/Code/Trinamic_TMCM6214_TMCL_IOC --list-pvs -v 
# once I have the main code in place.

from textwrap import dedent

from caproto.server import PVGroup, SubGroup, ioc_arg_parser, pvproperty, run
from caproto.server.records import MotorFields
from .axis_parameters import AxisParameters
from .board_parameters import BoardParameters
from .__init__ import ureg

class MotorAxisPVGroup(PVGroup):
    def __init__(self, *args, axis_parameters: AxisParameters, **kwargs):
        super().__init__(*args, **kwargs)
        self.axis_parameters = axis_parameters

        # Dynamically create PVs for each attribute listed in pv_attributes
        for attr_name in axis_parameters.pv_attributes:
            attr_value = getattr(axis_parameters, attr_name)
            create_pvs_for_attribute(attr_name, attr_value, axis_parameters, self)


async def broadcast_precision_to_fields(record):
    """Update precision of all fields to that of the given record."""

    precision = record.precision
    for field, prop in record.field_inst.pvdb.items():
        if hasattr(prop, 'precision'):
            await prop.write_metadata(precision=precision)


async def motor_record(instance, async_lib, defaults=None,
                                 tick_rate_hz=10., board_parameters:BoardParameters=None, axis_index:int=0):
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
    axpar = board_parameters.axes_parameters[axis_index]

    fields: MotorFields = instance.field_inst
    have_new_position = False

    async def value_write_hook(fields, value):
        nonlocal have_new_position
        # This happens when a user puts to `motor.VAL`
        print("New position requested!", value)
        have_new_position = True

    fields.value_write_hook = value_write_hook

    await instance.write_metadata(precision=defaults['precision'])
    await broadcast_precision_to_fields(instance)

    await fields.velocity.write(defaults['velocity'])
    await fields.seconds_to_velocity.write(defaults['acceleration'])
    await fields.motor_step_size.write(defaults['resolution'])
    await fields.user_low_limit.write(defaults['user_limits'][0])
    await fields.user_high_limit.write(defaults['user_limits'][1])

    while True:
        dwell = 1. / tick_rate_hz
        target_pos = instance.value
        diff = (target_pos - fields.user_readback_value.value)
        # compute the total movement time based an velocity
        total_time = abs(diff / fields.velocity.value)
        # compute how many steps, should come up short as there will
        # be a final write of the return value outside of this call
        num_steps = int(total_time // dwell)
        if abs(diff) < 1e-9 and not have_new_position:
            if fields.stop.value != 0:
                await fields.stop.write(0)
            await async_lib.library.sleep(dwell)
            continue

        if fields.stop.value != 0:
            await fields.stop.write(0)

        await fields.done_moving_to_value.write(0)
        await fields.motor_is_moving.write(1)

        readback = fields.user_readback_value.value
        step_size = diff / num_steps if num_steps > 0 else 0.0
        resolution = max((fields.motor_step_size.value, 1e-10))

        for _ in range(num_steps):
            if fields.stop.value != 0:
                await fields.stop.write(0)
                await instance.write(readback)
                break
            if fields.stop_pause_move_go.value == 'Stop':
                await instance.write(readback)
                break

            readback += step_size
            raw_readback = readback / resolution
            await fields.user_readback_value.write(readback)
            await fields.dial_readback_value.write(readback)
            await fields.raw_readback_value.write(raw_readback)
            await async_lib.library.sleep(dwell)
        else:
            # Only executed if we didn't break
            await fields.user_readback_value.write(target_pos)

        await fields.motor_is_moving.write(0)
        await fields.done_moving_to_value.write(1)
        have_new_position = False


class TrinamicMotor(PVGroup):
    motor = pvproperty(value=0.0, name='', record='motor',
                       precision=3)

    def __init__(self, *args,
                 velocity=0.1,
                 precision=3,
                 acceleration=1.0,
                 resolution=1e-6,
                 user_limits=(0.0, 100.0),
                 tick_rate_hz=10.,
                 boardpar:BoardParameters=None,
                 axis_index:int=0,
                 **kwargs):
        super().__init__(*args, **kwargs)
        self._have_new_position = False
        self.tick_rate_hz = tick_rate_hz
        self.boardpar = boardpar
        self.axis_index = axis_index
        self.defaults = {
            'velocity': velocity,
            'precision': precision,
            'acceleration': acceleration,
            'resolution': resolution,
            'user_limits': user_limits,
        }
        

    @motor.startup
    async def motor(self, instance, async_lib):
        # Start the simulator:
        await motor_record(
            self.motor, async_lib, self.defaults,
            tick_rate_hz=self.tick_rate_hz, board_parameters=self.boardpar, axis_index=self.axis_index
        )
        


class TrinamicIOC(PVGroup):
    """
    A fake motor IOC, with 3 fake motors.

    PVs
    ---
    mtr1 (motor)
    mtr2 (motor)
    mtr3 (motor)
    """

    motor1 = SubGroup(TrinamicMotor, velocity=1., precision=3, user_limits=(0, 10), prefix='mtr1')
    motor2 = SubGroup(TrinamicMotor, velocity=2., precision=2, user_limits=(-10, 20), prefix='mtr2')
    motor3 = SubGroup(TrinamicMotor, velocity=3., precision=2, user_limits=(0, 30), prefix='mtr3')


