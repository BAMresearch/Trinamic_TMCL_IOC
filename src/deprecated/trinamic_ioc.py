#!/usr/bin/env python3
from textwrap import dedent

from caproto.server import (
    PVGroup,
    SubGroup,
    ioc_arg_parser,
    pvproperty,
    run,
    get_pv_pair_wrapper,
    template_arg_parser,
)
from caproto.server.records import MotorFields
import numpy as np
from comms import sendReceiveBufferSocket
from pytrinamic.tmcl import TMCLRequest, TMCLReply, TMCLCommand
from typing import List
import logging
import asyncio

# get logger instance for message passing
logger = logging.getLogger("caproto")

RBV_SUFFIX = "_RBV"

# Create _two_ PVs with a single pvproperty_with_rbv:
pvproperty_with_rbv = get_pv_pair_wrapper(
    setpoint_suffix="", readback_suffix=RBV_SUFFIX
)


def clamp(variable, minimum, maximum):
    return max(min(variable, maximum), minimum)


class RecordSpecificFields:
    RMP = 1
    VELO = 3
    VMAX = 4
    ACCL = 5


class FieldNames:
    VELO = "velocity"
    VMAX = "max_velocity"
    ACCL = "seconds_to_velocity"


async def broadcast_precision_to_fields(record):
    """Update precision of all fields to that of the given record."""

    precision = record.precision
    for field, prop in record.field_inst.pvdb.items():
        if hasattr(prop, "precision"):
            await prop.write_metadata(precision=precision)


class Motor(PVGroup):
    async def set_motor_params(self, instance, value):
        print(f"Setter called with: {instance = }, {value = }")
        print(vars(instance.field_inst))

    motor = pvproperty(
        value=0.0, put=set_motor_params, name="", record="motor", precision=3
    )

    def __init__(
        self,
        *args,
        motor_bank,
        # tick_rate_hz=10.0,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self._have_new_position = False
        # self.tick_rate_hz = tick_rate_hz
        # self.defaults = {
        #     "velocity": velocity,
        #     "precision": precision,
        #     "acceleration": acceleration,
        #     "resolution": resolution,
        #     "user_limits": user_limits,
        # }

        self._motor_bank = motor_bank

    @motor.startup
    async def motor(self, instance, async_lib):
        pass

    async def update_motor_record(self, motor_status):

        # for k, v in vars(FieldNames).items():
        #     if not k.startswith("_"):
        #         exec(
        #             f"await self.motor.field_inst.{v}.write(value=motor_status[f'{str(k)}'])"
        #         )
        await asyncio.gather(
            *[
                self.motor.field_inst.velocity.write(value=motor_status["VELO"]),
                self.motor.field_inst.seconds_to_velocity.write(
                    value=motor_status["ACCL"]
                ),
                self.motor.field_inst.max_velocity.write(value=motor_status["VMAX"]),
            ]
        )


class TrinamicBoardIOC(PVGroup):
    """
    A fake motor IOC, with 3 fake motors.

    PVs
    ---
    mtr1 (motor)
    mtr2 (motor)
    mtr3 (motor)
    """

    motor_banks: List[int]
    motor1 = 0

    def __init__(
        self, *, address: int, host: str, port: int, motor_banks: List[int], **kwargs
    ):
        super().__init__(**kwargs)
        self._address = address
        self._host = host
        self._port = port
        self._motor_banks = motor_banks.split(",")
        motor_banks = [int(motor_bank) for motor_bank in self._motor_banks]

    for i in range(6):
        exec(f"motor{i} = SubGroup(Motor, motor_bank={i}, prefix=f'mtr{i}',)")

    update_hook = pvproperty(name="update", value=False, record="bi")

    async def query_motor(self, motor_bank: int):
        motor_info_dict = {}
        query_cmds = {}
        for k, v in vars(RecordSpecificFields).items():
            if not k.startswith("_"):
                query_cmds[k] = v

        motor_info = await asyncio.gather(
            *(self.get_axis_parameter(motor_bank, cmd) for cmd in query_cmds.values())
        )

        for k, value in zip(query_cmds.keys(), motor_info):
            motor_info_dict[k] = value

        return motor_info_dict

    @update_hook.scan(period=0.1, use_scan_field=True)
    async def update_hook(self, instance, async_lib):

        # Reach out to the device asynchronously and get back information:
        motor_status = await asyncio.gather(*[self.query_motor(i) for i in range(6)])
        await asyncio.gather(
            *(
                self.motor0.update_motor_record(motor_status[0]),
                self.motor1.update_motor_record(motor_status[1]),
                self.motor2.update_motor_record(motor_status[2]),
                self.motor3.update_motor_record(motor_status[3]),
                self.motor4.update_motor_record(motor_status[4]),
                self.motor5.update_motor_record(motor_status[5]),
            )
        )

    async def get_axis_parameter(self, motor_bank: int, cmd_type: int):
        cmd = TMCLRequest(
            self._address, TMCLCommand.GAP, cmd_type, motor_bank, 0
        ).to_buffer()
        response = await sendReceiveBufferSocket(self._host, self._port, cmd)
        value = await processResponse(self, motor_bank, response, cmd_type)
        return value

    async def set_axis_parameter(
        self, motor_bank: int, cmd_type: int, value: float
    ) -> None:
        cmd = TMCLRequest(
            self._address, TMCLCommand.SAP, cmd_type, motor_bank, value,
        ).to_buffer()
        response = await sendReceiveBufferSocket(self._host, self._port, cmd)
        value = await self.processResponse(response, cmd_type)
        return value

    @SubGroup
    class limit_switches(PVGroup):
        pass
        # right_limit_switch_polarity = pvproperty_with_rbv(
        #     value=0,
        #     record="ao",
        #     doc="Setting this parameter to 1 inverts the logic state of the right limit switch input.",
        # )

        # @right_limit_switch_polarity.setpoint.putter
        # async def right_limit_switch_polarity(self, instance, value):
        #     value = int(clamp(value, 0, 1))
        #     logger.debug(f"Set right limit switch polarity to {value}.")
        #     cmd_type = 24
        #     await self.parent.parent._set_axis_parameter(cmd_type, value)

        # @right_limit_switch_polarity.readback.scan(period=5.0)
        # async def right_limit_switch_polarity(self, instance, async_lib):
        #     await self.parent.parent._scan_axis_parameter(24)

        # left_limit_switch_polarity = pvproperty_with_rbv(
        #     value=0,
        #     record="ao",
        #     doc="Setting this parameter to 1 inverts the logic state of the left limit switch input.",
        # )

        # @left_limit_switch_polarity.setpoint.putter
        # async def left_limit_switch_polarity(self, instance, value):
        #     value = int(clamp(value, 0, 1))
        #     logger.debug(f"Set left limit switch polarity to {value}.")
        #     cmd_type = 25
        #     await self.parent.parent._set_axis_parameter(cmd_type, value)

        # @left_limit_switch_polarity.readback.scan(period=5.0)
        # async def left_limit_switch_polarity(self, instance, async_lib):
        #     await self.parent.parent._scan_axis_parameter(25)

        # limit_switch_swap = pvproperty_with_rbv(
        #     value=0,
        #     record="ao",
        #     doc="Swap the left and right limit switches when set to 1.",
        # )

        # @limit_switch_swap.setpoint.putter
        # async def limit_switch_swap(self, instance, value):
        #     value = int(clamp(value, 0, 1))
        #     logger.debug(f"Set limit switch swap value to {value}.")
        #     cmd_type = 14
        #     await self.parent.parent._set_axis_parameter(cmd_type, value)

        # @limit_switch_swap.readback.scan(period=5.0)
        # async def limit_switch_swap(self, instance, async_lib):
        #     await self.parent.parent._scan_axis_parameter(14)

    @SubGroup
    class motor_params(PVGroup):
        pass
        # max_current = pvproperty_with_rbv(
        #     value=0,
        #     record="ao",
        #     name="max_current",
        #     doc="Motor current used when motor is running. The maximum value is 255 which means 100% of the maximum current of the module. Refer to manual for more details.",
        # )

        # @max_current.setpoint.putter
        # async def max_current(self, instance, value):
        #     value = int(clamp(value, 0, 255))
        #     logger.debug(f"Setting the motor's max current to {value/255.}%")
        #     cmd_type = 6
        #     await self.parent.parent._set_axis_parameter(cmd_type, value)

        # @max_current.readback.scan(period=5.0)
        # async def max_current(self, instance, async_lib):
        #     await self.parent.parent._scan_axis_parameter(6)

        # standby_current = pvproperty_with_rbv(
        #     value=0,
        #     record="ao",
        #     name="standby_current",
        #     doc="The current used when the motor is not running. The maximum value is 255 which means 100% of the maximum current of the module. This value should be as low as possible so that the motor can cool down when it is not moving. Please see also parameter 214.",
        # )

        # @standby_current.setpoint.putter
        # async def standby_current(self, instance, value):
        #     value = int(clamp(value, 0, 255))
        #     logger.debug(f"Setting the motor's standby current to {value/255.}%")
        #     cmd_type = 7
        #     await self.parent.parent._set_axis_parameter(cmd_type, value)

        # @standby_current.readback.scan(period=5.0)
        # async def standby_current(self, instance, async_lib):
        #     await self.parent.parent._scan_axis_parameter(7)

        # microstep_resolution = pvproperty_with_rbv(
        #     value=0,
        #     record="ao",
        #     doc="Microstep resolutions per full step: 0=fullstep; 1=halfstep; 2=4 microsteps; 3=8 microsteps; 4=16 microsteps; 5=32 microsteps; 6=64 microsteps; 7=128 microsteps; 8=256 microsteps",
        # )

        # @microstep_resolution.setpoint.putter
        # async def microstep_resolution(self, instance, value):
        #     value = int(clamp(value, 0, 8))
        #     logger.debug(
        #         f"Setting the microstep resolution to {2**value} microstep resolutions per full step."
        #     )
        #     cmd_type = 140
        #     await self.parent.parent._set_axis_parameter(cmd_type, value)

        # @microstep_resolution.readback.scan(period=5.0)
        # async def microstep_resolution(self, instance, async_lib):
        #     await self.parent.parent._scan_axis_parameter(140)

        # dc_step_time = pvproperty_with_rbv(
        #     value=0,
        #     record="ao",
        #     name="dc_step_time",
        #     doc="This setting controls the reference pulse width for dcStep load measurement. It must be optimized for robust operation with maximum motor torque. A higher value allows higher torque and higher velocity, a lower value allows operation down to a lower velocity as set by VDCMIN.",
        # )

        # @dc_step_time.setpoint.putter
        # async def dc_step_time(self, instance, value):
        #     value = int(clamp(value, 0, 255))
        #     logger.debug(f"Setting the motor's dc step time to {value/255.}%")
        #     cmd_type = 32
        #     await self.parent.parent._set_axis_parameter(cmd_type, value)

        # @dc_step_time.readback.scan(period=5.0)
        # async def dc_step_time(self, instance, async_lib):
        #     await self.parent.parent._scan_axis_parameter(32)


async def processResponse(
    board: TrinamicBoardIOC, motor_bank: int, response: TMCLReply, command_type: int
):
    """Process the response from the commands to catch errors, returns extracted values"""

    module_address = response.module_address
    status = response.status
    command = response.command
    value = response.value

    if not (module_address == board._address):
        return  # Apparently the message was not directed at us

    if not (status == 100):
        error = f"An error status was sent by motor {motor_bank} of address {board._address}."
        logger.error(error)
        return

    return value


def create_parser():
    parser, split_args = template_arg_parser(
        default_prefix="Trinamic:",
        desc="Trinamic motor control board",
        supported_async_libs=("asyncio",),
    )

    parser.add_argument(
        "--host", help="Hostname or IP of the board", required=True, type=str,
    )

    parser.add_argument(
        "--port", help="Network port of the board (i.e. 4001)", default=4001, type=int,
    )

    parser.add_argument(
        "--address", help="Address of the board", required=True, type=int,
    )

    parser.add_argument(
        "--motor_banks", help="Motor bank numbers", required=True, type=str,
    )

    parser.add_argument(
        "--autosave",
        help="Path to the autosave file",
        default="autosave.json",
        type=str,
    )
    return parser, split_args


if __name__ == "__main__":
    # ioc_options, run_options = ioc_arg_parser(
    #     default_prefix="sim:", desc=dedent(TrinamicBoardIOC.__doc__)
    # )

    # print(ioc_options)
    # print(run_options)

    """Primary command-line entry point"""
    parser, split_args = create_parser()
    args = parser.parse_args()
    print(args)
    ioc_options, run_options = split_args(args)

    ioc = TrinamicBoardIOC(
        address=args.address,
        host=args.host,
        port=args.port,
        motor_banks=args.motor_banks,
        **ioc_options,
    )
    run(ioc.pvdb, **run_options)
