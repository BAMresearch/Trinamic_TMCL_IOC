#!/usr/bin/env python3
from builtins import NotImplementedError
import logging
import asyncio
import pathlib
from caproto.server import (
    pvproperty,
    PVGroup,
    run,
    SubGroup,
    get_pv_pair_wrapper,
    template_arg_parser,
)
from caproto.server.autosave import AutosaveHelper, RotatingFileManager, autosaved
from comms import sendReceiveBufferSocket
import numpy as np
from pytrinamic.tmcl import TMCLRequest, TMCLReply, TMCLCommand

# get logger instance for message passing
logger = logging.getLogger("caproto")
# copying a lot from the pcdshub/fluke_985 github comms class:
# largely copied from the hotplate class.

RBV_SUFFIX = "_RBV"

# Create _two_ PVs with a single pvproperty_with_rbv:
pvproperty_with_rbv = get_pv_pair_wrapper(
    setpoint_suffix="", readback_suffix=RBV_SUFFIX
)


def clamp(variable, minimum, maximum):
    return max(min(variable, maximum), minimum)


class trinamicBoardPVGroup(PVGroup):
    "group of PVs for motor control board produced by the 'TRINAMIC Motion Control GmbH & Co. KG', where the prefix is defined on instantiation"

    # again copying some style from the fluke ioc example
    def __init__(self, *args, host, port, address, motor_bank, **kwargs):
        super().__init__(*args, **kwargs)
        # note port is second argument of host.. see startup..
        self._host: str = host
        self._port: int = port
        self._motor_bank: int = motor_bank
        self._address: int = address
        self._consecutive_timeouts = 0

    host = pvproperty(
        value="",
        max_length=60,
        name="host",
        doc="IP address at which we can reach the motor control board",
        read_only=True,
        record="stringin",
    )

    @host.startup
    async def host(self, instance, async_lib):
        await self.host.write(self._host)

    port = pvproperty(
        value=0,
        name="port",
        doc="Port at which we can reach the motor control board",
        read_only=True,
        record="longin",
    )

    @port.startup
    async def port(self, instance, async_lib):
        await self.port.write(self._port)

    motor_bank = pvproperty(
        value=0,
        name="motor_bank_number",
        doc="Motor bank number",
        read_only=True,
        record="ai",
    )

    @motor_bank.startup
    async def motor_bank(self, instance, async_lib):
        await self.motor_bank.write(self._motor_bank)

    address = pvproperty(
        value=0,
        name="address",
        doc="Address of the motor control board",
        read_only=True,
        record="ai",
    )

    @address.startup
    async def address(self, instance, async_lib):
        await self.address.write(self._address)

    request_timeout = pvproperty(
        value=1.0,
        name="request_timeout",
        record="ai",
        lower_ctrl_limit=0.1,
        upper_ctrl_limit=100,
    )

    data_timeout = pvproperty(
        value=60.0,
        name="data_timeout",
        record="ai",
        lower_ctrl_limit=1,
        upper_ctrl_limit=1000,
    )

    # not sure what these do...
    autosaved(request_timeout)
    autosaved(data_timeout)

    async def processResponse(self, response: TMCLReply, command_type: int):
        """Process the response from the commands to catch errors, returns extracted values"""

        module_address = response.module_address
        status = response.status
        command = response.command
        value = response.value

        if not (module_address == self._address):
            return  # Apparently the message was not directed at us

        if not (status == 100):
            error = f"An error status was sent by the motor control board number {self._motor_bank} of address {self._address}"
            logger.error(error)
            return

        if command == TMCLCommand.GAP:
            if command_type == 0:
                await self.position.target_position.readback.write(value)
            elif command_type == 1:
                await self.position.actual_position.write(value)
            elif command_type == 3:
                await self.velocity.actual_speed.write(value)
            elif command_type == 4:
                await self.velocity.max_positioning_speed.readback.write(value)
            elif command_type == 5:
                await self.acceleration.max_acceleration.readback.write(value)
            elif command_type == 6:
                await self.motor_params.max_current.readback.write(value)
            elif command_type == 7:
                await self.motor_params.standby_current.readback.write(value)
            elif command_type == 8:
                await self.position.position_reached.write(value)
            elif command_type == 14:
                await self.limit_switches.limit_switch_swap.readback.write(value)
            elif command_type == 17:
                await self.acceleration.max_deceleration.readback.write(value)
            elif command_type == 24:
                await self.limit_switches.right_limit_switch_polarity.readback.write(
                    value
                )
            elif command_type == 25:
                await self.limit_switches.left_limit_switch_polarity.readback.write(
                    value
                )
            elif command_type == 32:
                await self.motor_params.dc_step_time.readback.write(value)
            elif command_type == 140:
                await self.motor_params.microstep_resolution.readback.write(value)
            elif command_type == 193:
                await self.referenceSearch.reference_search_mode.readback.write(value)
            elif command_type == 194:
                await self.referenceSearch.reference_search_speed.readback.write(value)
            elif command_type == 196:
                await self.referenceSearch.end_switch_distance.write(value)
            elif command_type == 206:
                await self.motor_state.actual_load.write(value)
            elif command_type == 208:
                await self.errors.motor_driver_error_flags.write(value)
        elif np.in1d(
            [command],
            [TMCLCommand.SAP, TMCLCommand.RFS, TMCLCommand.MVP, TMCLCommand.MST],
        ):
            # For now we pass every response that is not related to the GAP command.
            pass
        else:
            raise NotImplementedError

    async def _scan_axis_parameter(self, cmd_type: int):
        cmd = TMCLRequest(
            self._address, TMCLCommand.GAP, cmd_type, self._motor_bank, 0
        ).toBuffer()
        response = await sendReceiveBufferSocket(self.host.value, self.port.value, cmd)
        await self.processResponse(response, cmd_type)

    async def _set_axis_parameter(self, cmd_type: int, value: float) -> None:
        host, port = self.host.value, self.port.value
        cmd = TMCLRequest(
            self._address, TMCLCommand.SAP, cmd_type, self._motor_bank, value,
        ).toBuffer()
        response = await sendReceiveBufferSocket(host, port, cmd)
        await self.processResponse(response, cmd_type)

    async def _get_axis_parameter(self, cmd_type: int) -> None:
        host, port = self.host.value, self.port.value
        cmd = TMCLRequest(
            self._address, TMCLCommand.GAP, cmd_type, self._motor_bank, 0,
        ).toBuffer()
        response = await sendReceiveBufferSocket(host, port, cmd)
        await self.processResponse(response, cmd_type)

    @SubGroup
    class referenceSearch(PVGroup):

        reference_search = pvproperty(
            value=0,
            record="ao",
            doc="This command starts or stops the built-in reference search algorithm.",
        )

        @reference_search.putter
        async def reference_search(self, instance, value):
            host, port = self.parent.host.value, self.parent.port.value
            logger.debug(f"Perform reference search procedure.")
            cmd_type = value
            cmd = TMCLRequest(
                self.parent._address,
                TMCLCommand.RFS,
                cmd_type,
                self.parent._motor_bank,
                0,
            ).toBuffer()
            response = await sendReceiveBufferSocket(host, port, cmd)
            await self.parent.processResponse(response, cmd_type)

        reference_search_mode = pvproperty_with_rbv(
            value=0,
            record="ao",
            doc="1 Search left stop switch only. 2 Search right stop switch, thensearch left stop switch. 3 Search right stop switch, thensearch left stop switch from bothsides. 4 Search left stop switch from bothsides. 5 Search home switch in negative di-rection, reverse the direction whenleft stop switch reached. 6 Search home switch in positive di-rection, reverse the direction whenright stop switch reached. 7 Search home switch in positive di-rection, ignore end switches. 8 Search home switch in negative di-rection, ignore end switches. Additional functions: • Add 128 to a mode value for invertingthe home switch (can be used with mode5. . . 8). • Add 64 to a mode for driving the right in-stead of the left reference switch (can beused with mode 1. . . 4).",
        )

        @reference_search_mode.setpoint.putter
        async def reference_search_mode(self, instance, value):
            value = int(clamp(value, 1, 8))
            logger.debug(f"Set reference search mode to {value}")
            cmd_type = 193
            await self.parent.parent._set_axis_parameter(cmd_type, value)

        @reference_search_mode.readback.scan(period=5.0)
        async def reference_search_mode(self, instance, async_lib):
            await self.parent.parent._scan_axis_parameter(193)

        reference_search_speed = pvproperty_with_rbv(
            value=0,
            record="ao",
            doc="This value specifies the speed for roughly searching the reference switch.",
        )

        @reference_search_speed.setpoint.putter
        async def reference_search_speed(self, instance, value):
            value = int(clamp(value, 0, 7999774))
            logger.debug(f"Set reference search speed to {value}")
            cmd_type = 194
            await self.parent.parent._set_axis_parameter(cmd_type, value)

        @reference_search_speed.readback.scan(period=5.0)
        async def reference_search_speed(self, instance, async_lib):
            await self.parent.parent._scan_axis_parameter(194)

        end_switch_distance = pvproperty(
            value=0,
            record="ao",
            name=f"end_switch_distance{RBV_SUFFIX}",
            read_only=True,
            doc="This parameter provides the distance between the end switches after executing the RFS command (with reference search mode 2 or 3).",
        )

        @end_switch_distance.scan(period=5.0)
        async def end_switch_distance(self, instance, async_lib):
            await self.parent._scan_axis_parameter(196)

    @SubGroup
    class position(PVGroup):

        target_position = pvproperty_with_rbv(
            value=0,
            record="ao",
            name="target_position",
            doc="Target position of the specified motor.",
        )

        @target_position.setpoint.putter
        async def target_position(self, instance, value):
            value = int(clamp(value, -2147483648, 2147483647))
            logger.debug(f"Setting the target position to {value}μsteps.")
            cmd_type = 0
            await self.parent.parent._set_axis_parameter(cmd_type, value)

        @target_position.readback.scan(period=1.0)
        async def target_position(self, instance, async_lib):
            await self.parent.parent._scan_axis_parameter(0)

        move_to_position_absolute = pvproperty(
            value=0,
            record="ao",
            doc="This command starts or stops the built-in reference search algorithm.",
        )

        @move_to_position_absolute.putter
        async def move_to_position_absolute(self, instance, value):
            host, port = self.parent.host.value, self.parent.port.value
            value = int(clamp(value, -2147483648, 2147483647))
            logger.debug(f"Move motor to absolute position {value}.")
            cmd_type = 0
            cmd = TMCLRequest(
                self.parent._address,
                TMCLCommand.MVP,
                cmd_type,
                self.parent._motor_bank,
                value,
            ).toBuffer()
            response = await sendReceiveBufferSocket(host, port, cmd)
            await self.parent.processResponse(response, cmd_type)

        motor_stop = pvproperty(
            value=0,
            record="ao",
            doc="The motor is instructed to stop with a soft stop.",
        )

        @motor_stop.putter
        async def motor_stop(self, instance, value):
            host, port = self.parent.host.value, self.parent.port.value
            logger.debug(f"Instructing motor to stop.")
            cmd_type = 0
            cmd = TMCLRequest(
                self.parent._address,
                TMCLCommand.MST,
                cmd_type,
                self.parent._motor_bank,
                value,
            ).toBuffer()
            response = await sendReceiveBufferSocket(host, port, cmd)
            await self.parent.processResponse(response, cmd_type)

        actual_position = pvproperty(
            value=0,
            record="ao",
            name=f"actual_position{RBV_SUFFIX}",
            doc="Actual position of the specified motor.",
            read_only=True,
        )

        @actual_position.scan(period=0.1)
        async def actual_position(self, instance, async_lib):
            await self.parent._scan_axis_parameter(1)

        position_reached = pvproperty(
            value=0,
            record="ao",
            name=f"position_reached{RBV_SUFFIX}",
            doc="This flag is always set when target position and actual position are equal.",
            read_only=True,
        )

        @position_reached.scan(period=0.1)
        async def position_reached(self, instance, async_lib):
            await self.parent._scan_axis_parameter(8)

    @SubGroup
    class velocity(PVGroup):
        max_positioning_speed = pvproperty_with_rbv(
            value=0, record="ai", doc="The maximum speed used for positioning ramps"
        )

        @max_positioning_speed.setpoint.putter
        async def max_positioning_speed(self, instance, value):
            value = int(clamp(value, 0, 7999774))
            logger.debug(f"Setting the motor's max positioning speed to {value}pps.")
            cmd_type = 4
            await self.parent.parent._set_axis_parameter(cmd_type, value)

        @max_positioning_speed.readback.scan(period=5.0)
        async def max_positioning_speed(self, instance, async_lib):
            await self.parent.parent._scan_axis_parameter(4)

        actual_speed = pvproperty(
            value=0,
            record="ao",
            name=f"actual_speed{RBV_SUFFIX}",
            doc="The actual speed of the motor.",
            read_only=True,
        )

        @actual_speed.scan(period=0.1)
        async def actual_speed(self, instance, async_lib):
            await self.parent._scan_axis_parameter(3)

    @SubGroup
    class acceleration(PVGroup):
        max_acceleration = pvproperty_with_rbv(
            value=0,
            record="ai",
            doc="Maximum acceleration in positioning ramps. Acceleration and deceleration value in velocity mode.",
        )

        @max_acceleration.setpoint.putter
        async def max_acceleration(self, instance, value):
            value = int(clamp(value, 0, 7629278))
            logger.debug(f"Setting the motor's max acceleration to {value}pps^2.")
            cmd_type = 5
            await self.parent.parent._set_axis_parameter(cmd_type, value)

        @max_acceleration.readback.scan(period=5.0)
        async def max_acceleration(self, instance, async_lib):
            await self.parent.parent._scan_axis_parameter(5)

        max_deceleration = pvproperty_with_rbv(
            value=0,
            record="ai",
            doc="Maximum deceleration in positioning ramps. Used to decelerate from maximum positiong speed (axis parameter 4) to velocity V1.",
        )

        @max_deceleration.setpoint.putter
        async def max_deceleration(self, instance, value):
            value = int(clamp(value, 0, 7629278))
            logger.debug(f"Setting the motor's max deceleration to {value}pps^2.")
            cmd_type = 17
            await self.parent.parent._set_axis_parameter(cmd_type, value)

        @max_deceleration.readback.scan(period=5.0)
        async def max_deceleration(self, instance, async_lib):
            await self.parent.parent._scan_axis_parameter(17)

    @SubGroup
    class limit_switches(PVGroup):

        right_limit_switch_polarity = pvproperty_with_rbv(
            value=0,
            record="ao",
            doc="Setting this parameter to 1 inverts the logic state of the right limit switch input.",
        )

        @right_limit_switch_polarity.setpoint.putter
        async def right_limit_switch_polarity(self, instance, value):
            value = int(clamp(value, 0, 1))
            logger.debug(f"Set right limit switch polarity to {value}.")
            cmd_type = 24
            await self.parent.parent._set_axis_parameter(cmd_type, value)

        @right_limit_switch_polarity.readback.scan(period=5.0)
        async def right_limit_switch_polarity(self, instance, async_lib):
            await self.parent.parent._scan_axis_parameter(24)

        left_limit_switch_polarity = pvproperty_with_rbv(
            value=0,
            record="ao",
            doc="Setting this parameter to 1 inverts the logic state of the left limit switch input.",
        )

        @left_limit_switch_polarity.setpoint.putter
        async def left_limit_switch_polarity(self, instance, value):
            value = int(clamp(value, 0, 1))
            logger.debug(f"Set left limit switch polarity to {value}.")
            cmd_type = 25
            await self.parent.parent._set_axis_parameter(cmd_type, value)

        @left_limit_switch_polarity.readback.scan(period=5.0)
        async def left_limit_switch_polarity(self, instance, async_lib):
            await self.parent.parent._scan_axis_parameter(25)

        limit_switch_swap = pvproperty_with_rbv(
            value=0,
            record="ao",
            doc="Swap the left and right limit switches when set to 1.",
        )

        @limit_switch_swap.setpoint.putter
        async def limit_switch_swap(self, instance, value):
            value = int(clamp(value, 0, 1))
            logger.debug(f"Set limit switch swap value to {value}.")
            cmd_type = 14
            await self.parent.parent._set_axis_parameter(cmd_type, value)

        @limit_switch_swap.readback.scan(period=5.0)
        async def limit_switch_swap(self, instance, async_lib):
            await self.parent.parent._scan_axis_parameter(14)

    @SubGroup
    class motor_params(PVGroup):

        max_current = pvproperty_with_rbv(
            value=0,
            record="ao",
            name="max_current",
            doc="Motor current used when motor is running. The maximum value is 255 which means 100% of the maximum current of the module. Refer to manual for more details.",
        )

        @max_current.setpoint.putter
        async def max_current(self, instance, value):
            value = int(clamp(value, 0, 255))
            logger.debug(f"Setting the motor's max current to {value/255.}%")
            cmd_type = 6
            await self.parent.parent._set_axis_parameter(cmd_type, value)

        @max_current.readback.scan(period=5.0)
        async def max_current(self, instance, async_lib):
            await self.parent.parent._scan_axis_parameter(6)

        standby_current = pvproperty_with_rbv(
            value=0,
            record="ao",
            name="standby_current",
            doc="The current used when the motor is not running. The maximum value is 255 which means 100% of the maximum current of the module. This value should be as low as possible so that the motor can cool down when it is not moving. Please see also parameter 214.",
        )

        @standby_current.setpoint.putter
        async def standby_current(self, instance, value):
            value = int(clamp(value, 0, 255))
            logger.debug(f"Setting the motor's standby current to {value/255.}%")
            cmd_type = 7
            await self.parent.parent._set_axis_parameter(cmd_type, value)

        @standby_current.readback.scan(period=5.0)
        async def standby_current(self, instance, async_lib):
            await self.parent.parent._scan_axis_parameter(7)

        microstep_resolution = pvproperty_with_rbv(
            value=0,
            record="ao",
            doc="Microstep resolutions per full step: 0=fullstep; 1=halfstep; 2=4 microsteps; 3=8 microsteps; 4=16 microsteps; 5=32 microsteps; 6=64 microsteps; 7=128 microsteps; 8=256 microsteps",
        )

        @microstep_resolution.setpoint.putter
        async def microstep_resolution(self, instance, value):
            value = int(clamp(value, 0, 8))
            logger.debug(
                f"Setting the microstep resolution to {2**value} microstep resolutions per full step."
            )
            cmd_type = 140
            await self.parent.parent._set_axis_parameter(cmd_type, value)

        @microstep_resolution.readback.scan(period=5.0)
        async def microstep_resolution(self, instance, async_lib):
            await self.parent.parent._scan_axis_parameter(140)

        dc_step_time = pvproperty_with_rbv(
            value=0,
            record="ao",
            name="dc_step_time",
            doc="This setting controls the reference pulse width for dcStep load measurement. It must be optimized for robust operation with maximum motor torque. A higher value allows higher torque and higher velocity, a lower value allows operation down to a lower velocity as set by VDCMIN.",
        )

        @dc_step_time.setpoint.putter
        async def dc_step_time(self, instance, value):
            value = int(clamp(value, 0, 255))
            logger.debug(f"Setting the motor's dc step time to {value/255.}%")
            cmd_type = 32
            await self.parent.parent._set_axis_parameter(cmd_type, value)

        @dc_step_time.readback.scan(period=5.0)
        async def dc_step_time(self, instance, async_lib):
            await self.parent.parent._scan_axis_parameter(32)

    @SubGroup
    class motor_state(PVGroup):

        actual_load = pvproperty(
            value=0,
            record="ao",
            name=f"actual_load{RBV_SUFFIX}",
            doc="Readout of the actual load value used for stall detection (stallGuard2).",
            read_only=True,
        )

        @actual_load.scan(period=0.1)
        async def actual_load(self, instance, async_lib):
            await self.parent._scan_axis_parameter(206)

    @SubGroup
    class errors(PVGroup):

        motor_driver_error_flags = pvproperty(
            value=0,
            record="ao",
            read_only=True,
            doc="Individual bits represent errors. See Trinamic manuals for detailed description.",
        )

        @motor_driver_error_flags.scan(period=1.0)
        async def motor_driver_error_flags(self, instance, async_lib):
            await self.parent._scan_axis_parameter(208)


def create_ioc(
    prefix: str,
    *,
    host: str,
    port: int,
    address: int,
    motor_bank: int,
    autosave: str,
    **ioc_options,
) -> trinamicBoardPVGroup:
    """Create a new IOC for motor control through PyTrinamic."""
    autosave = pathlib.Path(autosave).resolve().absolute()

    class trinamicMain(trinamicBoardPVGroup):
        autosave_helper = SubGroup(
            AutosaveHelper, file_manager=RotatingFileManager(autosave)
        )
        autosave_helper.filename = autosave

    return trinamicBoardPVGroup(
        prefix=prefix,
        host=host,
        port=port,
        address=address,
        motor_bank=motor_bank,
        **ioc_options,
    )


def create_parser():
    parser, split_args = template_arg_parser(
        default_prefix="pytrinamic:",
        desc="PyTrinamic motor control board",
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
        "--motor_bank", help="Motor number", required=True, type=int,
    )

    parser.add_argument(
        "--autosave",
        help="Path to the autosave file",
        default="autosave.json",
        type=str,
    )
    return parser, split_args


if __name__ == "__main__":
    """Primary command-line entry point"""
    parser, split_args = create_parser()
    args = parser.parse_args()
    ioc_options, run_options = split_args(args)

    ioc = create_ioc(
        autosave=args.autosave,
        host=args.host,
        port=args.port,
        address=args.address,
        motor_bank=args.motor_bank,
        **ioc_options,
    )

    run(ioc.pvdb, **run_options)
