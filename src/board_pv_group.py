import attr
import pint

from typing import Union

from caproto.server import PVGroup, ioc_arg_parser, run
from src.board_parameters import BoardParameters
from src.board_control import BoardControl
from src.configuration_management import ConfigurationManagement
from src.create_pv_for_attribute import create_pvs_for_attribute
from src.motion_control import MotionControl

ureg = pint.UnitRegistry()

# I don't think we can use attrs here. 
# @attr.define
class BoardPVGroup(PVGroup):
    """exposes the board and motor control parameters as PVs on the EPICS channel-access bus with actions to control the motors"""
    board_parameters: BoardParameters = None # board and its axes parameters
    board_control: BoardControl = None # low level comms operations
    motion_control: MotionControl = None # high level motion operations
    sleep_duration: float = 5.0  # Default sleep duration in seconds

    def __init__(self, *args, board_parameters: BoardParameters, **kwargs):
        super().__init__(*args, **kwargs)
        self.board_parameters = board_parameters

        # Create PVs for board-level parameters specified in pv_attributes
        for attr_name in board_parameters.pv_attributes:
            attr_value = getattr(board_parameters, attr_name)
            create_pvs_for_attribute(attr_name, attr_value, board_parameters, self)

        # Create PVs for axis parameters specified in pv_attributes
        for axis_index, axis_param in enumerate(board_parameters.axes_parameters):
            for axis_attr_name in axis_param.pv_attributes:
                axis_attr_value = getattr(axis_param, axis_attr_name)
                pv_name = f"axis{axis_index}_{axis_attr_name}"
                create_pvs_for_attribute(pv_name, axis_attr_value, axis_param, self)

        # Initialize the motor control with the connection details from board_parameters
        self.board_control = BoardControl(self.board_parameters.connection_string)


    def __attrs_post_init__(self):
        # Initialize the motor control with the connection details from board_parameters
        self.board_control = BoardControl(self.board_parameters.connection_string)

    # set the caproto regular refresh intervals and what to do here. 
    @PVGroup.startup
    async def startup(self, instance, async_lib):
        # This method will be called when the IOC starts up
        self.loop = async_lib.library

        # Schedule a periodic callback
        self.periodic_task = self.loop.create_task(self.periodic_refresh())

   # do this at every refresh
    async def periodic_refresh(self):
        while True:
            try:
                # Perform the refresh
                await self.refresh_internal_state()

                # Wait for a specified interval (e.g., 1 second)
                await asyncio.sleep(self.sleep_duration)

            except asyncio.CancelledError:
                # Handle task cancellation
                break
            except Exception as e:
                # Handle other exceptions
                print(f"Error in periodic_refresh: {e}")


# Example usage
if __name__ == '__main__':
    board_params = BoardParameters()  # Initialize with default or configured values
    ioc_options, run_options = ioc_arg_parser(
        default_prefix='tmcm6214:',
        desc='Run an IOC for the TMCM-6214 motor controller board.')
    ioc = BoardPVGroup(board_parameters=board_params, **ioc_options)
    run(ioc.pvdb, **run_options)