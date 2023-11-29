import attr
import pint

from typing import Union

from caproto.server import PVGroup, ioc_arg_parser, run
from src.board_parameters import BoardParameters
from src.board_control import BoardControl
from src.configuration_management import ConfigurationManagement
from src.create_pv_for_attribute import create_pvs_for_attribute

ureg = pint.UnitRegistry()

@attr.define
class BoardPVGroup(PVGroup):
    """exposes the board and motor control parameters as PVs on the EPICS channel-access bus with actions to control the motors"""
    board_parameters: BoardParameters = attr.field()
    board_control: BoardControl = attr.field()
    sleep_duration: float = attr.field(default=5.0)  # Default sleep duration in seconds

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



    async def move_to_realspace_position(self, axis: Union[int, str], position: pint.Quantity, absolute_or_relative: str = 'absolute') -> None:
        """
        Move a motor axis to a specified position in real space.

        :param axis: Axis index or its short_id.
        :param position: Target position as a pint.Quantity with length unit.
        :param absolute_or_relative: 'absolute' for absolute position, 'relative' for relative movement.
        """
        axis_index = self._resolve_axis_index(axis)
        axis_params = self.board_parameters.axes_parameters[axis_index]

        # Validate position
        if not hasattr(position, 'units') or position.units not in [ureg.meter, ureg.mm, ureg.cm]:
            raise ValueError("Position must have a unit of length (m, mm, or cm).")

        if absolute_or_relative == 'absolute':
            target_position = position
        elif absolute_or_relative == 'relative':
            target_position = axis_params.actual_coordinate + position
        else:
            raise ValueError("absolute_or_relative must be 'absolute' or 'relative'.")

        if not (axis_params.negative_limit <= target_position <= axis_params.positive_limit):
            raise ValueError("Target position is outside of the axis motion limits.")

        # Convert target position to steps
        steps = axis_params.convert_to_steps(target_position)

        # Perform the movement
        await self.board_control.move_motor(axis_index, steps)

        # Update AxisParameters after the move
        axis_params.actual_coordinate = target_position
        # You may also want to update other parameters as necessary

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


    async def read_axis_state(self, axis_index: int) -> None:
        """
        Read and update the state of a specific motor axis.

        :param axis_index: Index of the motor axis.
        """
        # Implementation for reading axis state
        pass

    # Add other necessary methods and functionalities

# Example usage
if __name__ == '__main__':
    board_params = BoardParameters()  # Initialize with default or configured values
    ioc_options, run_options = ioc_arg_parser(
        default_prefix='tmcm6214:',
        desc='Run an IOC for the TMCM-6214 motor controller board.')
    ioc = BoardPVGroup(board_parameters=board_params, **ioc_options)
    run(ioc.pvdb, **run_options)