from typing import Union
from .board_control import BoardControl
from .axis_parameters import AxisParameters
from .__init__ import ureg

class MotionControl:
    """high-level interface for controlling motion of the motors"""
    def __init__(self, board_control: BoardControl):
        self.board_control = board_control

    def home_await_and_set_limits(self, axis_index: int):
        """ kicks off the homing process, waits for it to complete, and sets the stage motion range limit to the end switch distance. """
        # indicate the stage is not homed.
        self.board_control.boardpar.axes_parameters[axis_index].is_homed_RBV = False
        # home the axis
        self.board_control.home_axis(axis_index)
        # wait for the moves to complete
        self.board_control.await_move_completion(axis_index)
        # set the stage motion range limit to the end switch distance
        range_steps = self.board_control.get_end_switch_distance(axis_index)
        range_realworld = self.board_control.boardpar.axes_parameters[axis_index].steps_to_real_world(range_steps)
        self.board_control.boardpar.axes_parameters[axis_index].stage_motion_limit_RBV = range_realworld
        # now we re-set the user limits to re-validate that they lie within the stage motion limit
        self.board_control.boardpar.axes_parameters[axis_index].negative_user_limit = self.board_control.boardpar.axes_parameters[axis_index].negative_user_limit
        self.board_control.boardpar.axes_parameters[axis_index].positive_user_limit = self.board_control.boardpar.axes_parameters[axis_index].positive_user_limit
        # indicate the stage is now homed.
        self.board_control.boardpar.axes_parameters[axis_index].is_homed_RBV = True # should be finished now. 

    def _prepare_axis_for_motion(self, axis_params: AxisParameters):
        # make sure we have an updated state of the axis:
        self.board_control.update_axis_parameters(axis_params.axis_number)

        # ensure that the axis is homed before moving
        if not axis_params.is_homed_RBV:
            raise ValueError("Axis must be homed before moving.")

        # ensure that the axis is not moving before moving
        if axis_params.is_moving_RBV:
            raise ValueError("Axis must be stopped before moving.")

    def _calculate_adjusted_target(self, axis_params: AxisParameters, target_coordinate: ureg.Quantity, absolute_or_relative: str = 'absolute') -> ureg.Quantity:
        # Validate target coordinate
        target_coordinate = ureg(target_coordinate) # interpret as pint.Quantity
        # if not isinstance(target_coordinate, ureg.Quantity):
        #     raise TypeError("target_coordinate must be a pint.Quantity")
        
        if absolute_or_relative == 'absolute':
            adjusted_target = target_coordinate
        elif absolute_or_relative == 'relative':
            adjusted_target = axis_params.actual_coordinate_RBV + target_coordinate
        else:
            raise ValueError("absolute_or_relative must be 'absolute' or 'relative'.")
        return adjusted_target

    def move_to_coordinate_with_backlash(self, axis_index_or_name: Union[int, str], target_coordinate: ureg.Quantity, absolute_or_relative: str = 'absolute'):
        """
        Move a motor axis to a specified position in real space, considering backlash.

        :param axis_index_or_name: Axis index or its short_id.
        :param target_coordinate: Target position as a pint.Quantity with length unit.
        :param absolute_or_relative: 'absolute' for absolute position, 'relative' for relative movement.
        """
        axis_index = self._resolve_axis_index(axis_index_or_name)
        axis_params = self.board_control.boardpar.axes_parameters[axis_index]
        absolute_or_relative = absolute_or_relative.lower()

        # Prepare the axis for motion
        self._prepare_axis_for_motion(axis_params)

        # Validate target coordinate
        adjusted_target = self._calculate_adjusted_target(axis_params, target_coordinate, absolute_or_relative)

        # Check motion limits, taking bidirectional backlash into account
        if not ((axis_params.negative_user_limit + axis_params.backlash) <= adjusted_target <= (axis_params.positive_user_limit - axis_params.backlash)):
            raise ValueError("Target position is outside of the axis user motion limits.")

        # Apply backlash correction if needed
        backlash_needed = axis_params.backlash_direction * (adjusted_target - axis_params.actual_coordinate_RBV).magnitude > 0
        if backlash_needed:
            adjusted_backlashed_target += axis_params.backlash * axis_params.backlash_direction
        else:
            adjusted_backlashed_target = adjusted_target

        # Perform the movement
        steps = axis_params.get_target_coordinate_in_steps(adjusted_backlashed_target)
        self.board_control.move_axis(axis_index, steps)
        # wait for the move to complete
        self.board_control.await_move_completion(axis_index)
        # apply backlash move if needed
        if backlash_needed:
            steps = axis_params.get_target_coordinate_in_steps(adjusted_target)
            self.board_control.move_axis(axis_index, steps)
            self.board_control.await_move_completion(axis_index)

        # Update AxisParameters after the move
        self.board_control.update_axis_parameters(axis_index)

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

    async def home_and_wait(self, axis_index: int):
        # Implement logic to home the axis and wait until motion is complete
        pass

    # Additional advanced motion control methods