import attr
from typing import Dict
from .__init__ import ureg
from .__init__ import pint
import logging 

def validate_backlash_direction(instance, attribute, value):
    if value not in [-1, 1]:
        raise ValueError(f"Backlash direction must be -1 or 1, got {value}")

def validate_quantity(instance, attribute, value):
    if not isinstance(value, pint.Quantity):
        raise TypeError("this value must be a pint.Quantity")
    
def quantity_converter(input_value):
    if isinstance(input_value, str):
        return ureg.Quantity(input_value)
    elif isinstance(input_value, ureg.Quantity):
        return input_value
    else:
        raise TypeError('Value must be either a string that can be interpreted as a Quantity or a Quantity already')

def validate_user_limits(instance, attribute, value):
    # Adjusted limits considering the user offset
    adjusted_negative_limit = instance.negative_user_limit + instance.user_offset
    adjusted_positive_limit = instance.positive_user_limit + instance.user_offset

    if (adjusted_negative_limit < 0) or (adjusted_positive_limit > instance.stage_motion_limit_RBV):
        logging.error(f"User limits must not exceed the stage motion limits after considering the user offset")

@attr.define
class AxisParameters:
    configurable_parameters: Dict[int, int] = attr.field(factory=dict)

    backlash_direction: int = attr.field(default=1, validator=validate_backlash_direction)
    # invert axis direction is not implemented yet.
    invert_axis_direction: bool = attr.field(default=False) # invert user coordinate representation
    # Custom unit conversion factor (e.g., steps to mm or steps to radians)
    steps_to_realworld_conversion_quantity: pint.Quantity = attr.field(
        default='1 steps/mm', validator=validate_quantity, converter=quantity_converter)

    # Base unit for real-world measurements (e.g., mm for linear axes, radian for rotational axes)
    base_realworld_unit: pint.Unit = attr.field(default=ureg.mm, converter=ureg.Unit)
    
    backlash: pint.Quantity = attr.field(default='1.0 mm', validator=validate_quantity, converter=quantity_converter)

    invert_limit_values: bool = attr.field(default=False)
    actual_coordinate_RBV: pint.Quantity = attr.field(default='0.0 mm', validator=validate_quantity, converter=quantity_converter)
    target_coordinate: pint.Quantity = attr.field(default='0.0 mm', validator=validate_quantity, converter=quantity_converter)
    # this one is automatically set on home_awit_and_set_limits operation. initially set large to avoid issues on configuration loading.
    stage_motion_limit_RBV: pint.Quantity = attr.field(default='99999999 mm', validator=validate_quantity, converter=quantity_converter)
    # user limits must always lie within the stage motion limits. It is validated for that when set. They are used in the motor motions to ensure that the motor does not move beyond the stage motion limits.
    user_offset: pint.Quantity = attr.field(default='50.0 mm', validator=validate_quantity, converter=quantity_converter)
    negative_user_limit: pint.Quantity = attr.field(default='-40 mm', validator=[validate_quantity, validate_user_limits], converter=quantity_converter)
    positive_user_limit: pint.Quantity = attr.field(default='150 mm', validator=[validate_quantity, validate_user_limits], converter=quantity_converter)

    # some flags to indicate the state of the axis
    is_moving_RBV: bool = attr.field(default=False)
    is_homed_RBV: bool = attr.field(default=False)
    is_position_reached_RBV: bool = attr.field(default=False)
    
    # internal states:    
    is_move_interrupted: bool = attr.field(default=False) # this flag is set when the motion is interrupted by a limit switch or a stop command. It is reset when the motion is restarted.
    update_interval_nonmoving: float = attr.field(default=5.0) # interval in seconds to update the axis parameters from the board. This is increased during a move to 0.1s. 
    update_interval_moving: float = attr.field(default=0.1) # interval in seconds to update the axis parameters from the board. This is increased during a move to 0.1s.
    # axis description
    axis_number: int = attr.field(default=0) # axis number on the board
    short_id: str = attr.field(default="Motor1") # short ID for the axis, should be alphanumeric
    description: str = attr.field(default="TMCM-6214 Axis") # description of the axis

    # List of attribute names to be converted into PVs
    pv_attributes: list = [
        'actual_coordinate_RBV', 
        'target_coordinate',
        'negative_user_limit',
        'positive_user_limit',
        'stage_motion_limit_RBV',
        'user_offset',
        'is_moving_RBV',
        'is_homed_RBV',
        'is_position_reached_RBV',
        'short_id']

    def set_actual_coordinate_RBV_by_steps(self, steps: int):
        """Sets the actual coordinate (Read-Back Value) by converting from steps to real-world units. It adds the user_offset to the conversion result, maintaining the intended offset in the actual position."""
        self.actual_coordinate_RBV = self.steps_to_real_world(steps) - self.user_offset

    def get_target_coordinate_in_steps(self, target_coordinate:ureg.Quantity) -> int:
        """Calculates the target coordinate in step units. It first adds the user_offset from the target_coordinate, which is in real-world units, and then converts the result to steps."""
        return self.real_world_to_steps(target_coordinate + self.user_offset)

    def set_target_coordinate_by_steps(self, steps: int):
        """Sets the target coordinate by converting from steps to real-world units. Similar to set_actual_coordinate_RBV_by_steps, it subtracts the user_offset to the conversion result."""
        self.target_coordinate = self.steps_to_real_world(steps) - self.user_offset

    def steps_to_real_world(self, steps: int) -> pint.Quantity:
        """
        Convert steps to real-world units.

        :param steps: Number of steps.
        :return: The equivalent distance or angle in real-world units.
        """
        result = ureg.Quantity(steps, 'steps') / self.steps_to_realworld_conversion_quantity # * self.base_realworld_unit
        # check if the result is compatible with the base unit
        if not result.is_compatible_with(self.base_realworld_unit):
            logging.error(f"Conversion of {steps} steps to real-world units failed. Problem in conversion quantity or base realworld unit.")
        return result

    def real_world_to_steps(self, distance_or_angle: pint.Quantity) -> int:
        """
        Convert real-world units (distance or angle) to steps.

        :param distance_or_angle: Distance or angle in real-world units.
        :return: The equivalent number of steps.
        """
        result = (distance_or_angle * (self.steps_to_realworld_conversion_quantity)).to('steps')
        # check if the result is compatible with the base unit
        if not result.is_compatible_with('steps'):
            logging.error(f"Conversion of {distance_or_angle} to steps failed. Problem in conversion quantity or base realworld unit.")
        return int(result.magnitude)