import attr
from typing import Dict
from .__init__ import ureg
from .__init__ import pint

def validate_backlash_direction(instance, attribute, value):
    if value not in [-1, 1]:
        raise ValueError(f"Backlash direction must be -1 or 1, got {value}")

def validate_quantity(instance, attribute, value):
    if not isinstance(value, pint.Quantity):
        raise TypeError("steps_to_realworld must be a pint.Quantity")

# Similar validators can be defined for other attributes

@attr.define
class AxisParameters:
    configurable_parameters: Dict[int, int] = attr.field(factory=dict)

    backlash_direction: int = attr.field(default=1, validator=validate_backlash_direction)
    invert_axis_direction: bool = attr.field(default=False) # invert user coordinate representation
    # Custom unit conversion factor (e.g., steps to mm or steps to radians)
    steps_to_realworld_conversion_quantity: pint.Quantity = attr.field(
        default='1 steps/mm', validator=validate_quantity, converter=ureg)

    # Base unit for real-world measurements (e.g., mm for linear axes, radian for rotational axes)
    base_realworld_unit: pint.Unit = attr.field(default=ureg.mm, converter=ureg.Unit)
    
    backlash: pint.Quantity = attr.field(default='1.0 mm', validator=validate_quantity, converter=ureg)

    invert_limit_values: bool = attr.field(default=False)
    actual_step_coordinate_RBV: int = attr.field(default=0)
    target_step_coordinate_RBV: int = attr.field(default=0)
    actual_coordinate_RBV: pint.Quantity = attr.field(default='0.0 mm', validator=validate_quantity, converter=ureg)
    target_coordinate: pint.Quantity = attr.field(default='0.0 mm', validator=validate_quantity, converter=ureg)
    negative_limit: pint.Quantity = attr.field(default='-100 mm', validator=validate_quantity, converter=ureg)
    positive_limit: pint.Quantity = attr.field(default='100 mm', validator=validate_quantity, converter=ureg)
    user_offset: pint.Quantity = attr.field(default='0.0 mm', validator=validate_quantity, converter=ureg)

    is_moving_RBV: bool = attr.field(default=False)
    is_homed_RBV: bool = attr.field(default=False)
    is_position_reached_RBV: bool = attr.field(default=False)

    axis_number: int = attr.field(default=0) # axis number on the board
    short_id: str = attr.field(default="Motor1") # short ID for the axis, should be alphanumeric
    description: str = attr.field(default="TMCM-6214 Axis") # description of the axis

    # List of attribute names to be converted into PVs
    pv_attributes: list = [
        'backlash_direction', 
        'invert_axis_direction', 
        'actual_coordinate', 
        'short_id']

    def steps_to_real_world(self, steps: int) -> pint.Quantity:
        """
        Convert steps to real-world units.

        :param steps: Number of steps.
        :return: The equivalent distance or angle in real-world units.
        """
        result = ureg.Quantity(steps, 'steps') / self.steps_to_realworld_conversion_quantity # * self.base_realworld_unit
        # check if the result is compatible with the base unit
        if not result.is_compatible_with(self.base_realworld_unit):
            raise ValueError(f"Conversion of {steps} steps to real-world units failed. Problem in conversion quantity or base realworld unit.")
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
            raise ValueError(f"Conversion of {distance_or_angle} to steps failed. Problem in conversion quantity or base realworld unit.")
        return int(result.magnitude)