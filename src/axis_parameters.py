import attr
from typing import Dict, Union
from . import ureg
import logging 

def validate_backlash_direction(instance, attribute, value):
    if value not in [-1, 1]:
        raise ValueError(f"Backlash direction must be -1 or 1, got {value}")

def validate_quantity(instance, attribute, value):
    if not isinstance(value, ureg.Quantity):
        raise TypeError("this value must be a ureg.Quantity")
    
def quantity_converter(input_value: Union[str, float, int, ureg.Quantity], target_unit:Union[str, ureg.Unit]=None):
    """
    Convert input_value to a ureg.Quantity. If input_value is a string, it is interpreted as a Quantity. If it is a float or int, it is converted to a Quantity using target_unit. If input_value is already a Quantity, it is returned unchanged.
    Parameters:
    input_value: value to convert
    target_unit: unit to use for conversion if input_value is a float or int
    """
    if isinstance(input_value, str):
        return ureg.Quantity(input_value)
    elif isinstance(input_value, ureg.Quantity):
        return input_value
    elif isinstance(input_value, Union[float, int]) and target_unit is not None:
        return ureg.Quantity(input_value, target_unit) # can deal with both str and ureg.Unit as target_unit
    else:
        raise TypeError('Value must be either: 1) a float or int with a target_unit specified, 2) a string that can be interpreted as a Quantity or 3) a Quantity already')

def validate_user_limits(instance, attribute, value):
    # Adjusted limits considering the user offset, adjusted to the EPICS definition. 
    adjusted_negative_limit = instance.negative_user_limit - instance.user_offset
    adjusted_positive_limit = instance.positive_user_limit - instance.user_offset

    if (adjusted_negative_limit < 0) or (adjusted_positive_limit > instance.stage_motion_limit_RBV):
        logging.error(f"User limits must not exceed the stage motion limits after considering the user offset")

@attr.define
class AxisParameters:
    configurable_parameters: Dict[int, int] = attr.field(factory=dict)

    # while these can be configured using configurable_parameters, I think it is nice to have access to them here. 
    velocity: ureg.Quantity = attr.field(default='1.0 mm/s', validator=validate_quantity, converter=quantity_converter)
    acceleration_duration: ureg.Quantity = attr.field(default='1.0 s', validator=validate_quantity, converter=quantity_converter)

    backlash_direction: int = attr.field(default=1, validator=validate_backlash_direction)
    # backlash_velocity: ureg.Quantity = attr.field(default=velocity, validator=validate_quantity, converter=quantity_converter)
    # backlash_acceleration_duration: ureg.Quantity = attr.field(default=acceleration_duration, validator=validate_quantity, converter=quantity_converter)

    # Custom unit conversion factor (e.g., steps to mm or steps to radians)
    steps_to_realworld_conversion_quantity: ureg.Quantity = attr.field(
        default='1 steps/mm', validator=validate_quantity, converter=quantity_converter)

    # Base unit for real-world measurements (e.g., mm for linear axes, radian for rotational axes)
    base_realworld_unit: ureg.Unit = attr.field(default=ureg.mm, converter=ureg.Unit)
    
    backlash: ureg.Quantity = attr.field(default='1.0 mm', validator=validate_quantity, converter=quantity_converter)

    invert_limit_values: bool = attr.field(default=False) # invert logical values before displaying them to the user
    # invert axis direction can be done on the board level via configurable_parameters, or here on the software level. 
    invert_axis_direction: bool = attr.field(default=False) # invert user coordinate representation, similar to EPICS DIRection field
    swap_limit_switches: bool = attr.field(default=False) # swap the limit switches when they are connected wrong. This gets inverted when the axis direction is inverted.

    actual_coordinate_RBV: ureg.Quantity = attr.field(default='0.0 mm', validator=validate_quantity, converter=quantity_converter)
    # # during a backlash move, the immediate target read from the board will deviate from the final target coordinate. 
    # immediate_target_coordinate_RBV: ureg.Quantity = attr.field(default='0.0 mm', validator=validate_quantity, converter=quantity_converter)
    # this is the eventual / final target coordinate. 
    target_coordinate: ureg.Quantity = attr.field(default='0.0 mm', validator=validate_quantity, converter=quantity_converter)
    # this one is automatically set on home_awit_and_set_limits operation. initially set large to avoid issues on configuration loading.
    stage_motion_limit_RBV: ureg.Quantity = attr.field(default='99999999 mm', validator=validate_quantity, converter=quantity_converter)
    # user limits must always lie within the stage motion limits. It is validated for that when set. They are used in the motor motions to ensure that the motor does not move beyond the stage motion limits.
    user_offset: ureg.Quantity = attr.field(default='50.0 mm', validator=validate_quantity, converter=quantity_converter)
    negative_user_limit: ureg.Quantity = attr.field(default='-40 mm', validator=[validate_quantity, validate_user_limits], converter=quantity_converter)
    positive_user_limit: ureg.Quantity = attr.field(default='150 mm', validator=[validate_quantity, validate_user_limits], converter=quantity_converter)

    # some flags to indicate the state of the axis
    is_moving_RBV: bool = attr.field(default=False)
    is_homed_RBV: bool = attr.field(default=False)
    is_position_reached_RBV: bool = attr.field(default=False)
    negative_limit_switch_status_RBV: bool = attr.field(default=False)
    positive_limit_switch_status_RBV: bool = attr.field(default=False)
    
    # internal states:    
    is_move_interrupted: bool = attr.field(default=False) # this flag is set when the motion is interrupted by a limit switch or a stop command. It is reset when the motion is restarted.
    update_interval_nonmoving: float = attr.field(default=5.0) # interval in seconds to update the axis parameters from the board. This is increased during a move to 0.1s. 
    update_interval_moving: float = attr.field(default=0.1) # interval in seconds to update the axis parameters from the board. This is increased during a move to 0.1s.
    # axis description
    axis_number: int = attr.field(default=0) # axis number on the board
    short_id: str = attr.field(default="Motor1") # short ID for the axis, should be alphanumeric
    description: str = attr.field(default="TMCM-6214 Axis") # description of the axis

    @property
    def direction(self) -> int:
        """Returns +/- 1 depending on whether the axis direction is positive (normal) or negative (inverted)"""
        if self.invert_axis_direction: return int(-1)
        else: return int(1)

    def velocity_in_microsteps_per_second(self, velocity:ureg.Quantity=None, as_quantity:bool=False) -> Union[int, ureg.Quantity]:
        """
        Convert velocity to microsteps per second.
        parameters:
        velocity: velocity to convert. If None, the default value from the axis parameters is used.

        """
        if velocity is None:
            velocity = self.velocity
        else: 
            velocity = quantity_converter(velocity, target_unit = self.base_realworld_unit/ureg.s)
        if not velocity.dimensionality == (self.base_realworld_unit/ureg.s).dimensionality:
            logging.warning(f"incompatible units {velocity.units} in velocity_in_microsteps_per_second")
        if not as_quantity:
            return int((velocity * self.steps_to_realworld_conversion_quantity).to('steps/s').magnitude) # steps per second
        else:
            return (velocity * self.steps_to_realworld_conversion_quantity).to('steps/s') # steps per second
    
    def acceleration_in_microsteps_per_second_squared(self, acceleration_duration:ureg.Quantity=None) -> int:
        """
        Convert acceleration duration to microsteps per second squared.
        parameters:
        acceleration_duration: duration of the acceleration phase. If None, the default value from the axis parameters is used.
        """
        if acceleration_duration is None:
            acceleration_duration = self.acceleration_duration
        if not acceleration_duration.dimensionality == (ureg.s).dimensionality:
            logging.warning(f"incompatible units {acceleration_duration.units} in acceleration_duration_in_microsteps_per_second_squared")
        return int((self.velocity_in_microsteps_per_second(as_quantity=True) / self.acceleration_duration).to('steps/s**2').magnitude)

    def user_to_raw(self, userCoordinate:ureg.Quantity) -> int:
        """ 
        Returns raw steps for provided user coordinates by sequencing user_to_dial and dial_to_raw. 
        uses the EPICS definition: (fixes https://github.com/BAMresearch/Trinamic_TMCM6214_TMCL_IOC/issues/2)
        userVAL = DialVAL * DIRection + OFFset
        """
        return self.dial_to_raw(self.user_to_dial(userCoordinate))

    def raw_to_user(self, rawCoordinate:Union[int, ureg.Quantity]) -> ureg.Quantity:
        """
        returns the user coordinate matching a raw position in steps. by sequencing raw_to_dial and dial_to_user 
        uses the EPICS definition: (fixes https://github.com/BAMresearch/Trinamic_TMCM6214_TMCL_IOC/issues/2)
        userVAL = DialVAL * DIRection + OFFset
        """
        return self.dial_to_user(self.raw_to_dial(rawCoordinate))

    def user_to_dial(self, userCoordinate:ureg.Quantity) -> ureg.Quantity:
        """ 
        Returns dial coordinates for provided user coordinates in the same unit as provided. 
        uses the EPICS definition: (fixes https://github.com/BAMresearch/Trinamic_TMCM6214_TMCL_IOC/issues/2)
        userVAL = DialVAL * DIRection + OFFset
        """
        return ((userCoordinate - self.user_offset) / self.direction ).to(userCoordinate.units)
    
    def dial_to_raw(self, dialCoordinate:ureg.Quantity) -> ureg.Quantity:
        """
        returns raw steps for a given dial coordinate. uses the motor resolution to translate
        """
        return self.real_world_to_steps(dialCoordinate)

    def raw_to_dial(self, rawCoordinate:Union[int, ureg.Quantity]) -> ureg.Quantity:
        """
        returns the dial coordinate matching a raw position in steps. 
        """
        if isinstance(rawCoordinate, ureg.Quantity):
            assert rawCoordinate.is_compatible_with('steps'), 'quantity provided to raw_to_dial must have unit of steps'
            rawCoordinate=rawCoordinate.magnitude
        return self.steps_to_real_world(rawCoordinate)
    
    def dial_to_user(self, dialCoordinate:ureg.Quantity) -> ureg.Quantity:
        """
        returns the user coordinates for given dial coordinates, in the same unit as provided. 
        uses the EPICS definition: (fixes https://github.com/BAMresearch/Trinamic_TMCM6214_TMCL_IOC/issues/2)
        userVAL = DialVAL * DIRection + OFFset
        """
        return (dialCoordinate * self.direction + self.user_offset).to(dialCoordinate.units)

    def steps_to_real_world(self, steps: int) -> ureg.Quantity:
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

    def real_world_to_steps(self, distance_or_angle: ureg.Quantity) -> int:
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
    
