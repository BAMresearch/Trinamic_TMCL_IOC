from caproto.server import PVGroup, ioc_arg_parser, run
from src.axis_parameters import AxisParameters
from src.create_pv_for_attribute import create_pvs_for_attribute

class MotorAxisPVGroup(PVGroup):
    def __init__(self, *args, axis_parameters: AxisParameters, **kwargs):
        super().__init__(*args, **kwargs)
        self.axis_parameters = axis_parameters

        # Dynamically create PVs for each attribute listed in pv_attributes
        for attr_name in axis_parameters.pv_attributes:
            attr_value = getattr(axis_parameters, attr_name)
            create_pvs_for_attribute(attr_name, attr_value, axis_parameters, self)

# Example usage
if __name__ == '__main__':
    axis_params = AxisParameters()  # Initialize with default or configured values
    ioc_options, run_options = ioc_arg_parser(
        default_prefix='motor_axis:',
        desc='Run an IOC for a motor axis.')
    ioc = MotorAxisPVGroup(axis_parameters=axis_params, **ioc_options)
    run(ioc.pvdb, **run_options)