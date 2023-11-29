import pint
from caproto.server import PVGroup, pvproperty
from src.axis_parameters import AxisParameters
from typing import Any

ureg = pint.UnitRegistry()  # Ensure you have a unit registry instance

def create_pv_for_attribute(attr_name: str, attr_value: Any, axis_parameters: AxisParameters) -> pvproperty:
    """
    Create a pvproperty for a given attribute name and value.

    :param attr_name: Name of the attribute.
    :param attr_value: Value of the attribute.
    :param axis_parameters: AxisParameters instance containing the attribute.
    :return: A pvproperty corresponding to the attribute.
    """
    # For simple types, create a basic pvproperty
    @pvproperty(value=attr_value)
    async def pv(instance):
        # Getter
        return getattr(instance.parent.axis_parameters, attr_name)

    @pv.putter
    async def pv(instance, value):
        # Setter
        setattr(instance.parent.axis_parameters, attr_name, value)
    
    return pv

def create_pv_for_attribute(attr_name, attr_value, axis_parameters):
    """
    Create a pvproperty based on the attribute type.
    """
    # Implementation for creating a single PV or handling different types...
    # ...

def create_pvs_for_attribute(attr_name: str, attr_value: Any, axis_parameters: AxisParameters, pv_group: PVGroup):
    """
    Create one or more PVs for a given attribute based on its type.
    Handles pint.Quantity by creating separate PVs for value and unit.

    :param attr_name: Name of the attribute.
    :param attr_value: Value of the attribute.
    :param axis_parameters: AxisParameters instance containing the attribute.
    :param pv_group: The PVGroup to which the PVs will be added.
    """
    if isinstance(attr_value, pint.Quantity):
        create_quantity_pvs(attr_name, attr_value, axis_parameters, pv_group)
    else:
        pv = create_pv_for_attribute(attr_name, attr_value, axis_parameters)
        setattr(pv_group, attr_name, pv)

def create_quantity_pvs(attr_name: str, quantity: pint.Quantity, axis_parameters: AxisParameters, pv_group: PVGroup):
    """
    Create separate PVs for the magnitude and unit of a pint.Quantity.

    :param attr_name: Name of the attribute.
    :param quantity: The pint.Quantity instance.
    :param axis_parameters: AxisParameters instance containing the attribute.
    :param pv_group: The PVGroup to which the PVs will be added.
    """
    # Create PV for the magnitude (value)
    value_pv = create_pv_for_attribute(f"{attr_name}_value", quantity.magnitude, axis_parameters)
    setattr(pv_group, f"{attr_name}_value", value_pv)

    # Create PV for the unit
    unit_pv = create_pv_for_attribute(f"{attr_name}_unit", str(quantity.units), axis_parameters)
    setattr(pv_group, f"{attr_name}_unit", unit_pv)
