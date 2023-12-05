import asyncio
import logging
from caproto.server.records import MotorFields, pvproperty
from . import ureg
from src.axis_parameters import AxisParameters
from src.board_parameters import BoardParameters    

async def update_epics_motorfields_instance(axpar: AxisParameters, instance:MotorFields)
    """Updates the motor record fields in the EPICS IOC with the values from the AxisParameters instance."""
    fields: MotorFields = instance.field_inst
    await fields.user_low_limit.write(axpar.negative_user_limit.to(axpar.base_realworld_unit).magnitude) 
    await fields.user_high_limit.write(axpar.positive_user_limit.to(axpar.base_realworld_unit).magnitude)
