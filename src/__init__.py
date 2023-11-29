# This can be empty or include specific imports for convenience
# Example: Import key classes for easy access

import pint
ureg = pint.UnitRegistry(auto_reduce_dimensions=True)
ureg.define('step = 1 * count = steps')
# Initialize logging
import logging
logging.basicConfig(level=logging.INFO)
