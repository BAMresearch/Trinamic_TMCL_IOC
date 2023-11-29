# Trinamic TMCM-6214 TMCL IOC

## Overview

Trinamic TMCM-6214 TMCL IOC is a Python package designed for controlling stepper motors connected to a Trinamic TMCM-6214 board using the TMCL language. 

It utilizes Caproto for EPICS IOCs and PyTrinamic for direct motor control, providing a flexible and powerful interface for motor operations.

For this to work, we need to use the PyTrinamic code that I extended with a socket_tmcl communications class (for motor controller boards connected over a machine network via a network-to-serial converter). Other than that, it should behave like normal. 

## Usage

(Provide basic examples of how to use the package. For instance, initializing the control for a motor, moving a motor, etc.)

```python
from Trinamic_TMCM6214_TMCL_IOC import BoardControl, BoardParameters

# Load board parameters and initialize board control
board_params = BoardParameters()
# ... load parameters ...
board_control = BoardControl(board_params)

# Example of moving a motor
board_control.move_axis(axis_index=0, position_steps=10000)
```

## Development

This package is in the early stages of development. Contributions and feedback are welcome.

## License

This project is licensed under the GNU General Public License v3.0 - see the LICENSE file for details.

## Authors

Brian R. Pauw - Initial work - toqduj

## Acknowledgments

Thanks to all the amazing authors of open-source works whose code was unwillingly used for training GPT-4. I don't like it but it was needed for the emergency development and rollout of this communication package to fix my machine. 