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
Tasks to do include: 
  v make a method that updates the axis parameters from the EPICS fields (in case we're adjusting the axis parameters via EPICS). 
  v make a method that allows us to update the offset value. This should perhaps adjust the user limits too...
  v make a convenience method that allows us to set the current position to zero user value
  v make a method that starts a home action when HOMF or HOMR is set to 1. 
  v enable and test axis inversion (parameter 251 on Trinamics boards) (enabled, not testd yet)
  v tie the above together with a neat bow. 
  v assure the axis does not move if we try a move beyond limits. 
  v check the limit switch setting: is the "right" limit switch the negative or positive limit?
  v assure we're doing a new backlash move if we move to a new position during a previous backlash move. 
  - write configuration to state file. 
  - fix IOC crash when board does not respond (temporarily) to communication - is a PyTrinamics RuntimeError... not sure how. 
  - check that the board retains settings when powered off -> it doesn't. 
  - At the moment it's not doing an EPICS-to-axis_parameter sync. Not sure if needed.
  - check for power cycle (done) and restore on fail (not yet done)

## License

This project is licensed under the GNU General Public License v3.0 - see the LICENSE file for details.

## Authors

Brian R. Pauw - Initial work - toqduj

## Acknowledgments

Thanks to all the amazing authors of open-source works whose code was unwillingly used for training GPT-4. I don't like it but it was needed for the emergency development and rollout of this communication package to fix my machine. 