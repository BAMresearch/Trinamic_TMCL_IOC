# Trinamic TMCL IOC

## Overview

Trinamic TMCL IOC is a Python package designed for controlling stepper motors connected to a Trinamic board using the TMCL language (all boards supported by PyTrinamic should now work, has been tested on the TMCM 6110 and the TMCM 6214). Since it is implementing the TMCL protocol, it should be easy to adapt to other Trinamic motor controller boards. This package assumes the motor controller is connected over a machine network via a network-to-serial converter, but the underlying PyTrinamic package allows for other connections too. 

This allows the control of attached motors via the EPICS Channel-Access virtual communications bus. If EPICS is not desired, plain Pythonic control via motion_control should also be possible. An example for this will be provided in the example.ipynb Jupyter notebook.

This package leverages Caproto for EPICS IOCs and a modified PyTrinamic library for the motor board control, and interfaces between the two via an internal set of dataclasses. Configuration for the motors and boards are loaded from YAML files (see tests/testdata/example_config.yaml). 

The modifications to PyTrinamic involved extending the library with a socket interface. This was a minor modification that should eventually find its way into the official package (a pull request has been submitted). 

## Usage

*note that for any of this to work, the requirements for EPICS communication must be satisfied, in particular you must have the ability to communicate over a broadcast UDP ethernet connection*

The IOC can be started, loading the example configuration in the tests/testdata/example.yaml configuration file from the command line from the main package directory as follows: 
```
screen -- python . --configfile tests/testdata/example_config.yaml --list-pvs
```
In another terminal, the EPICS motor can be addressed and controlled by either polling values, e.g. to get the raw read-back position in steps:

```
caproto-get mc0:ysam.RRBV
```
and by setting fields, such as homing the axis, changing the max velocity, and moving to a position of 10 mm:
```
caproto-put mc0:ysam.HOMF 1
# wait until it's completed, then set to 5 mm/s speed
caproto-put mc0:ysam.VELO 5
caproto-put mc0:ysam 10.0
```
You can also set up continuous parameter monitoring in a terminal, for example through
```
caproto-monitor mc0:ysam.RRBV
```

## Features:

Besides changing motion parameters such as the user offset, limits and speeds, there are motion control methods available for homing and moving with (optional) backlash. 

The IOC also allows you to move to a different position while a previous motion is still underway. the full backlash will still be applied if required. Large and small motions can be made without issue. 

Units support is provided to make the library internally units-aware via the Pint package. That means that using mixed but compatible units should not be a problem. Linear and rotation motions should both be supported, units-wise.

Extensive use of the asyncio library internally enables generally good response times, even during actions. 

EPICS flags "STOP" and "SPMG" (Stop-Pause-Move-Go) should be respected by the package (not extensively tested yet), so that motors can be externally prevented or interrupted from moving. 

# Nomenclature:

User values are values presented to the user. That means these are corrected for offsets and converted to engineering units (EGU). 

Dial values are values related to the actual motor stage position. These are not corrected for offsets, but converted to engineering units. That means if you have a 75mm translation stage that is homed to its lower limit, its dial range goes from 0 to 75. 

Raw values are values in (micro-)steps reported to and from the motor controller.

Read-back values are values reported by the IOC, denominating the current state (or the most current read-out) of the device. They cannot be set.

More information can be found in the EPICS documentaiton for motor records. 

## EPICS compatibility

Caproto exposes quite a few of the EPICS motor fields by default. The following EPICS motor fields are actively updated by this package, mostly in line with the requirements of controlling EPICS motors via SPEC with a few extra's thrown in, and use EGU's (Engineering units) when possible:
  - ACCL - acceleration time (seconds)
  - BDST - backlash distance (EGU)
  - BVEL - Not fully implemented, just assumes VELO as RBV for now. 
  - BACC - Not fully implemented, just assumes ACCL as RBV for now. 
  - DHLM - dial high limit
  - DIFF - Dial difference target - actual
  - DIR  - axis direction, best set in the configuration file, not recommended to change during operation, although it should be possible. 
  - DLLM - Dial low limit
  - DMOV - Done moving to target
  - DRBV - Dial position read-back value (EGU)
  - DVAL - Dial desired value
  - EGU  - Engineering base units, e.g. 'mm' for millimeters. 
  - FOFF - Fix OFFset. In combination with SET can be used to adjust the motor steps value on the board to follow changes in e.g. VAL or DRBV.
  - HLS  - high-limit switch
  - IGSET - ignores the SET field... why is this even here. Do you want me to SET or not?
  - LLS  - low-limit switch
  - MOVN - motor is moving
  - MRES - Motor step size (EGU/step)
  - OFF  - user offset (EGU) of the user scale from the dial scale (offset of user zero from dial zero). 
  - RBV  - User position read-back value (EGU)
  - RDIF - Raw difference target - actual
  - RHLS - Raw high-limit switch status
  - RLLS - Raw low-limit switch status
  - RLV - Relative (user) value change
  - RRBV - Raw position read-back value (steps)
  - RVAL - Raw desired value
  - SET - Allows modification of the calibration values without physical movement
  - SPMG - Stop-pause-move-go, an externally set flag, normally set to "Go". Can be set to "Stop" to prevent motion
  - STOP - STOP flag. prevents motion if set True
  - VAL  - User desired value
  - VELO - Maximum positioning velocity (EGU/s)

Most of these have been tested. Others may be easily implementable; add them to update_epics_motorfields_instance for readable fields in epics_utils.py. Also consider adding bidirectional functionality for settable EPICS fields through update_axpar_from_epics_and_take_action in board_pv_group.py. 

## Development

This package is in the beta stage, and should be functional. Contributions and feedback are welcome.
Tasks to do (-), to revisit later (r) and done (v) include: 
  - write configuration to state file. this needs some extra work to make the output a bit more useful. 
  - fix IOC crash when board does not respond (temporarily) to communication - is a PyTrinamics RuntimeError... not sure how unless I encapsulate all communication calls in a try.. except statement
  - check that the board retains settings when powered off -> it doesn't. 
  - check for power cycle (done) and restore on fail (not yet done)

## License

This project is licensed under the GNU General Public License v3.0 - see the LICENSE file for details.

## Authors

Brian R. Pauw - design and implementation - GitHub username toqduj

## Acknowledgments

Thanks to all the amazing authors of open-source works whose code was unwillingly used for training GPT-4. I don't like it but it was needed for the emergency development and rollout of this communication package to fix my machine. 

Thanks to Julius Frontzek for starting the very early efforts on this. None of that code has been reused, but it did inspire and show it was possible via Caproto. 
