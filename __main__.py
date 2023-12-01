#!/usr/bin/env python3

# this started out as a straight copy of caproto's fake_motor_record and should be gradually modified to fit our needs.
# can be run from the command line using: 
# python /usr/bpauw/Code/Trinamic_TMCM6214_TMCL_IOC --list-pvs -v 
# once I have the main code in place.

from pathlib import Path
from textwrap import dedent

from caproto.server import PVGroup, SubGroup, ioc_arg_parser, pvproperty, run, template_arg_parser
# from caproto.server.records import MotorFields
# from src.axis_parameters import AxisParameters
# from src.board_parameters import BoardParameters
from src.__init__ import ureg
from src.board_pv_group import TrinamicIOC

# This should contain the script that you want to run when the package is executed

def validate_file(arg):
    path = Path(arg)
    if not path.is_file():
        raise FileNotFoundError(f"File {path} does not exist.")
    return path

if __name__ == '__main__':
    parser, split_args = template_arg_parser(
    # ioc_options, run_options = ioc_arg_parser(
        default_prefix='mc0:', # motor controller at address zero. 
        desc=dedent(TrinamicIOC.__doc__),
        supported_async_libs=('asyncio',)
        )
    parser.add_argument('--configfile', help='Path to the board- and axes YAML configuration file', required=True, type=validate_file)
    
    args = parser.parse_args()
    ioc_options, run_options = split_args(args)

    ioc = TrinamicIOC(config_file=args.configfile, **ioc_options)
    run(ioc.pvdb, **run_options)
