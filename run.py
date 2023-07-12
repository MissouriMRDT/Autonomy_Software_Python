#
# Mars Rover Design Team
# run.py
#
# Created on Sep 22, 2020
# Updated on Aug 21, 2022
#

import argparse
import asyncio
import importlib
import logging
import os
import sys
import time

import yaml
import rich

import core
import interfaces

from logging import config
from rich import logging


def setup_logger(level) -> logging.Logger:
    """
    Sets up the logger used for the autonomy codebase with
    appropriate handlers and formatting

    :param level: logging level to be used
    :return: logging.Logger
    """

    yaml_conf = yaml.safe_load(open("resources/logging/logging.yaml", "r").read())
    logging.config.dictConfig(yaml_conf)

    for handler in logging.getLogger().handlers:
        if isinstance(handler, type(rich.logging.RichHandler())):
            handler.setLevel(level)

    return logging.getLogger()


def main() -> None:
    """
    Sets up autonomy by parsing arguments and getting core
    systems enabled

    :return: None
    """

    # Parse arguments for autonomy
    parser = argparse.ArgumentParser()

    # Optional: Maps the file name to a known module if found
    parser.add_argument(
        "--file",
        help="Specify the name of the custom module to be run",
        default="autonomy.py"
    )

    # Optional: Sets the logging level for autonomy
    parser.add_argument(
        "--level",
        help="Specify the logging level to be used",
        choices=["DEBUG", "INFO", "WARN", "CRITICAL", "ERROR"],
        default="INFO"
    )

    # Optional: Sets the vision system to be used
    parser.add_argument(
        "--vision",
        help="Specify the vision system for autonomy",
        choices=["ZED", "SIM"],
        default="ZED"
    )

    # Optional: Sets whether we are streaming or not
    parser.add_argument(
        "--stream",
        help="Specify if we are streaming",
        choices=["Y", "N"],
        default="N"
    )

    # Optional: Sets the mode of operation
    parser.add_argument(
        "--mode",
        help="Sets if we are running on rover or on sim",
        choices=["REGULAR", "SIM"],
        default="REGULAR"
    )

    args = parser.parse_args()
    if (level := getattr(logging, args.level, -1)) < 0:
        parser.print_help()
        exit(1)

    # SIM mode defaults vision subsystem to also originate from simulator
    if args.mode == "SIM":
        args.vision = "SIM"

    # Add the examples' folder to our path, so we can run example files
    sys.path.insert(0, "example/")

    # Add the unit test folder to our path, so we can run tests
    sys.path.insert(0, "tests/unit/")

    # Enable the logger, also pass-in optional logging level for console output
    logger = setup_logger(level)

    # Initialize the rovecomm node
    core.rovecomm_node = core.RoveComm(11000, ("127.0.0.1", 11111))

    # Initialize the core handlers (excluding vision)
    core.setup(args.mode)

    # Initialize the core vision components
    core.vision.setup(args.vision, args.stream)

    # Initialize the Interfaces
    interfaces.setup()

    # Sleep so everything can be set up
    time.sleep(2)

    try:
        # Remove .py and directly import module
        module = importlib.import_module(os.path.splitext(args.file)[0])
        module.main()
    except ImportError as error:
        # Couldn't find module because file doesn't exist or tried to import
        # from package
        logger.error(f"Failed to import module '{args.file}'")
        logger.exception(error)
        core.rovecomm_node.close_thread()
        core.vision.close(args.vision)
        exit(1)
    except AttributeError as error:
        # Successful import but module does not define main
        logger.error(f"{args.file}: Undefined reference to main")
        logger.exception(error)
        core.rovecomm_node.close_thread()
        core.vision.close(args.vision)
        exit(1)
    except KeyboardInterrupt:
        core.rovecomm_node.close_thread()
        core.vision.close(args.vision)
        loop = asyncio.get_event_loop()
        loop.close()
        exit(0)


if __name__ == "__main__":
    # Run main()
    main()
