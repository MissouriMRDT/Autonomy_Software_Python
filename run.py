import argparse
import logging
import logging.config
import yaml
import core
import interfaces
import importlib
import os
import sys
import time
import cv2


def setup_logger(level) -> logging.Logger:
    """
    Sets up the logger used in the autonomy project with appropriate
    handlers and formatting

    Returns
    -------

        Logger: root set up for console and file logging
    """

    # logging file
    yaml_conf = yaml.safe_load(open("core/logging.yaml", "r").read())
    logging.config.dictConfig(yaml_conf)

    for handler in logging.getLogger().handlers:
        if isinstance(handler, type(logging.StreamHandler())):
            handler.setLevel(level)

    return logging.getLogger()


def main() -> None:
    parser = argparse.ArgumentParser()

    # Maps the passed in file name to a known module and main() (if it is known)
    parser.add_argument("--file", help="Specify the name of the custom module to be run", default="autonomy.py")

    # Optional parameter to set logging level
    parser.add_argument("--level", choices=["DEBUG", "INFO", "WARN", "CRITICAL", "ERROR"], default="INFO")

    # Optional parameter to set the vision system to use
    parser.add_argument("--vision", choices=["ZED", "NONE", "SIM", "WEBCAM"], default="ZED")

    # Optional parameter to set the mode of operation:
    # Regular (on rover) or Sim (using the autonomy simulator)
    parser.add_argument("--mode", choices=["REGULAR", "SIM"], default="REGULAR")

    args = parser.parse_args()
    if (level := getattr(logging, args.level, -1)) < 0:
        parser.print_help()
        exit(1)

    # Sim mode defaults vision subsystem to also originate from simulator
    if args.mode == "SIM":
        args.vision = "SIM"

    # Add the examples folder to our path so we can run example files
    sys.path.insert(0, "example/")

    # Add the unit test folder to our path so we can run tests
    sys.path.insert(0, "tests/unit/")

    # Setup the logger, also pass-in optional logging level for console output
    logger = setup_logger(level)

    # Initialize the rovecomm node
    core.rovecomm_node = core.RoveComm(11000, ("127.0.0.1", 11111))

    # Initialize the core handlers (excluding vision)
    core.setup(args.mode)

    # Initialize the core vision components
    core.vision.setup(args.vision)

    # Initialize the Interfaces
    interfaces.setup()

    # Sleep so everything can be set up
    time.sleep(0.1)

    try:
        # Remove .py and directly import module
        module = importlib.import_module(os.path.splitext(args.file)[0])
        module.main()
    except ImportError as error:
        # Couldn't find module because file doesn't exist or tried to import
        # from package
        logger.error(f"Failed to import module '{args.file}'")
        logger.error(error)
        core.rovecomm_node.close_thread()
        core.vision.close(args.vision)
        exit(1)
    except NameError as error:
        # Successful import but module does not define main
        logger.error(f"{args.file}: Undefined reference to main")
        logger.error(error)
        core.rovecomm_node.close_thread()
        core.vision.close(args.vision)
        exit(1)
    except KeyboardInterrupt as error:
        core.rovecomm_node.close_thread()
        core.vision.close(args.vision)
        cv2.destroyAllWindows()
        exit(0)
    else:
        core.rovecomm_node.close_thread()
        core.vision.close(args.vision)
        cv2.destroyAllWindows()
        exit(0)


if __name__ == "__main__":
    # Run main()
    main()
