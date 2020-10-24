import argparse
import logging
import logging.config
import yaml
from yaml import CLoader
import core
import importlib
import os


def setup_logger(level) -> logging.Logger:
    '''
    Sets up the logger used in the autonomy project with appropriate
    handlers and formatting

    Returns
    -------

        Logger: root set up for console and file logging
    '''

    # logging file
    yaml_conf = yaml.load(open('core/logging.yaml', 'r').read(), Loader=CLoader)

    # Set log level of root logger to command line argument
    yaml_conf['handlers']['console']['level'] = level

    logging.config.dictConfig(yaml_conf)

    return logging.getLogger(__name__)


def main() -> None:
    parser = argparse.ArgumentParser()

    # Maps the passed in file name to a known module and main() (if it is known)
    parser.add_argument('--file', help="Specify the name of the custom module to be run", default="autonomy.py")

    # Optional parameter to set logging level
    parser.add_argument('--level', choices=["DEBUG", "INFO", "WARN", "CRITICAL", "ERROR"], default="INFO")

    args = parser.parse_args()
    if (level := getattr(logging, args.level, -1)) < 0:
        parser.print_help()
        exit(1)

    logger = setup_logger(level)

    # Initialize the rovecomm node
    core.rovecomm = core.RoveComm(11000, ('127.0.0.1', 11111))

    try:
        # Remove .py and directly import module
        module = importlib.import_module(os.path.splitext(args.file)[0])
        module.main()
    except ImportError as error:
        # Couldn't find module because file doesn't exist or tried to import
        # from package
        logger.error(f"Failed to import module '{args.file}'")
        logger.error(error)
        core.rovecomm.close_thread()
        exit(1)
    except NameError as error:
        # Successful import but module does not define main
        logger.error(f"{args.file}: Undefined reference to main")
        logger.error(error)
        core.rovecomm.close_thread()
        exit(1)
    else:
        core.rovecomm.close_thread()
        exit(0)


if __name__ == "__main__":
    # Run main()
    main()
