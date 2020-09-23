import argparse
import logging
import logging.config
import yaml
from yaml import CLoader
import core
# Import files that should be calleable from run.py here
import example

# Map all calleable file names to their respective mains here
FUNCTIONS = {'example.py': example.main}


def setup_logger() -> logging.Logger:
    '''
    Sets up the logger used in the autonomy project with appropriate
    handlers and formatting

    Returns
    -------

        Logger: Autonomy_Logger set up for console and file logging
    '''

    # logging file
    yaml_conf = yaml.load(open('logging_files/logging.yaml', 'r').read(), Loader=CLoader)
    logging.config.dictConfig(yaml_conf)
    logging.addLevelName(21, "NUMERICAL_INFO")

    return logging.getLogger('Autonomy_Logger')


def main() -> None:

    logger = setup_logger()

    # Initialize the rovecomm node
    core.rovecomm_node = core.RoveCommEthernetUdp()

    parser = argparse.ArgumentParser()

    # Optional parameter to run a custom main() from other module
    parser.add_argument('-c', '--custom', action="store_true", help="Specifies we want to run a custom main()")

    # Maps the passed in file name to a known module and main() (if it is known)
    parser.add_argument('file', choices=FUNCTIONS.keys(), nargs='?', help="Specify the name of the custom module to be run")

    args = parser.parse_args()

    # Call the mapped main as a function if we specify --execute
    if args.custom:
        func = FUNCTIONS[args.file]

        try:
            logger.info("Calling module: %s", args.file)
            func()
        except Exception as e:
            logger.exception("Tried to run: %s had exception %s", args.file, e)
    else:
        # Execute autonomy_main code here (not set up to be called yet)
        logger.info("Calling Autonomy main()")


if __name__ == "__main__":
    # Run main()
    main()
