import argparse, logging
import logging.config
import importlib
import core
import os


def setup_logger(level) -> logging.Logger:
    '''
    Sets up the logger used in the autonomy project with appropriate
    handlers and formatting

    Returns
    -------

        Logger: root set up for console and file logging
    '''

    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)
    
    # Primary handlers. Levels can be set with --level option
    c_handler = logging.StreamHandler()
    f_handler = logging.FileHandler(filename='logs/logging.csv', mode='w')
    c_handler.setLevel(level)
    f_handler.setLevel(level)

    # Debug level: print out file:line for IDE's that support file navigation.
    d_handler = logging.StreamHandler()
    d_handler.setLevel(logging.DEBUG)
    # Only prints out DEBUG level messages
    d_handler.addFilter(lambda log_record: log_record.levelno <= logging.DEBUG)


    # Create formatters and add it to handlers
    c_format = logging.Formatter('%(name)s, %(levelname)s, %(module)s, %(message)s')
    f_format = logging.Formatter('%(asctime)s, %(name)s, %(levelname)s, %(module)s, %(message)s')
    d_format = logging.Formatter("%(name)s, %(levelname)s, %(module)s, %(message)s %(pathname)s:%(lineno)d")
    c_handler.setFormatter(c_format)
    f_handler.setFormatter(f_format)
    d_handler.setFormatter(d_format)

    # Add handlers to the logger
    logger.addHandler(c_handler)
    logger.addHandler(f_handler)
    logger.addHandler(d_handler)
    return logger




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
    core.rovecomm_node = core.RoveCommEthernetUdp()

    try:
        # Remove .py and directly import module
        module = importlib.import_module(os.path.splitext(args.file)[0])
        module.main()
    except ImportError:
        # Couldn't find module because file doesn't exist or tried to import
        # from package
        logger.error(f"Failed to import module '{args.file}'")
        exit(1)
    except NameError:
        # Successful import but module does not define main
        logger.error(f"{args.file}: Undefined reference to main")
        exit(1)
    else:
        exit(0)
    

if __name__ == "__main__":
    # Run main()
    main()
