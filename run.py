import argparse, logging, core
# Import files that should be calleable from run.py here
import example

# Map all calleable file names to their respective mains here
FUNCTIONS = {'example.py': example.main}


def setupLogger() -> logging.Logger:
    '''
    Sets up the logger used in the autonomy project with appropriate
    handlers and formatting

    Returns
    -------

        Logger: Autonomy_Logger set up for console and file logging
    '''

    # logging file
    logger = logging.getLogger('Autonomy_Logger')
    logger.setLevel(logging.DEBUG)

    c_handler = logging.StreamHandler()
    f_handler = logging.FileHandler(filename='logging.csv', mode='w')

    # Create formatters and add it to handlers
    c_format = logging.Formatter('%(name)s, %(levelname)s, %(module)s, %(message)s')
    f_format = logging.Formatter('%(asctime)s, %(name)s, %(levelname)s, %(module)s, %(message)s')
    c_handler.setFormatter(c_format)
    f_handler.setFormatter(f_format)

    # Add handlers to the logger
    logger.addHandler(c_handler)
    logger.addHandler(f_handler)

    return logger


def main() -> None:

    logger = setupLogger()

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
            packet = core.RoveCommPacket(100, 'b', (1, 2), '')
            packet.SetIp('127.0.0.1')
            core.rovecomm_node.write(packet)
            func()
        except Exception as e:
            logger.exception("Tried to run: %s had exception %s", args.file, e)
    else:
        # Execute autonomy_main code here
        logger.info("Calling Autonomy main()")


if __name__ == "__main__":
    # Run main()
    main()
