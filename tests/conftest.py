import pytest
import core
import interfaces
import logging
import logging.config
import yaml


def pytest_sessionstart(session) -> None:
    # Configure RoveComm so we can use it in our tests (when applicable)
    core.rovecomm_node = core.RoveComm(11000, ("127.0.0.1", 11111))

    # Setup the core components, mainly manifest
    core.setup()

    # Setup our interfaces
    interfaces.setup()

    yaml_conf = yaml.safe_load(open("core/logging.yaml", "r").read())
    logging.config.dictConfig(yaml_conf)


def pytest_sessionfinish(session, exitstatus) -> None:
    # Properly clean up RoveComm sockets when the test session ends
    core.rovecomm_node.close_thread()
