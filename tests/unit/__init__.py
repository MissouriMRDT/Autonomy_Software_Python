import core
import logging
import logging.config
import yaml
from yaml import CLoader

# set up dependencies
core.rovecomm = core.RoveComm(11000, ('127.0.0.1', 11111))

yaml_conf = yaml.load(open('core/logging.yaml', 'r').read(), Loader=CLoader)
logging.config.dictConfig(yaml_conf)
