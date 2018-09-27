from drivers.notify import Notify
from drivers.rovecomm import RoveComm

rovecomm_node = RoveComm()
object = Notify(rovecomm_node)

object.notifyFinish()
