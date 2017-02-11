import drivers.rovecomm

rove_comm = drivers.rovecomm.RoveComm()

def MsgHandler(packet_contents):
    print packet_contents

rove_comm.callbacks[2576] = MsgHandler
rove_comm.callbacks[2577] = MsgHandler
rove_comm.callbacks[2578] = MsgHandler
rove_comm.callbacks[2579] = MsgHandler
rove_comm.callbacks[2580] = MsgHandler

raw_input("hit enter to continue")
