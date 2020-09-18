
from algorithms.PID_controller import PIDcontroller


from core.rovecomm import RoveCommEthernetUdp
from interfaces.drive_board import DriveBoard
from interfaces.nav_board import NavBoard


def clamp(n, min_n, max_n):
    return max(min(max_n, n), min_n)


pid = PIDcontroller(Kp=3, Ki=0.25, Kd=0, wraparound=360)


def get_motor_power_from_heading(speed, goal_heading, drive_board, nav_board):

    heading_correction = pid.update(goal_heading, nav_board.heading())
    clamp(heading_correction, -180, 180)
    return drive_board.calculate_move(speed, heading_correction)


if __name__ == "__main__":

    rovecomm_node = RoveCommEthernetUdp("")
    drive = DriveBoard("")
    drive.enable()
    nav_board = NavBoard(rovecomm_node, "")

    while True:
        print("Heading: " + str(nav_board.heading()))
        left, right = get_motor_power_from_heading(100, 90, drive, nav_board)
        print("Drive: " + str(left) + ", " + str(right))
        packet = drive.send_drive(left, right)
        rovecomm_node.write(packet)

