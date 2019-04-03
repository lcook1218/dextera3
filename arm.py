from dof import MotorDOF, ServoDOF, GripperDOF
from kinematics import FK, IK


class Arm:
    q = []  # joint angle positions (length 3) VERTICAL, ROTATE, PAN
    o_curr = []  # x, y, z in space
    # prev_cmd = ''  # what is the point of this?

    GRIPPER_TIME = 5  # gripper wait time to open
    GRIPPER_POWER = -0.1  # Needs to be negative based on current motor config
    VERTICAL_POWER = 0.4

    VERTICAL_MOTOR = 0
    ROTATE_MOTOR = 1
    PAN_MOTOR = 2

    START_Q = [50, 0, 0]  # TODO: NOTE - 50 is just a placeholder
    OFF_Q = [0, -90, -90]  # TODO: SET TO VALUES WE WANT

    ROTATE_FACTOR = 1  # how much the servo rotates by

    VERTICAL_PWM = 18
    GRIPPER_1_PWM = 23
    # GRIPPER_2_PWM = X

    MOVE_WORD = 'up'
    ROTATE_WORD = 'in'
    PAN_WORD = 'up'

    gripper_closed = [True, True]  # gripper 1 and gripper 2 status (closed is true)

    def __init__(self):
        self.vertical = MotorDOF(17, 11, self.VERTICAL_PWM)
        self.rotate = ServoDOF(19)
        self.pan = ServoDOF(26)
        self.gripper_1 = GripperDOF(22, 10, self.GRIPPER_1_PWM)
        # self.gripper_2 = GripperDOF(x, x, self.GRIPPER_2_PWM)
        self.q = self.START_Q
        self.__full_set_position(self.q)
        self.o_curr = FK(self.q)
        return

    def __full_set_position(self, q):
        self.vertical.set_position(q[self.VERTICAL_MOTOR])
        self.rotate.set_position(q[self.ROTATE_MOTOR])
        self.pan.set_position(q[self.PAN_MOTOR])
        self.gripper_1.close(self.GRIPPER_POWER)
        # self.gripper_2.close(self.GRIPPER_POWER)
        return

    def __update_q(self, arr):
        self.q = arr
        return

# *** TODO: NEED TO ADD IN CATCHING ERRORS HERE, and dof.py ***

    def parse_text(self, command):
        if command is None:
            print('Sorry, I did not hear you')
        else:
            command = command.lower()
            command = self.__remove_symbols(command)
            if 'start' in command or 'stop' in command:
                self.__parse_motion_cmd(command)
            elif 'open' in command or 'close' in command:
                self.__parse_gripper_cmd(command)
            elif 'go to' in command:
                self.__parse_location_cmd(command)
            elif 'power' in command:
                self.__parse_power_cmd(command)
            else:  # finite movement "move up 10 inches"
                self.__parse_relative_cmd(command)
        return

    def __remove_symbols(self, command):
        return command.replace('°', ' degrees')  # NOTE: If deg sign not in, it does nothing

    def __parse_motion_cmd(self, command):
        if 'start' in command:
            if 'moving' in command:
                direction = 1 if self.MOVE_WORD in command else -1
                self.vertical.set_power(direction * self.VERTICAL_POWER)
                return
            if 'rotating' in command:
                direction = 1 if self.ROTATE_WORD in command else -1
                self.rotate.rotate_indefinitely(self.q[self.ROTATE_MOTOR], direction * self.ROTATE_FACTOR)
                return
            if 'panning' in command:
                direction = 1 if self.PAN_WORD in command else -1
                self.pan.rotate_indefinitely(self.q[self.PAN_MOTOR], direction * self.ROTATE_FACTOR)
                return
        else:  # stop
            # TODO: figure out stop position, update q, stop for other motors...
            self.vertical.stop()
            self.rotate.stop()
            self.pan.stop()
            self.__update_q([self.vertical.get_position(), self.rotate.get_position(), self.pan.get_position()])
        return

    def __parse_gripper_cmd(self, command):  # 'Open gripper x'
        g_num = int(command.split()[-1])
        gripper = self.gripper_1 if g_num == 1 else None  # self.gripper_2 TODO: add when self.gripper_2 is implemented
        if 'open' in command and self.gripper_closed[g_num - 1]:  # Array offset
            gripper.open(self.GRIPPER_POWER, self.GRIPPER_TIME)
            self.gripper_closed[g_num - 1] = False
            return
        if 'closed' in command and not self.gripper_closed[g_num - 1]:  # Array offset
            gripper.close(self.GRIPPER_POWER)
            self.gripper_closed[g_num - 1] = True
        return

    def __parse_location_cmd(self, command):  # "Go to x, y, z" TODO: see how STT formats to get parse correctly
        # degree to radian conversion handled in FK/IK
        x = int(command.split()[-3])
        y = int(command.split()[-2])
        z = int(command.split()[-1])
        o = [x, y, z]
        self.q = IK(o)
        self.o_curr = o
        return

    def __parse_relative_cmd(self, command):
        new_relative_pos = int(command.split()[-2])
        if 'move' in command:
            direction = 1 if self.MOVE_WORD in command else -1
            new_relative_pos *= direction
            if self.vertical.set_position(self.q[self.VERTICAL_MOTOR] + new_relative_pos):
                self.q[self.VERTICAL_MOTOR] += new_relative_pos
            return
        if 'rotate' in command:
            direction = 1 if self.ROTATE_WORD in command else -1
            new_relative_pos *= direction
            if self.rotate.set_position(self.q[self.ROTATE_MOTOR] + new_relative_pos):
                self.q[self.ROTATE_MOTOR] += new_relative_pos
            return
        if 'pan' in command:
            direction = 1 if self.PAN_WORD in command else -1
            new_relative_pos *= direction
            if self.pan.set_position(self.q[self.PAN_MOTOR] + new_relative_pos):
                self.q[self.PAN_MOTOR] += new_relative_pos
            return
        return

    def __parse_power_cmd(self, command):
        self.__update_q(self.START_Q) if 'on' in command else self.__update_q(self.OFF_Q)
        self.__full_set_position(self.q)
        return
