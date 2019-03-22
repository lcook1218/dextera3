import math
# from word2number import w2n

from dof import MotorDOF, ServoDOF, GripperDOF
from kinematics import FK, IK


class Arm:

    q = []  # joint angle positions
    o_curr = []  # x, y, z in space
    prev_cmd = ''

    GRIPPER_TIME = 5  # gripper wait time to open
    GRIPPER_POWER = -0.1  # Needs to be negative based on current motor config
    VERTICAL_POWER = 0.4

    VERTICAL_MOTOR = 0
    ROTATE_MOTOR = 1
    PAN_MOTOR = 2

    MOVE_WORD = 'up'
    ROTATE_WORD = 'in'
    PAN_WORD = 'up'

    gripper_closed = [True, True]  # gripper 1 and gripper 2 status (closed is true)

    def __init__(self):
        self.vertical = MotorDOF(17, 11)
        self.wrist_rotate = ServoDOF(19)
        self.wrist_pan = ServoDOF(26)
        self.gripper_1 = GripperDOF(22, 10)
        # self.gripper_2 = GripperDOF(x, x)
        self.q = [0, 1500, 1500]  # TODO: CONVERT EVERYTHING TO THIS CRAZY NUMBERS SHIT
        self.full_set_position(self.q)
        self.o_curr = FK(self.q)
        return

    def full_set_position(self, q):
        self.vertical.set_position(q[self.VERTICAL_MOTOR])
        self.wrist_rotate.set_position(q[self.ROTATE_MOTOR])
        self.wrist_pan.set_position(q[self.PAN_MOTOR])
        self.gripper_1.close(self.GRIPPER_POWER)
        # self.gripper_2.close(self.GRIPPER_POWER)
        return

# *** # TODO: NEED TO ADD IN CATCHING ERRORS HERE, and dof.py ***

    def parse_text(self, command):
        if command is None:
            print('Sorry, I did not hear you')
        else:
            command = command.lower()
            if 'start' in command or 'stop' in command:
                self.parse_motion_cmd(command)
            elif 'open' in command or 'close' in command:
                self.parse_gripper_cmd(command)
            elif 'go to' in command:
                self.parse_location_cmd(command)
            else:
                self.parse_relative_cmd(command)
        return

    def parse_motion_cmd(self, command):
        if 'start' in command:
            if 'moving' in command:
                direction = 1 if self.MOVE_WORD in command else -1
                self.vertical.set_power(direction * self.VERTICAL_POWER)
                return
            if 'rotating' in command:
                if self.ROTATE_WORD in command:
                    # TODO: figure out servo movement
                    print('not yet implemented')
                else:  # out
                    # TODO: figure out servo movement
                    print('not yet implemented')
                return
            if 'panning' in command:
                if self.PAN_WORD in command:
                    # TODO: figure out servo movement
                    print('not yet implemented')
                else:  # down
                    # TODO: figure out servo movement
                    print('not yet implemented')
                return
        else:  # stop
            # TODO: figure out stop position, update q, stop for other motors...
            self.vertical.stop()
        return

    def parse_gripper_cmd(self, command):  # 'Open gripper x'
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

    def parse_location_cmd(self, command):  # "Go to x, y, z" TODO: see how STT formats to get parse correctly
        x = command.split()[-3]
        y = command.split()[-2]
        z = command.split()[-1]
        o = [int(x), int(y), int(z)]
        self.q = IK(o)
        self.o_curr = o
        return

    def parse_relative_cmd(self, command):
        new_relative_pos = int(command.split()[-2])  # value in mm or degrees to move
        if 'move' in command:
            if self.MOVE_WORD in command:
                # TODO: figure out with encoder
                print('not yet implemented')
            else:  # down
                # TODO: figure out with encoder
                print('not yet implemented')
            return

        # new_relative_pos = math.radians(new_relative_pos)  # TODO: FIX THIS JUST FOR FK/IK... STORE Q IN DEGREES
        if 'rotate' in command:
            direction = 1 if self.ROTATE_WORD in command else -1
            new_relative_pos *= direction
            print(self.q[self.ROTATE_MOTOR] + new_relative_pos)
            if self.wrist_rotate.set_position(self.q[self.ROTATE_MOTOR] + new_relative_pos):
                self.q[self.ROTATE_MOTOR] += new_relative_pos
            print(self.q)
            return
        if 'pan' in command:
            direction = 1 if self.PAN_WORD in command else -1
            new_relative_pos *= direction
            if self.wrist_pan.set_position(self.q[self.PAN_MOTOR] + new_relative_pos):
                self.q[self.PAN_MOTOR] += new_relative_pos
            return
        return
