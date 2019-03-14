from dof import DOF, MotorDOF, ServoDOF
from word2number import w2n
from kinematics import FK, IK
import math
import time

class Arm:

    q = []
    o_curr = []
    prev_cmd = ''
    POWER_GRIPPER = -0.1 #Needs to be negative based on current motor config
    POWER_VERTICAL = 0.4
    GRIPPER_TIME = 5

    VERTICAL_MOTOR = 0
    ROTATE_MOTOR = 1
    PAN_MOTOR = 2

    MOVE_WORD = 'up'
    ROTATE_WORD = 'in'
    PAN_WORD = 'up'

    gripper_closed = [True, True] #gripper 1 and gripper 2 status (closed is true)

    def __init__(self):
        self.vertical = MotorDOF(17, 11)
        self.wrist_rotate = ServoDOF(19)
        self.wrist_pan = ServoDOF(26)
        self.gripper_A = MotorDOF(22, 10)
        self.q = [0, 0, 90, 0, 0]
        self.o_curr = FK(self.q)
        #self.gripper_B = MotorDOF()


    #FORMS OF COMMAND:
    # MOVE UP/DOWN ... STOP
    # ROTATE TOWARDS/AWAY FROM ME ___ DEGREES
    # PAN LEFT/RIGHT ___ DEGREES
    # GO TO (X,Y,Z)
    # OPEN GRIPPER ___

# *** NEED TO ADD IN CATCHING ERRORS HERE ***

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
                direction = 1 if self.MOVE_WORD in command else direction = -1
                self.vertical.setPower(direction * self.POWER_VERTICAL)
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
                else: # down
                    # TODO: figure out servo movement
                    print('not yet implemented')
                return
        else: # stop
            # TODO: figure out stop position, update
            print('not yet implemented')
        return

    def parse_gripper_cmd(self, command):
        g_num = command.split()[-1] - 1  # Adjust for array offset
        if 'open' in command and self.gripper_closed[g_num]:
            self.open_gripper(g_num)
            return
        if 'closed' in command and not self.gripper_closed[g_num]:
            self.close_gripper(g_num)
        return

    def open_gripper(self, g_num):
        self.gripper_A.setPower(self.POWER_GRIPPER) if g_num == 0 else self.gripper_B.setPower(self.POWER_GRIPPER)
        time.sleep(self.GRIPPER_TIME)
        self.gripper_A.setPower(0) if g_num == 0 else self.gripper_B.setPower(0)
        self.gripper_closed[g_num] = False
        return

    def close_gripper(self, g_num):
        self.gripper_A.setPower(-1 * self.POWER_GRIPPER) if g_num == 0 else self.gripper_B.setPower(-1 * self.POWER_GRIPPER)
        return

    def parse_location_cmd(self, command):
        # TODO: implement parse_location
        return

    def parse_relative_cmd(self, command):
        new_relative_pos = command.split()[-2] # value in mm or degrees to move
        if 'move' in command:
            if self.MOVE_WORD in command:
                # TODO: figure out with encoder
                print('not yet implemented')
            else: # down
                # TODO: figure out with encoder
                print('not yet implemented')
            return

        new_relative_pos = math.radians(new_relative_pos)  # Other 2 need in radians
        if 'rotate' in command:
            direction = 1 if self.ROTATE_WORD in command else direction = -1
            new_relative_pos *= direction
            if self.wrist_rotate.set_position(self.q[self.ROTATE_MOTOR] + new_relative_pos):
                self.q[self.ROTATE_MOTOR] += new_relative_pos
            return
        if 'pan' in command:
            direction = 1 if self.PAN_WORD in command else direction = -1
            new_relative_pos *= direction
            if self.wrist_pan.set_position(self.q[self.PAN_MOTOR] + new_relative_pos):
                self.q[self.PAN_MOTOR] += new_relative_pos
            return
        return










    # takes in speech command as string, returns nothing
    def parse_text1(self, command): #DEPRICATE
        #adapt for multi-word numbers
        if command is None:
            print('Sorry, I did not hear you.')
        else:
            command = command.lower() # make lowercase
            cmd_arr = command.split() # create array of strings
            if 'move up' in command: # move up/down -> joint 1 motion
                self.vertical.setPower(self.POWER_VERTICAL)
                self.q[0] = self.q[0] + 20 #placeholder... should update after the stop command
            elif 'move down' in command:
                self.vertical.setPower(self.POWER_VERTICAL * -1)
                self.q[0] = self.q[0] - 20 #placeholder
                o_curr = FK(self.q)
            elif 'rotate' in command: # rotate twds/away -> joint 2 motion
                new_pos = w2n.word_to_num(cmd_arr[-2])
                if 'towards me' in command:
                    self.q[1] = self.q[1] + new_pos
                elif 'away from me' in command:
                    self.q[1] = self.q[1] - new_pos
                self.wrist_rotate.set_position(self.q[1])
                o_curr = FK(self.q)
            elif 'pan' in command: # pan left/right -> joint 3 motion
                new_pos = w2n.word_to_num(cmd_arr[-2])
                if 'left' in command:
                    self.q[2] = self.q[2] + new_pos
                elif 'right' in command:
                    self.q[2] = self.q[2] - new_pos
                self.wrist_pan.set_position(self.q[1])
                o_curr = FK(self.q)
            elif 'go to' in command: # exact position -> use IK to get q
                x = w2n.word_to_num(cmd_arr[-3])
                y = w2n.word_to_num(cmd_arr[-2])
                z = w2n.word_to_num(cmd_arr[-1])
                o = [x, y, z]
                self.q = IK(o)
                self.o_curr = o
            elif 'open gripper' in command:
                if (cmd_arr[-1] == 'one'):
                    self.gripper_A.setPower(self.POWER_GRIPPER)
                else:
                    self.gripper_B.setPower(self.POWER_GRIPPER)
            elif 'close gripper' in command:
                if (cmd_arr[-1] == 'one'):
                    self.gripper_A.setPower(self.POWER_GRIPPER * -1)
                else:
                    self.gripper_B.setPower(self.POWER_GRIPPER * -1)
            elif 'stop' in command:
                if (self.prev_cmd != None and 'gripper' in self.prev_cmd):
                    self.gripper_A.stop()
                    #self.gripper_B.stop()
                else:
                    self.vertical.stop()

            self.prev_cmd = command
            print(self.prev_cmd)
            self.set_dofs(self.q)
            print(self.q)
