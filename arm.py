from dof import DOF, MotorDOF, ServoDOF
from word2number import w2n
from kinematics import FK, IK
import math
#install word to num package

class Arm:

    q = []
    o_curr = []
    prev_cmd = ''
    POWER_GRIPPER = -0.1 #Needs to be negative based on current motor config
    POWER_VERTICAL = 0.4

    def __init__(self):
        self.vertical = MotorDOF(17, 11)
        self.wrist_rotate = ServoDOF(19)
        self.wrist_tilt = ServoDOF(26)
        self.gripper_A = MotorDOF(22, 10)
        self.q = [0, 0, 90, 0, 0]
        self.o_curr = FK(self.q)
        #self.gripper_B = MotorDOF()

    # takes in arm object, joint positions array, returns nothing
    def set_dofs(self, dof_positions): # need to define set_positions for MotorDOFs
        self.vertical.set_position(dof_positions[0])
        self.wrist_rotate.set_position(dof_positions[1])
        self.wrist_tilt.set_position(dof_positions[2])
        #self.gripper_A.set_position(dof_positions[3])
        #self.gripper_B.set_position(dof_positions[4])

    #FORMS OF COMMAND:
    # MOVE UP/DOWN ... STOP
    # ROTATE TOWARDS/AWAY FROM ME ___ DEGREES
    # PAN LEFT/RIGHT ___ DEGREES
    # GO TO (X,Y,Z)
    # OPEN GRIPPER ___

# *** NEED TO ADD IN CATCHING ERRORS HERE ***

    # takes in speech command as string, returns nothing
    def parse_text(self, command):
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
                self.wrist_tilt.set_position(self.q[1])
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
