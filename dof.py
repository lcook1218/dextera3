import gpiozero
# import pigpio
import time


class DOF(object):

    min_limit = None
    max_limit = None

    def __init__(self):
        pass

    # Will be in degrees or mm
    def set_position(self, position):
        pass

    def set_velocity(self, velocity):
        pass

    def set_limits(self, min_limit, max_limit):  # TODO: fix this function / class
        self.min_limit = min_limit
        self.max_limit = max_limit
        return


class ServoDOF(DOF):

    def __init__(self, pin):
        super(ServoDOF, self).__init__()
        self.servo = gpiozero.AngularServo(pin)
        self.set_limits(-90, 90)
        return

    def set_position(self, position):
        if self.min_limit <= position <= self.max_limit:
            self.servo.angle = position
            return True
        print('Position input exceeds limits')
        return False


class MotorDOF(DOF):  # TODO: implement limits (timer isn't a bad idea)
    
    def __init__(self, pin_a, pin_b):
        super(MotorDOF, self).__init__()
        self.motor = gpiozero.Motor(pin_a, pin_b)
        return

    def set_power(self, power):
        if power > 0:
            self.motor.forward(power)
        else:
            self.motor.backward(power*-1)
        return

    def stop(self):
        self.motor.forward(0)
        return


class GripperDOF(MotorDOF):
    def __init__(self, pin_a, pin_b):
        super(GripperDOF, self).__init__(pin_a, pin_b)
        return

    def open(self, power, sleep_time):
        self.set_power(power)
        time.sleep(sleep_time)
        self.set_power(0)
        return

    def close(self, power):
        self.set_power(-1 * power)
        return
