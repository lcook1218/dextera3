import gpiozero
import pigpio
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

    def set_limits(self, min_limit, max_limit):
        self.min_limit = min_limit
        self.max_limit = max_limit
        return


class ServoDOF(DOF):
    
    pi = pigpio.pi()
    pin = None

    def __init__(self, pin):
        super(ServoDOF, self).__init__()
        self.pin = pin
        self.pi.set_mode(self.pin, pigpio.OUTPUT)
        self.set_limits(1000, 2000)  # actual limits 500, 2500 (500 barrier for safety as suggested online)
        return

    def set_position(self, position):
        if self.min_limit <= position <= self.max_limit:
            self.pi.set_servo_pulsewidth(self.pin, position)
            return True
        print('Position input exceeds limits ' + str(position))
        return False


class MotorDOF(DOF):  # TODO: implement limits (timer isn't a bad idea)

    pi = pigpio.pi()
    pin_a = None
    pin_b = None
    FREQUENCY = 2000
    HIGH = 1
    LOW = 0

    def __init__(self, pin_a, pin_b):
        super(MotorDOF, self).__init__()
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.pi.set_mode(self.pin_a, pigpio.OUTPUT)
        self.pi.set_mode(self.pin_b, pigpio.OUTPUT)
        return

    def set_power(self, power):  # decide on stop or set_power(0)
        if power > 0:
            self.pi.hardware_PWM(18, self.FREQUENCY, power * 100000)  # TODO: REFACTOR TO PASS AS VARIABLE
            self.pi.write(self.pin_a, self.HIGH)
            self.pi.write(self.pin_b, self.LOW)
            return
        if power < 0:
            self.pi.hardware_PWM(18, self.FREQUENCY, power * -1000000)
            self.pi.write(self.pin_a, self.LOW)
            self.pi.write(self.pin_b, self.HIGH)
            return
        if power == 0:
            self.pi.hardware_PWM(18, 0, 0)
            self.pi.write(self.pin_a, self.LOW)
            self.pi.write(self.pin_b, self.LOW)
        return

    def stop(self):
        self.set_power(0)
        return


class GripperDOF(MotorDOF):
    def __init__(self, pin_a, pin_b):
        super(GripperDOF, self).__init__(pin_a, pin_b)
        return

    def open(self, power, sleep_time):
        self.set_power(power)
        time.sleep(sleep_time)
        self.stop()
        return

    def close(self, power):
        self.set_power(-1 * power)
        return
