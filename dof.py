import pigpio
import time


class DOF(object):

    FREQUENCY = 2000
    POWER_FACTOR = 100000
    HIGH = 1
    LOW = 0

    min_limit = None
    max_limit = None

    pi = pigpio.pi()

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
    
    pin = None
    # actual limits 500, 2500 (500 barrier for safety as suggested online)
    MIN_LIMIT = 1000
    MAX_LIMIT = 2000

    def __init__(self, pin):
        super(ServoDOF, self).__init__()
        self.pin = pin
        self.pi.set_mode(self.pin, pigpio.OUTPUT)
        self.set_limits(self.MIN_LIMIT, self.MAX_LIMIT)
        return

    def set_position(self, position):
        if self.min_limit <= position <= self.max_limit:
            self.pi.set_servo_pulsewidth(self.pin, position)
            return True
        print('Position input exceeds limits ' + str(position))
        return False


class MotorDOF(DOF):  # TODO: implement limits with encoder

    pin_a = None
    pin_b = None
    pwm_pin = None

    def __init__(self, pin_a, pin_b, pwm_pin):
        super(MotorDOF, self).__init__()
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.pi.set_mode(self.pin_a, pigpio.OUTPUT)
        self.pi.set_mode(self.pin_b, pigpio.OUTPUT)
        self.pwm_pin = pwm_pin
        return

    def set_power(self, power):  # decide on stop or set_power(0)
        if power > 0:
            self.pi.hardware_PWM(self.pwm_pin, self.FREQUENCY, power * self.POWER_FACTOR)
            self.pi.write(self.pin_a, self.HIGH)
            self.pi.write(self.pin_b, self.LOW)
            return
        if power < 0:
            self.pi.hardware_PWM(self.pwm_pin, self.FREQUENCY, power * -1 * self.POWER_FACTOR)
            self.pi.write(self.pin_a, self.LOW)
            self.pi.write(self.pin_b, self.HIGH)
            return
        if power == 0:
            self.pi.hardware_PWM(self.pwm_pin, 0, 0)
            self.pi.write(self.pin_a, self.LOW)
            self.pi.write(self.pin_b, self.LOW)
        return

    def stop(self):
        self.set_power(0)
        return


class GripperDOF(MotorDOF):
    def __init__(self, pin_a, pin_b, pwm_pin):
        super(GripperDOF, self).__init__(pin_a, pin_b, pwm_pin)
        return

    def open(self, power, sleep_time):
        self.set_power(power)
        time.sleep(sleep_time)
        self.stop()
        return

    def close(self, power):
        self.set_power(-1 * power)
        return
