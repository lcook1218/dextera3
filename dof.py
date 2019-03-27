import time

import pigpio


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

    def set_position(self, position):
        pass

    def set_velocity(self, velocity):
        pass

    def __set_limits(self, min_limit, max_limit):
        self.min_limit = min_limit
        self.max_limit = max_limit
        return


class ServoDOF(DOF):
    
    pin = None
    # actual limits 500, 2500 (500 barrier for safety as suggested online)
    MIN_LIMIT = 1000
    MAX_LIMIT = 2000

    MIN_DEGREE = -90
    MAX_DEGREE = 90

    def __init__(self, pin):
        super(ServoDOF, self).__init__()
        self.pin = pin
        self.pi.set_mode(self.pin, pigpio.OUTPUT)
        self.__set_limits(self.MIN_LIMIT, self.MAX_LIMIT)
        return

    def __convert_to_degrees(self, thousands_val):
        thous_range = self.MAX_LIMIT - self.MIN_LIMIT
        deg_range = self.MAX_DEGREE - self.MIN_DEGREE
        new_value = (thousands_val - self.MIN_LIMIT) * deg_range / thous_range + self.MIN_DEGREE
        return int(new_value)

    def __convert_to_thousands(self, degree_val):
        thous_range = self.MAX_LIMIT - self.MIN_LIMIT
        deg_range = self.MAX_DEGREE - self.MIN_DEGREE
        new_value = (degree_val - self.MIN_DEGREE) * thous_range / deg_range + self.MIN_LIMIT
        return int(new_value)

    def __in_limits(self, thous_position):
        return self.min_limit <= thous_position <= self.max_limit

    def __out_of_limits(self, thous_position):
        print('Position input exceeds limits ' + str(self.__convert_to_degrees(thous_position)))
        return

    def set_position(self, deg_position):
        thous_position = self.__convert_to_thousands(deg_position)
        if self.__in_limits(thous_position):
            self.pi.set_servo_pulsewidth(self.pin, thous_position)
            return True
        self.__out_of_limits(thous_position)
        return False

    def rotate_indefinitely(self, curr_deg_pos, rotate_factor):  # TODO: TEST THIS
        curr_pos = self.__convert_to_thousands(curr_deg_pos)
        while self.__in_limits(curr_pos):
            curr_pos += rotate_factor
            self.pi.set_servo_pulsewidth(self.pin, curr_pos)
        self.__out_of_limits(curr_pos)
        return

    def get_position(self):
        thous_position = self.pi.get_servo_pulsewidth(self.pin)
        return self.__convert_to_degrees(thous_position)

    def stop(self):  # TODO: IMPLEMENT THIS
        pass


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

    def set_position(self, position):  # TODO: IMPLEMENT
        print('not yet implemented')
        return False

    def get_position(self):  # TODO: IMPLEMENT
        pass


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
