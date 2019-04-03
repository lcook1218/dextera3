import time

import pigpio

#dof types
#grippers
#pan servo 
#rotate cont. rotation servo
#elevator controller w/ PID

#motor class - can set power, assign pins, limit motor output values (-1 to 1)
#sensor class - can get sensor value, assign pins

#dof class - has position, assigns position limits, can set position or velocity

##Motor  ---
class Motor(object):
	#can set power

	def __init__(self):
		pass

	#sets power between -1 and 1
	#needs to be overridden by subclasses
	def set_power(self, power):
		if abs(power) > 1.0 :
			print("Error: motor assigned power outside -1 to 1")
			return 0
		return power 

class ServoMotor(Motor):
	
	CENTER_DUTY = 1500
	DUTY_AMPLITUDE = 500
	DUTY_MIN = CENTER_DUTY + DUTY_AMPLITUDE
	DUTY_MAX = CENTER_DUTY - DUTY_AMPLITUDE

	pi = pigpio.pi()
	pin = None

	def __init__(self, pin):
		super(GripperDOF, self).__init__()
		self.pin = pin

	def set_power(self, power):
		power = super(GripperDOF, self).set_power(power)
		self.pi.set_servo_pulsewidth(pin, CENTER_DUTY + power * DUTY_AMPLITUDE)

class GearMotor(Motor):

	pi = pigpio.pi()
	pin_a = None
	pin_b = None
	FREQUENCY = 50
	FULL_ON = 1000000

	def __init__(self, pin_a, pin_b):
		super(GripperDOF, self).__init__()

	def set_power(self, power):
		power = super(GripperDOF, self).set_power(power)
		if power > 1:
			self.pi.hardware_PWM(self.FREQUENCY, self.FULL_ON * power)
			self.pi.write(self.pin_b, low)
		if power < 1:
			self.pi.write(self.pin_a, low)
			self.pi.hardware_PWM(self.FREQUENCY, self.FULL_ON * -power)
		if power == 0:
			self.pi.write(pin_a, 0)
			self.pi.write(pin_b, 0)

##Sensors
class Sensor(object):

	def __init__(self):
		pass

	def get_value():
		pass

class Encoder(Sensor):

	pi = pigpio.pi()
	pin_a = None
	pin_b = None
	position = 0

	def interupt_callback(self, gpio, level, tick):
		if (self.pi.read(pin_b)):
			self.position = self.position + 1
		else:
			self.position = self.position - 1

	def reset_position():
		position = 0

	def __init__(self, pin_a, pin_b):
		self.pin_a = pin_a
		self.pin_b = pin_b
		self.pi.callback(self.pin_a, pigpio.RISING_EDGE, self.interupt_callback)
		self.reset_position()

	def get_value():
		return position		

class AnalogInput(Sensor):

	#todo:allow reading from pin 1 (currently only reads pin 0)
	pi = pigpio.pi()
	pin = None
	channel = None

	command = 0b11 << 6
	command |= (0 & 0x07) << 3

	def __init__(self):
		self.channel = self.pi.open(0, 1000000, 0)

	def get_value():
		(count, resp) = pi.spi_xfer(self.channel, [self.command, 0x0, 0x0])
		result = (resp[0] & 0x01) << 9
		result |= (resp[1] & 0xFF) << 1
		result |= (resp[2] & 0x80) >> 7
		value = result & 0x3FF
		return value

##DOFs

class DOF(object):

    def __init__(self):
        pass

    def set_position(self, position):
        pass

    def set_velocity(self, velocity): #will effectively just set power
        pass

    def __set_limits(self, min_limit, max_limit):
        self.min_limit = min_limit
        self.max_limit = max_limit
        return

class GripperDOF():

	motor = none
	close_power = -0.2
	open_power = 0.2
	sleep_time = 0.5

    def __init__(self, motor):
    	self.motor = motor
        return

    def open(self, power, sleep_time):
        self.motor.set_power(self.open_power)
        time.sleep(self.sleep_time) #TODO: maybe should make this non-blocking? idk if it matters
        self.motor.set_power(0)
        return

    def close(self, power):
        self.motor.set_power(self.close_power)
        return


class ServoDOF(DOF):
    
	FREQUENCY = 50

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
            self.pi.hardware_PWM(self.pin, FREQUENCY, (thous_position/20000)*1000000)
            return True
        self.__out_of_limits(thous_position)
        return False

    #deleted rotate indefinelty because not continous rotation - can be done by setting position to max

    def get_position(self):
        thous_position = self.pi.get_servo_pulsewidth(self.pin)
        return self.__convert_to_degrees(thous_position)

    #def stop(self):  # can't tell servo to stop


class MotorPIDDOF(DOF):  # TODO: implement limits with encoder

    motor = None
    sensor = None
    integral = 0
    last_value = None
    pid_enabled = 0
    last_time = None
    target = None
    continous = 0
    kp = None
    ki = None
    kd = None
    s = sched.scheduler(time.time, time.sleep)


    def __init__(self, motor, sensor, continous, kp, ki, kd):
        super(MotorPIDDOF, self).__init__()
        self.motor = motor
        self.sensor = sensot
        self.continous = continous
        self.kp = kp
        self.ki = ki
        self.kd = kd
        return

    def stop(self):
        self.set_power(0)
        return

    def set_position(self, position):  # TODO: IMPLEMENT
    	target = position
        self.enable_pid()
        return False

    def enable_pid():
    	pid_enabled = 1
    	update_pid()
    	#TODO: set up thread to call PID

    def disable_pid():
    	pid_enabled = 0
    	self.motor.set_power(0)

    def update_pid():

    	#TODO: get minimum error for continous rotation
    	error = target - self.sensor.get_value()

    	dT = time.clock() - last_time


    	if integral is not None:
    		integral = integral + error+dT
    	else 
    		integral = 0

    	if last_error is not None:
    		derivative = (error - last_error)/dT

    	self.motor.set_power(kp*error + ki*integral + kd*derivative)
    	
    	last_error = error
    	last_time = time.clock()
    	
    	#temp for debugging thread calls
    	print(last_time)
    	if pid_enabled:
    		self.scheduler.enter(0.02, 1, self.update_pid)
    		self.scheduler.run()

    def get_position(self):  # TODO: IMPLEMENT
        pass

