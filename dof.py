import gpiozero


class DOF:

    def __init__(self):
        pass

    #Will be in degrees or mm
    def set_position(self, position):
        pass

    def set_velocity(self, velocity):
        pass

    def set_limits(self, minLimit, maxLimit):
        self.minLimit = minLimit
        self.maxLimit = maxLimit

class ServoDOF(DOF):

    def __init__(self, pin):
        self.servo = gpiozero.AngularServo(pin)
        self.set_limits(-90, 90)

    def set_position(self, position):
        if position >= self.minLimit and position <= self.maxLimit:
            self.servo.angle = position
        else:
            print('Position input exceeds limits')


class MotorDOF(DOF):
    
    def __init__(self, pinA, pinB):
        self.motor = gpiozero.Motor(pinA, pinB)

    def setPower(self, power):
        if power > 0:
            self.motor.forward(power)
        else:
            self.motor.backward(power*-1)
            
    def stop(self):
        self.motor.forward(0)
