import rospy
import math
from pyquaternion import Quaternion
class v_PID:
    def __init__(self, Kp=0.6, Ki=0.6, Kd=0.5):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.target = 0
        self.vel_setpoint = 0
        self.output = 0
        self.error = 0
        self.maxI=None
        self.maxOut=None
        self.minOut=None
        self.reset()
        self.lastTime = rospy.get_time()
        self.last_error = 0.0

    def update(self, target, state):
        # u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}
        self.target = target
        self.state = state
        self.error = self.target - self.state

        #Proportional
        self.Pout = self.Kp * self.error;

        self.dTime = 1/10.0

        #Integral
        self.intError += self.Ki * self.error * self.dTime

        # Make sure I does not exceed maximum windup
        if (self.maxI is not None and self.intError > self.maxI):
            self.intError = self.maxI
        elif (self.maxI is not None and self.intError < -self.maxI):
            self.intError = -self.maxI

        #Derivative
        self.pos_error = (self.error - self.last_error)/self.dTime 
        self.Dout = self.Kd * self.pos_error

        output = self.Pout + self.intError + self.Dout

        # Make sure output does not exceed maximum
        if (self.maxOut is not None and output > self.maxOut):
            output = self.maxOut
        elif (self.maxOut is not None and output < self.minOut):
            output = self.minOut 

        self.last_error = self.error

        return output

    def setKp(self, proportional_gain):
        self.Kp = Kp

    def setKi(self, Ki):
        self.Ki = Ki

    def setKd(self, Kd):
        self.Kd = Kd

    def setMaxI(self, maxI):
        self.maxI = maxI

    def reset(self):
        self.target = 0.0
        self.error = 0.0
        self.state = 0.0
        self.intError = 0.0
        self.lastError = 0.0
        self.output = 0.0
