import rospy
import math
from pyquaternion import Quaternion
class a_PID:
    def __init__(self, Kp=0.6, Ki=0.6, Kd=0.5, maxI=0.07, maxOut=0.9,minOut=0.3):
        #Kp = 0.6,Ki = 0.6, Kd = 0.5
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.target = 0
        self.vel_setpoint = 0
        self.output = 0
        self.error = 0
        self.maxI = maxI
        self.maxOut = maxOut
        self.minOut = minOut
        self.reset()
        self.lastTime = rospy.get_time()
        self.roll = 0
        self.pitch = 0
        self.offset = 0.605

    def generate_attitude_thrust(self,x,y,z,state,z_vel):
        z = z + 9.8
        z = 1 if z ==0 else z

        roll = math.atan2(x,z)
        pitch = math.atan2(y,z)

        #Thresholding
        thresh = 45*math.pi/180.0
        roll = thresh if roll > thresh else -thresh if roll<-thresh else roll
        pitch = thresh if pitch > thresh else -thresh if pitch<-thresh else pitch

        q_roll = Quaternion(axis=[0,1,0],angle=-1*roll)
        q_pitch = Quaternion(axis=[1,0,0],angle=pitch)
        q_out = q_roll * q_pitch
        self.roll = roll
        self.pitch = pitch

        thrust = self.update(2,state,z_vel)
        w,i,j,k = q_out

        return w,i,j,k,thrust 

    def update(self, target, state, z_vel):
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
        self.vel_error = self.vel_setpoint - z_vel 
        self.Dout = self.Kd * self.vel_error

        tilted_offset = self.offset/math.cos(self.pitch)/math.cos(self.roll)
        output = self.Pout + self.intError + self.Dout + tilted_offset

        # Make sure output does not exceed maximum
        if (self.maxOut is not None and output > self.maxOut):
            output = self.maxOut
        elif (self.maxOut is not None and output < self.minOut):
            output = self.minOut 

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
