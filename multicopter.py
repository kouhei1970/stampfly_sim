import numpy as np
import motor_prop as mp
import rigid_body as rb
import battery as bt

class multicopter():
    '''
    Rotor configuration:
    FL4   FR1
        X
    RL3   RR2

    DCM(Direct Cosine Matrix) from body frame to inertial frame:
    '''
    def __init__(self):
        self.body = rb.rigidbody(0.5, np.array([[0.0], [0.0], [0.0]]), np.array([[0.0], [0.0], [0.0]]))
        self.mp1 = mp.motor_prop()
        self.mp2 = mp.motor_prop()
        self.mp3 = mp.motor_prop()
        self.mp4 = mp.motor_prop()
        self.battery = bt.battery()

    def force_moment(self, voltage):
        thrust1 = self.mp1.get_thrust()
        thrust2 = self.mp2.get_thrust()
        thrust3 = self.mp3.get_thrust()
        thrust4 = self.mp4.get_thrust()
        armx1 = self.mp1.armx
        armx2 = self.mp2.armx
        armx3 = self.mp3.armx
        armx4 = self.mp4.armx
        army1 = self.mp1.army
        army2 = self.mp2.army
        army3 = self.mp3.army
        army4 = self.mp4.army
        k1 = self.mp1.k
        k2 = self.mp2.k
        k3 = self.mp3.k
        k4 = self.mp4.k        
        Thrust = thrust1 + thrust2 + thrust3 + thrust4
        Moment_L = -thrust1 * army1 - thrust2 * army2 + thrust3 * army3 + thrust4 * army4
        Moment_M = thrust1 * armx1 - thrust2 * armx2 - thrust3 * armx3 + thrust4 * armx4
        Moment_N = thrust1 * k1 - thrust2 * k2 + thrust3 * k3 - thrust4 * k4
        return np.array([[Thrust], [Moment_L], [Moment_M], [Moment_N]]) 






