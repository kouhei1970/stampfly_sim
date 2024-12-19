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

    '''
    def __init__(self, mass, inersia):
        self.body = rb.rigidbody(mass=mass, inersia=inersia)
        
        self.mp1 = mp.motor_prop()
        self.mp2 = mp.motor_prop()
        self.mp3 = mp.motor_prop()
        self.mp4 = mp.motor_prop()
        self.motor_prop = [self.mp1, self.mp2, self.mp3, self.mp4]

        self.battery = bt.battery()

    def force_moment(self):
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
        kappa1 = self.mp1.kappa
        kappa2 = self.mp2.kappa
        kappa3 = self.mp3.kappa
        kappa4 = self.mp4.kappa    
        Moment_L = -thrust1 * army1 - thrust2 * army2 + thrust3 * army3 + thrust4 * army4
        Moment_M = thrust1 * armx1 - thrust2 * armx2 - thrust3 * armx3 + thrust4 * armx4
        Moment_N = thrust1 * kappa1 - thrust2 * kappa2 + thrust3 * kappa3 - thrust4 * kappa4
        Thrust = np.array([[0.0],[0.0],[-(thrust1+thrust2+thrust3+thrust4)]])
        Moment = np.array([[Moment_L],[Moment_M],[Moment_N]])
        return Thrust, Moment

    def step(self,voltage, dt):
        #Body state update
        force, moment = self.force_moment()
        self.body.step(force, moment, dt)
        #Motor and Prop state update
        for index, mp in enumerate(self.motor_prop):
            mp.step(voltage[index], dt)

    #EOF