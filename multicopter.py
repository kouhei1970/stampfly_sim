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
        self.distuerbance_moment = [4.4e-6, 4.4e-6, 4.e-6]
        self.distuerbance_force = [1e-6, 1e-6, 1e-6]
        self.battery = bt.battery()

    def set_duturbance(self, moment, force):
        self.distuerbance_moment = moment
        self.distuerbance_force = force

    def force_moment(self):
        vel_u = self.body.uvw[0][0]
        vel_v = self.body.uvw[1][0]
        vel_w = self.body.uvw[2][0]
        rate_p = self.body.pqr[0][0]
        rate_q = self.body.pqr[1][0]
        rate_r = self.body.pqr[2][0]
        weight = self.body.mass * 9.81
        gravity = np.array([[0.0], [0.0], [weight]])
        gravity_body= self.body.DCM.T @ gravity
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
        
        #Moment
        moment_L = -thrust1 * army1 - thrust2 * army2 + thrust3 * army3 + thrust4 * army4    - 1e-5*np.sign(rate_p)*rate_p**2
        moment_M = thrust1 * armx1 - thrust2 * armx2 - thrust3 * armx3 + thrust4 * armx4     - 1e-5*np.sign(rate_q)*rate_q**2
        moment_N = thrust1 * kappa1 - thrust2 * kappa2 + thrust3 * kappa3 - thrust4 * kappa4 - 1e-5*np.sign(rate_r)*rate_r**2        
        
        #Force
        thrust = -(thrust1+thrust2+thrust3+thrust4)        
        fx = gravity_body[0][0] - 2e-2*np.sign(vel_u)*vel_u**2
        fy = gravity_body[1][0] - 2e-2*np.sign(vel_v)*vel_v**2
        fz = gravity_body[2][0] + thrust - 2e-2*np.sign(vel_w)*vel_w**2
        
        #Add disturbance
        moment_L += np.random.normal(0, self.distuerbance_moment[0])
        moment_M += np.random.normal(0, self.distuerbance_moment[1])
        moment_N += np.random.normal(0, self.distuerbance_moment[2])
        fx += np.random.normal(0, self.distuerbance_force[0])
        fy += np.random.normal(0, self.distuerbance_force[1])
        fz += np.random.normal(0, self.distuerbance_force[2])

        #Output
        Force = [[fx], [fy], [fz]] 
        Moment = np.array([[moment_L],[moment_M],[moment_N]])
        return Force, Moment

    def step(self,voltage, dt):
        #Body state update
        force, moment = self.force_moment()
        self.body.step(force, moment, dt)
        #Motor and Prop state update
        for index, mp in enumerate(self.motor_prop):
            mp.step(voltage[index], dt)

    #EOF