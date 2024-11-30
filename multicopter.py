import numpy as np
import motor_prop as mp
import rigid_body as rb
import battery as bt

class multicopter():
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
        

        force = np.array([[0.0], [0.0], [-thrust]])
        
        return np.array([[0.0], [0.0], [thrust]]) 

    def moment(self, voltage):





