import numpy as np
import motor_prop as mp

class multicopter():
    def __init__(self):
        self.mp1 = mp.motor_prop()
        self.mp2 = mp.motor_prop()
        self.mp3 = mp.motor_prop()
        self.mp4 = mp.motor_prop()
        self.pqr = np.array([[0.0], [0.0], [0.0]])
        self.uvw = np.array([[0.0], [0.0], [0.0]])
        self.quat = np.array([[0.0], [0.0], [0.0], [0.0]])
        self.euler = np.array([[0.0], [0.0], [0.0]])
        self.dcm = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])



