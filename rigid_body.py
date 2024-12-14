import numpy as np

class rigidbody():
    def __init__(self, mass, position, velocity):
        #DCM(Direct Cosine Matrix) from body frame to inertial frame:

        self.mass = mass
        self.inertia = np.zeros((3,3))
        self.position = np.zeros((3,1))
        self.velocity = np.zeros((3,1))
        self.pqr = np.zeros((3,1))
        self.uvw = np.zeros((3,1))
        self.quat = np.zeros((4,1))
        self.euler = np.zeros((3,1))
        self.DCM = np.eye(3)

    def update_quat_dcm(self):
        q0=self.quat[0]
        q1=self.quat[1]
        q2=self.quat[2]
        q3=self.quat[3]
        self.DCM = np.array([[q0**2 + q1**2 - q2**2 - q3**2, 2*(q1*q2 + q0*q3), 2*(q1*q3 - q0*q2)], 
                             [2*(q1*q2 + q0*q3), q0**2 - q1**2 + q2**2 - q3**2, 2*(q2*q3 + q0*q1)], 
                             [2*(q1*q3 + q0*q2), 2*(q2*q3 - q0*q1), q0**2 - q1**2 - q2**2 + q3**2]])
        return self.DCM

    def update_euler_dcm(self):
        phi = self.euler[0]
        tht = self.euler[1]
        psi = self.euler[2]
        s_phi = np.sin(phi)
        c_phi = np.cos(phi)
        s_tht = np.sin(tht)
        c_tht = np.cos(tht)
        s_psi = np.sin(psi)
        c_psi = np.cos(psi)
        self.DCM = np.array([[c_tht*c_psi, s_phi*s_tht*c_psi - c_phi*s_psi, c_phi*s_tht*c_psi + s_phi*s_psi], 
                             [c_tht*s_psi, s_phi*s_tht*s_psi + c_phi*c_psi, c_phi*s_tht*s_psi - s_phi*c_psi], 
                             [-s_tht, s_phi*c_tht, c_phi*c_tht]])
        return self.DCM
        
    def update_velocity(self):
        return self.DCM @ self.uvw

    def euler_dot(self):
        phi = self.euler[0]
        tht = self.euler[1]
        s_phi = np.sin(phi)
        c_phi = np.cos(phi)
        t_phi = np.tan(phi)
        c_tht = np.cos(tht)    
        euler_dot = np.array([[1, s_phi*t_phi, c_phi*t_phi], 
                              [0, c_phi, -s_phi], 
                              [0, s_phi/c_tht, c_phi/c_tht]]) @ self.pqr
        return euler_dot

    def quat_dot(self):
        p = self.pqr[0]
        q = self.pqr[1]
        r = self.pqr[2]
        quat_dot = 0.5*np.array([[0, -p, -q, -r], 
                                 [p, 0, r, -q], 
                                 [q, -r, 0, p], 
                                 [r, q, -p, 0]]) @ self.quat
        return quat_dot
    
    def uvw_dot(self, force):
        #m [duvw/dt] + omaga x uvw = force
        #[duvw/dt] = force/m - (omaga x uvw)/m
        uvw_dot = force/self.mass - np.cross(self.pqr, self.uvw, axis=0)/self.mass
        return uvw_dot

    def pqr_dot(self, torque):
        #dH/dt = torque
        #H = I @ pqr
        #dH/dt = I @ pqr_dot + (pqr x I) @ pqr
        #I @ pqr_dot + (pqr x I) @ pqr = torque
        #I @ pqr_dot = torque - (pqr x I) @ pqr
        # pqr_dot = I^-1 @ (torque - (pqr x I) @ pqr) 
        pqr_dot = np.linalg.inv(self.inertia) @ (torque - np.cross(self.pqr, self.inertia @ self.pqr, axis=0))
        return pqr_dot
    
    def position_dot(self):
        #dposition/dt = velocity
        #velocity = DCM @ uvw
        #dposition/dt = DCM @ uvw
        position_dot = self.DCM @ self.uvw
        return position_dot

    def uvw2velocities(self):
        self.velocity = self.DCM @ self.uvw
        return self.velocity
    
    def velocities2uvw(self):
        self.uvw = np.linalg.inv(self.DCM) @ self.velocity
        return self.uvw
    
    def euler2quat(self):
        phi = self.euler[0]
        tht = self.euler[1]
        psi = self.euler[2]
        s_phi = np.sin(phi/2)
        c_phi = np.cos(phi/2)
        s_tht = np.sin(tht/2)
        c_tht = np.cos(tht/2)
        s_psi = np.sin(psi/2)
        c_psi = np.cos(psi/2)
        self.quat = np.array([[c_phi*c_tht*c_psi + s_phi*s_tht*s_psi], 
                              [s_phi*c_tht*c_psi - c_phi*s_tht*s_psi], 
                              [c_phi*s_tht*c_psi + s_phi*c_tht*s_psi], 
                              [c_phi*c_tht*s_psi - s_phi*s_tht*c_psi]])
        return self.quat
    
    def quat2euler(self):
        q0=self.quat[0]
        q1=self.quat[1]
        q2=self.quat[2]
        q3=self.quat[3]
        self.euler = np.array([[np.arctan2(2*(q0*q1 + q2*q3), 1 - 2*(q1**2 + q2**2))], 
                              [np.arcsin(2*(q0*q2 - q3*q1))], 
                              [np.arctan2(2*(q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2))]])
        return self.euler
    
    def normalize_quat(self):
        self.quat = self.quat/np.linalg.norm(self.quat)
        return self.quat