import numpy as np

class rigidbody():
    def __init__(self, mass=1.0, inersia=np.eye(3), position=[[0.0],[0.0],[0.0]], velocity=[[0.0],[0.0],[0.0]], pqr=[[0.0],[0.0],[0.0]], euler=[[0.0],[0.0],[0.0]]):
        #DCM(Direct Cosine Matrix) from body frame to inertial frame:
        self.mass = mass
        self.inertia = inersia#np.array()#np.eye(3)
        self.euler = np.array(euler)
        self.quat = self.euler2quat(euler)
        self.DCM = self.quat_dcm(self.quat)
        self.position = np.array(position)#np.zeros((3,1))
        self.velocity = np.array(velocity)#
        self.pqr = np.array(pqr)
        self.uvw = self.velocity2uvw(velocity, self.DCM)

    def quat_dcm(self, quat):
        q0 = quat[0][0]
        q1 = quat[1][0]
        q2 = quat[2][0]
        q3 = quat[3][0]
        DCM = np.array([[q0**2 + q1**2 - q2**2 - q3**2, 2*(q1*q2 + q0*q3), 2*(q1*q3 - q0*q2)], 
                             [2*(q1*q2 + q0*q3), q0**2 - q1**2 + q2**2 - q3**2, 2*(q2*q3 + q0*q1)], 
                             [2*(q1*q3 + q0*q2), 2*(q2*q3 - q0*q1), q0**2 - q1**2 - q2**2 + q3**2]])
        return DCM

    def euler_dcm(self, euler):
        phi = euler[0][0]
        tht = euler[1][0]
        psi = euler[2][0]
        s_phi = np.sin(phi)
        c_phi = np.cos(phi)
        s_tht = np.sin(tht)
        c_tht = np.cos(tht)
        s_psi = np.sin(psi)
        c_psi = np.cos(psi)
        DCM = np.array([[c_tht*c_psi, s_phi*s_tht*c_psi - c_phi*s_psi, c_phi*s_tht*c_psi + s_phi*s_psi], 
                             [c_tht*s_psi, s_phi*s_tht*s_psi + c_phi*c_psi, c_phi*s_tht*s_psi - s_phi*c_psi], 
                             [-s_tht, s_phi*c_tht, c_phi*c_tht]])
        return DCM
        
    def velocity(self, uvw, DCM):
        return DCM @ uvw

    def euler_dot(self, euler, pqr):
        phi = euler[0][0]
        tht = euler[1][0]
        s_phi = np.sin(phi)
        c_phi = np.cos(phi)
        t_phi = np.tan(phi)
        c_tht = np.cos(tht)    
        euler_dot = np.array([[1, s_phi*t_phi, c_phi*t_phi], 
                              [0, c_phi, -s_phi], 
                              [0, s_phi/c_tht, c_phi/c_tht]]) @ pqr
        return euler_dot

    def quat_dot(self, quat, pqr):
        p = pqr[0][0]
        q = pqr[1][0]
        r = pqr[2][0]
        quat_dot = 0.5*np.array([[0, -p, -q, -r], 
                                 [p, 0, r, -q], 
                                 [q, -r, 0, p], 
                                 [r, q, -p, 0]]) @ quat
        return quat_dot
    
    def uvw_dot(self, uvw, pqr, force):
        #m [duvw/dt] + omaga x uvw = force
        #[duvw/dt] = force/m - (omaga x uvw)/m
        force = np.array(force)
        uvw_dot = force/self.mass - np.cross(pqr, uvw, axis=0)/self.mass
        return uvw_dot

    def pqr_dot(self, pqr, torque):
        #dH/dt = torque
        #H = I @ pqr
        #dH/dt = [dH/dt]+pqr x H
        #[dI@pqr/dt] + pqr x (I @ pqr) = torque
        #I @ [dpqr/dt] + pqr x (I @ pqr) = torque
        #I @ [dpqr/dt] = torque - pqr x (I @ pqr)
        #dpqr/dt = I^-1 @ (torque - pqr x (I @ pqr))
        #pqr_dot = I^-1 @ (torque - pqr x (I @ pqr))
        torque = np.array(torque)
        pqr_dot = np.linalg.inv(self.inertia) @ (torque - np.cross(pqr, self.inertia @ pqr, axis=0))
        return pqr_dot
    
    def position_dot(self, uvw, quat):
        #dposition/dt = velocity
        #velocity = DCM @ uvw
        #dposition/dt = DCM @ uvw
        position_dot = self.quat_dcm(quat) @ uvw
        return position_dot

    def uvw2velocity(self, uvw, DCM):
        velocity = DCM @ uvw
        return velocity
    
    def velocity2uvw(self, velocity, DCM):
        uvw = np.linalg.inv(DCM) @ velocity
        return uvw
    
    def euler2quat(self, euler):
        phi = euler[0][0]
        tht = euler[1][0]
        psi = euler[2][0]
        s_phi = np.sin(phi/2)
        c_phi = np.cos(phi/2)
        s_tht = np.sin(tht/2)
        c_tht = np.cos(tht/2)
        s_psi = np.sin(psi/2)
        c_psi = np.cos(psi/2)
        quat = np.array([[c_phi*c_tht*c_psi + s_phi*s_tht*s_psi], 
                         [s_phi*c_tht*c_psi - c_phi*s_tht*s_psi], 
                         [c_phi*s_tht*c_psi + s_phi*c_tht*s_psi], 
                         [c_phi*c_tht*s_psi - s_phi*s_tht*c_psi]])
        return quat
    
    def quat2euler(self, quat):
        q0=quat[0][0]
        q1=quat[1][0]
        q2=quat[2][0]
        q3=quat[3][0]

        asin = -2*(q1*q3 - q0*q2)
        if asin > 1.0:
            asin = 1.0 
        elif asin < -1.0:
            asin = -1.0

        euler = np.array([[np.arctan2(2*(q0*q1 + q2*q3), q0**2 - q1**2 - q2**2 + q3**2)], 
                              [np.arcsin(asin)], 
                              [np.arctan2(2*(q0*q3 + q1*q2), q0**2 + q1**2 - q2**2 - q3**2)]])
        return euler
    
    def normalize_quat(self):
        self.quat = self.quat/np.linalg.norm(self.quat)
    
    def step(self, force, torque, h):
        #RK4で剛体の運動方程式を解くとともに,グローバル座標系の速度,位置,DCMを更新する
        #1. k1を求める
        k1_uvw = self.uvw_dot(self.uvw, self.pqr, force)
        k1_pqr = self.pqr_dot(self.pqr, torque)
        k1_quat = self.quat_dot(self.quat, self.pqr)
        k1_position = self.position_dot(self.uvw, self.quat)
        #2. k2を求める
        k2_uvw = self.uvw_dot(self.uvw + h/2*k1_uvw, self.pqr + h/2*k1_pqr, force)
        k2_pqr = self.pqr_dot(self.pqr + h/2*k1_pqr, torque)
        k2_quat = self.quat_dot(self.quat + h/2*k1_quat, self.pqr + h/2*k1_pqr)
        k2_position = self.position_dot(self.uvw + h/2*k1_uvw, self.quat + h/2*k1_quat)
        #3. k3を求める
        k3_uvw = self.uvw_dot(self.uvw + h/2*k2_uvw, self.pqr + h/2*k2_pqr, force)
        k3_pqr = self.pqr_dot(self.pqr + h/2*k2_pqr, torque)
        k3_quat = self.quat_dot(self.quat + h/2*k2_quat, self.pqr + h/2*k2_pqr)
        k3_position = self.position_dot(self.uvw + h/2*k2_uvw, self.quat + h/2*k2_quat)
        #4. k4を求める
        k4_uvw = self.uvw_dot(self.uvw + h*k3_uvw, self.pqr + h*k3_pqr, force)
        k4_pqr = self.pqr_dot(self.pqr + h*k3_pqr, torque)
        k4_quat = self.quat_dot(self.quat + h*k3_quat, self.pqr + h*k3_pqr)
        k4_position = self.position_dot(self.uvw + h*k3_uvw, self.quat + h*k3_quat)
        #5. 次の状態を求める
        self.uvw += h/6*(k1_uvw + 2*k2_uvw + 2*k3_uvw + k4_uvw)
        self.pqr += h/6*(k1_pqr + 2*k2_pqr + 2*k3_pqr + k4_pqr)
        self.quat += h/6*(k1_quat + 2*k2_quat + 2*k3_quat + k4_quat)
        self.position += h/6*(k1_position + 2*k2_position + 2*k3_position + k4_position)
        #6. quatを正規化する
        self.normalize_quat()
        #7. DCMを更新する
        self.DCM = self.quat_dcm(self.quat)
        #8. eulerを更新する
        self.euler = self.quat2euler(self.quat)
        #9. velocityを更新する
        self.velocity = self.uvw2velocity(self.uvw, self.DCM)        
    