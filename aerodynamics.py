class aero():
    def __init__(self, name):
        self.name = name
        self.rho = 1.225
        self.cr = 0.1

    def moment(self, pqr, uvw, quat, thrust, torque):
        L = 0.0
        M = 0.0
        N = 0.0
        return np.array([[L], [M], [N]])
