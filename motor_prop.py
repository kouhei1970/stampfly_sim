class motor_prop():
    def __init__(self):
        self.omega = 0.0
        self.e = 0.0
        self.i = 0.0
        self.thrust = 0.0

        #実験値
        #回転数と電圧の関係から求めたパラメータ
        self.Am = 5.39e-8
        self.Bm = 6.33e-4
        self.Cm = 1.53e-2
        #LCRメータで測定したパラメータ
        self.Lm = 1.0e-6
        self.Rm = 0.34
        #回転数と推力・トルク測定実験から求めたパラメータ
        self.Ct = 1.0e-8
        self.Cq = 9.75e-11
        #形状と重量から推定した慣性モーメント
        self.Jmp = 2.01e-8 

        #推定値
        self.Km = self.Cq*self.Rm/self.Am
        self.Dm = (self.Bm - self.Cq*self.Rm/self.Am)*(self.Cq/self.Am)
        self.Qf = self.Cm*self.Cq/self.Am
        self.kappa = self.Cq/self.Ct

        #self.Km = 6.15e-4
        #self.Lm = 1.0e-6
        #self.Dm = 3.25e-8
        #self.Qf = 2.77e-5
        #self.kappa = self.Cq/self.Ct

        #パラメータの確認
        #print('A=',(self.Cq*self.Rm/self.Km))    
        #print('B=',(self.Dm+self.Km**2/self.Rm)/(self.Km/self.Rm))
        #print('C=',(self.Qf*self.Rm/self.Km))    

        self.armx = 0.025
        self.army = 0.025


    def omega_dot(self, omega, voltage):
        return ( -(self.Dm + self.Km**2/self.Rm ) * omega - self.Cq * omega - self.Qf + self.Km * voltage/self.Rm)/self.Jmp

    def get_current(self, voltage):
        return (voltage - self.Km * self.omega)/self.Rm
    
    def get_thrust(self):
        return self.Ct * self.omega**2
    
    def get_torque(self):
        return self.Cq * self.omega**2

    def step(self, voltage, dt):
        # Runge-Kutta 4th order
        k1 = self.omega_dot(self.omega, voltage)
        k2 = self.omega_dot(self.omega + k1 * dt / 2.0, voltage)
        k3 = self.omega_dot(self.omega + k2 * dt / 2.0, voltage)
        k4 = self.omega_dot(self.omega + k3 * dt, voltage)
        self.omega += (k1 + 2*k2 + 2*k3 + k4) * dt / 6.0
        self.i = self.get_current(voltage)
        self.thrust = self.get_thrust()
        return self.omega