class rigidbody():
    def __init__(self, mass, position, velocity):
        self.mass = mass
        self.position = np.array([[0.0], [0.0], [0.0]])
        self.velocity = np.array([[0.0], [0.0], [0.0]])

        self.pqr = np.array([[0.0], [0.0], [0.0]])
        self.uvw = np.array([[0.0], [0.0], [0.0]])
        self.quat = np.array([[0.0], [0.0], [0.0], [0.0]])
        self.euler = np.array([[0.0], [0.0], [0.0]])
        self.dcm = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])


