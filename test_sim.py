import multicopter as mc
import numpy as np
import matplotlib.pyplot as plt

def test_stampfly():
    stampfly = mc.multicopter()

    t =0.0
    h = 0.001
    OMEGA=[]
    T=[]
    OMEGA.append(stampfly.mp1.omega)
    T.append(t)

    while t < 0.2:
        voltage = 1.0
        stampfly.mp1.step(voltage, h)
        t += h
        OMEGA.append(stampfly.mp1.omega)
        T.append(t)

    plt.plot(T, OMEGA)
    plt.grid()
    plt.xlabel('Time(s)')
    plt.ylabel('Omega(rad/s)')
    plt.show()
    print(stampfly.mp1.omega)

if __name__ == "__main__":
    test_stampfly()
