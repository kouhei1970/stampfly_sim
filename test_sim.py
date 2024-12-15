import multicopter as mc
import numpy as np
import matplotlib.pyplot as plt

def test_stampfly():
    stampfly = mc.multicopter()

    t =0.0
    h = 0.001
    OMEGA=[]
    CURRENT=[]
    THRUST=[]
    T=[]
    OMEGA.append(stampfly.mp1.omega)
    CURRENT.append(stampfly.mp1.i)
    THRUST.append(stampfly.mp1.thrust)
    T.append(t)

    while t < 0.4:
        if t<0.2:
            voltage = 3.7*0.5
        else:
            voltage = 3.7*0.6
            
        stampfly.mp1.step(voltage, h)
        t += h
        OMEGA.append(stampfly.mp1.omega)
        CURRENT.append(stampfly.mp1.i)
        THRUST.append(stampfly.mp1.thrust)
        T.append(t)
    
    OMEGA = np.array(OMEGA)
    CURRENT = np.array(CURRENT)
    THRUST = np.array(THRUST)
    T = np.array(T)

    #subplotする
    plt.subplot(3,1,1)
    plt.plot(T, CURRENT)
    plt.grid()
    plt.xlabel('Time(s)')
    plt.ylabel('Current(A)')
    plt.subplot(3,1,2)
    plt.plot(T, 1000*THRUST/9.81)
    plt.grid()
    plt.xlabel('Time(s)')
    plt.ylabel('Thrust(gf)')
    plt.subplot(3,1,3)
    plt.plot(T, OMEGA)
    plt.grid()
    plt.xlabel('Time(s)')
    plt.ylabel('Omega(rad/s)')
    plt.show()

    print(stampfly.mp1.omega)

if __name__ == "__main__":
    test_stampfly()
