import multicopter as mc
import numpy as np
import matplotlib.pyplot as plt
from visualizer import *
from vpython import *

def test_sim():
    stampfly = mc.multicopter(mass= 0.035, inersia=[[9.16e-6,0,0],[0,13.3e-6,0],[0,0,20.4e-6]])
    Render=render(60)
    t =0.0
    h = 0.001
    T=[]
    PQR=[]
    UVW=[]
    EULER=[]
    POS=[]
    T.append(t)
    PQR.append(stampfly.body.pqr.copy())
    UVW.append(stampfly.body.uvw.copy())
    EULER.append(stampfly.body.euler.copy())
    POS.append(stampfly.body.position.copy())

    dv = 0.005
    while t < 10.0:
        if t<1.0:
            voltage = [3.7*0.5, 3.7*0.5, 3.7*0.5 ,3.7*0.5] 
        else:
            voltage = [3.7*0.6+dv, 3.7*0.6-dv, 3.7*0.6+dv, 3.7*0.6-dv]
            
        stampfly.step(voltage, h)
        t += h

        Render.rendering(t, stampfly)

        T.append(t)
        PQR.append(stampfly.body.pqr.copy())
        UVW.append(stampfly.body.uvw.copy())
        EULER.append(stampfly.body.euler.copy())
        POS.append(stampfly.body.position.copy())

    T=np.array(T)
    EULER=np.array(EULER)
    PQR=np.array(PQR)
    UVW=np.array(UVW)
    POS=np.array(POS)

    plt.subplot(4,1,1)
    plt.plot(T, UVW[:,0,0], label='u')
    plt.plot(T, UVW[:,1,0], label='v')
    plt.plot(T, UVW[:,2,0], label='w')
    plt.legend()
    plt.grid()
    plt.xlabel('Time(s)')
    plt.ylabel('uvw(m/s)')

    plt.subplot(4,1,2)
    plt.plot(T, PQR[:,0,0], label='P')
    plt.plot(T, PQR[:,1,0], label='Q')
    plt.plot(T, PQR[:,2,0], label='R')
    plt.legend()
    plt.grid()
    plt.xlabel('Time(s)')
    plt.ylabel('PQR(rad/s)')

    plt.subplot(4,1,3)
    plt.plot(T, EULER[:,0,0], label='phi')
    plt.plot(T, EULER[:,1,0], label='theta')
    plt.plot(T, EULER[:,2,0], label='psi')
    plt.legend()
    plt.grid()
    plt.xlabel('Time(s)')
    plt.ylabel('Euler angle(rad)')

    plt.subplot(4,1,4)
    plt.plot(T, POS[:,0,0], label='X')
    plt.plot(T, POS[:,1,0], label='Y')
    plt.plot(T, POS[:,2,0], label='Z')
    plt.legend()
    plt.grid()
    plt.xlabel('Time(s)')
    plt.ylabel('Position(m)')

    plt.show()




def test_stampfly_motor():
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
    test_sim()
