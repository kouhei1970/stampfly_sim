import multicopter as mc
import numpy as np
import matplotlib.pyplot as plt
from visualizer import *
from vpython import *
import serial
import struct

def test_template():
    mass = 0.035
    Weight = mass * 9.81
    stampfly = mc.multicopter(mass= mass, inersia=[[9.16e-6, 0.0, 0.0],[0.0, 13.3e-6, 0.0],[0.0, 0.0, 20.4e-6]])
    Render=render(60)
    t =0.0
    h = 0.001

    stampfly.body.set_pqr([[0.0],[0.0],[0.0]])
    stampfly.body.set_uvw([[0.0],[0.0],[0.0]])
    stampfly.set_duturbance(moment=[0.0, 0.0, 0.0], force=[0.0, 0.0, 0.0])
    battery_voltage = 3.7
    nominal_voltage = stampfly.motor_prop[0].equilibrium_voltage(Weight/4)
    damage_voltage = stampfly.motor_prop[0].equilibrium_voltage(Weight/2)
    nominal_anguler_velocity = stampfly.motor_prop[0].equilibrium_anguler_velocity(Weight/4)
    stampfly.mp1.omega = nominal_anguler_velocity
    stampfly.mp2.omega = nominal_anguler_velocity
    stampfly.mp3.omega = nominal_anguler_velocity
    stampfly.mp4.omega = nominal_anguler_velocity
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

    while t < 10.0:
        if t<5.0:
            voltage = [nominal_voltage, nominal_voltage, nominal_voltage, nominal_voltage]
        else:
            voltage = [0.0, damage_voltage, 0.0, damage_voltage]
        stampfly.step(voltage, h)
        Render.rendering(t, stampfly)
        t += h
        T.append(t)
        PQR.append(stampfly.body.pqr.copy())
        UVW.append(stampfly.body.uvw.copy())
        EULER.append(stampfly.body.euler.copy())
        POS.append(stampfly.body.position.copy())



    if True:
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

def test_two_rotor():
    mass = 0.035
    Weight = mass * 9.81
    stampfly = mc.multicopter(mass= mass, inersia=[[9.16e-6, 0.0, 0.0],[0.0, 13.3e-6, 0.0],[0.0, 0.0, 20.4e-6]])
    Render=render(60)
    t =0.0
    h = 0.001

    stampfly.body.set_pqr([[0.0],[0.0],[0.0]])
    stampfly.body.set_uvw([[0.0],[0.0],[0.0]])
    dist = 0*1e-4
    stampfly.set_duturbance(moment=[dist, dist, dist], force=[dist, dist, dist])
    battery_voltage = 3.7
    nominal_voltage = stampfly.motor_prop[0].equilibrium_voltage(Weight/4)
    damage_voltage = stampfly.motor_prop[0].equilibrium_voltage(Weight/2)
    nominal_anguler_velocity = stampfly.motor_prop[0].equilibrium_anguler_velocity(Weight/4)
    stampfly.mp1.omega = nominal_anguler_velocity
    stampfly.mp2.omega = nominal_anguler_velocity
    stampfly.mp3.omega = nominal_anguler_velocity
    stampfly.mp4.omega = nominal_anguler_velocity
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

    while t < 10.0:
        if t<2.0:
            voltage = [nominal_voltage, nominal_voltage, nominal_voltage, nominal_voltage]
        else:
            voltage = [0.0, damage_voltage, 0.0, damage_voltage]
        stampfly.step(voltage, h)
        Render.rendering(t, stampfly)
        t += h
        T.append(t)
        PQR.append(stampfly.body.pqr.copy())
        UVW.append(stampfly.body.uvw.copy())
        EULER.append(stampfly.body.euler.copy())
        POS.append(stampfly.body.position.copy())

    UVW=np.array(UVW)
    PQR=np.array(PQR)
    EULER=np.array(EULER)
    POS=np.array(POS)


    if True:
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

def test_three_rotor():
    mass = 0.035
    Weight = mass * 9.81
    stampfly = mc.multicopter(mass= mass, inersia=[[9.16e-6, 0.0, 0.0],[0.0, 13.3e-6, 0.0],[0.0, 0.0, 20.4e-6]])
    Render=render(60)
    t =0.0
    h = 0.001

    stampfly.body.set_pqr([[0.0],[0.0],[0.0]])
    stampfly.body.set_uvw([[0.0],[0.0],[0.0]])
    dist = 0*1e-4
    stampfly.set_duturbance(moment=[dist, dist, dist], force=[dist, dist, dist])
    battery_voltage = 3.7
    nominal_voltage = stampfly.motor_prop[0].equilibrium_voltage(Weight/4)
    damage_voltage = stampfly.motor_prop[0].equilibrium_voltage(Weight/3)
    nominal_anguler_velocity = stampfly.motor_prop[0].equilibrium_anguler_velocity(Weight/4)
    stampfly.mp1.omega = nominal_anguler_velocity
    stampfly.mp2.omega = nominal_anguler_velocity
    stampfly.mp3.omega = nominal_anguler_velocity
    stampfly.mp4.omega = nominal_anguler_velocity
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

    while t < 10.0:
        if t<5.0:
            voltage = [nominal_voltage, nominal_voltage, nominal_voltage, nominal_voltage]
        else:
            voltage = [0.0, damage_voltage, damage_voltage, damage_voltage]
        stampfly.step(voltage, h)
        Render.rendering(t, stampfly)
        t += h
        T.append(t)
        PQR.append(stampfly.body.pqr.copy())
        UVW.append(stampfly.body.uvw.copy())
        EULER.append(stampfly.body.euler.copy())
        POS.append(stampfly.body.position.copy())

    UVW=np.array(UVW)
    PQR=np.array(PQR)
    EULER=np.array(EULER)
    POS=np.array(POS)


    if True:
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

def test_sim():
    mass = 0.035
    Weight = mass * 9.81
    stampfly = mc.multicopter(mass= mass, inersia=[[9.16e-6, 0.0, 0.0],[0.0, 13.3e-6, 0.0],[0.0, 0.0, 20.4e-6]])
    Render=render(60)
    t =0.0
    h = 0.001

    stampfly.body.set_pqr([[0.0],[0.0],[0.0]])
    stampfly.body.set_uvw([[0.8],[0.0],[0.0]])
    stampfly.set_duturbance(moment=[0.0, 0.0, 0.0], force=[0.0, 0.0, 0.0])
    battery_voltage = 3.7
    nominal_voltage = stampfly.motor_prop[0].equilibrium_voltage(Weight/4)
    damage_voltage = stampfly.motor_prop[0].equilibrium_voltage(Weight/2)
    nominal_anguler_velocity = stampfly.motor_prop[0].equilibrium_anguler_velocity(Weight/4)
    stampfly.mp1.omega = nominal_anguler_velocity
    stampfly.mp2.omega = nominal_anguler_velocity
    stampfly.mp3.omega = nominal_anguler_velocity
    stampfly.mp4.omega = nominal_anguler_velocity
    print(nominal_voltage)
    print(damage_voltage)

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

    
    #Ring座標
    sqrt2 = np.sqrt(2)
    ring_position = [(4, 0, 0), (6, 0, 0), (6+sqrt2, -2+sqrt2, 0), (8, -2, 0), 
                (6+sqrt2, -2-sqrt2, 0),(6, -4, 0), (6-sqrt2, -6+sqrt2, 0), (4, -6, 0),
                (4, -8, 0),(2+sqrt2, -8-sqrt2, 0), (2, -10, 0),(2-sqrt2, -8-sqrt2, 0),
                (0, -8, 0), (0, -6, 0),(0, -4, 0), (0, -2, 0)]
    
    ring_axis = [(1, 0, 0), (1, 0, 0), (-1, 1, 0), (0, 1, 0),
            (1, 1, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0),
            (0, 1, 0), (1, 1, 0), (1, 0, 0),(-1, 1, 0), 
            (0, 1, 0), (0, 1, 0), (0, 1, 0), (0, 1, 0)]

    vel_ref = 4.0
    uvw_ref = [[[vel_ref],[0.0],[0.0]], [[vel_ref],[0.0],[0.0]], [[vel_ref],[0.0],[0.0]], [[vel_ref],[0.0],[0.0]],
               [[vel_ref],[0.0],[0.0]], [[vel_ref],[0.0],[0.0]], [[vel_ref],[0.0],[0.0]], [[vel_ref],[0.0],[0.0]],
               [[vel_ref],[0.0],[0.0]], [[vel_ref],[0.0],[0.0]], [[vel_ref],[0.0],[0.0]], [[vel_ref],[0.0],[0.0]],
               [[vel_ref],[0.0],[0.0]], [[vel_ref],[0.0],[0.0]], [[vel_ref],[0.0],[0.0]], [[vel_ref],[0.0],[0.0]]]
    
    radius = 2.0
    r_ref = vel_ref/radius
    pqr_ref = [[[0.0],[0.0],[0.0]], [[0.0],[0.0],[0.0]], [[0.0],[0.0],[r_ref]], [[0.0],[0.0],[r_ref]],
               [[0.0],[0.0],[r_ref]], [[0.0],[0.0],[r_ref]], [[0.0],[0.0],[-r_ref]], [[0.0],[0.0],[-r_ref]],
               [[0.0],[0.0],[0.0]], [[0.0],[0.0],[r_ref]], [[0.0],[0.0],[r_ref]], [[0.0],[0.0],[r_ref]],
               [[0.0],[0.0],[r_ref]], [[0.0],[0.0],[0.0]], [[0.0],[0.0],[0.0]], [[0.0],[0.0],[0.0]]]
                
    flag = 0
    index = 0
    eps =1e-4
    forward_vec = np.array([1.0, 0.0, 0.0])
    while t < 24.0:
        voltage = [nominal_voltage, nominal_voltage, nominal_voltage, nominal_voltage]
        stampfly.body.set_uvw(uvw_ref[index])
        stampfly.body.set_pqr(pqr_ref[index])
        distance = np.linalg.norm(stampfly.body.position.T[0] - np.array(ring_position[index]))
        #print(stampfly.body.position.T[0],ring_position[index], distance)
        #previous_forward_vec = forward_vec.copy()
        ring_vec = np.array(ring_position[index]) - stampfly.body.position.T[0]
        if forward_vec @ ring_vec < 0:
            index += 1
            if index > 15:
                index = 15
            print(index)
            forward_vec = np.array(ring_position[index]) - stampfly.body.position.T[0]
        stampfly.step(voltage, h)
        t += h
        if np.isnan(stampfly.body.uvw[0][0]):
            break 

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

    if False:
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
    np.random.seed(1)
    test_two_rotor()
