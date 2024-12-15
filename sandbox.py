import rigid_body as rb
import numpy as np
import matplotlib.pyplot as plt

pqr0= [[36*np.pi/180],[0.001],[0.0]]
body = rb.rigidbody( pqr=pqr0)

t=0.0
h = 0.001
tmax =20.0

T=[]
EULER=[]
PQR=[]

T.append(t)
EULER.append(body.euler)
PQR.append(body.pqr)

for i in range(int(tmax/h)):
    force = [[0.0],[0.0],[0.0]]
    torque = [[0.0],[0.0],[0.0]]
    body.step(force=force, torque=torque, h=h)
    t = t + h
    #print(body.euler[0][0], body.euler[1][0], body.euler[2][0])
    T.append(t)
    EULER.append(body.euler)
    PQR.append(body.pqr)

T=np.array(T)
EULER=np.array(EULER)
PQR=np.array(PQR)
#print(EULER)
#print(T)
#print(EULER[:,0,0])
plt.plot(T, PQR[:,0,0], label='P')
plt.plot(T, PQR[:,1,0], label='Q')
plt.plot(T, PQR[:,2,0], label='R')
plt.legend()
plt.grid()
plt.xlabel('Time(s)')
plt.ylabel('Euler angle(rad)')
plt.show()
