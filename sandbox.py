import rigid_body as rb
import numpy as np
import matplotlib.pyplot as plt

from vpython import *

#3D描画setteng
scene = canvas(title="Drone Simulation", width=600, height=600)
#Tハンドルを作成
handle1 = box( size=vec(0.2, 0.05, 1.0), pos=vec(0.0, 0.0, 0.0), color=color.red )
handle2 = box( size=vec(1.0, 0.05, 0.2), pos=vec(0.6, 0.0, 0.0), color=color.green )
T_handle = compound( [handle1, handle2] )
T_handle.pos = vec(-0.05, 0, 0)
T_handle.axis = vec(1,0,0)


#Cameraの設定
scene.autoscale = False  # オートスケールを無効
#scene.center = vector(0, 0, 0)  # カメラの注視点
scene.camera.pos = vector(0, 0,  2.5)  # カメラの位置
scene.camera.axis = vector(0, 0,-2.5)  # カメラの向き
#print(scene.camera.axis)
#print(scene.camera.pos)
#print(scene.center)
#print(frame.axis)

#シミュレーションの初期値
pqr0= [[270*np.pi/180],[0.001],[0.0]]
body = rb.rigidbody( pqr=pqr0)

t=0.0
fps = 60
anim_time = 1/fps
h = 0.01
tmax =5.0

T=[]
EULER=[]
PQR=[]

T.append(t)
EULER.append(body.euler)
PQR.append(body.pqr)


for i in range(int(tmax/h)):
    rate(fps)
    force = [[0.0],[0.0],[0.0]]
    torque = [[0.0],[0.0],[0.0]]
    body.step(force=force, torque=torque, h=h)
    t = t + h
    T.append(t)
    print(body.pqr)
    EULER.append(body.euler)
    PQR.append(body.pqr)


    #3D描画
    if(t>anim_time):
        T_handle.axis = vector(body.DCM[0,0], body.DCM[1,0], body.DCM[2,0])
        T_handle.up = vector(body.DCM[0,1], body.DCM[1,1], body.DCM[2,1])
        anim_time += 1/fps
    #frame.pos = vector(0, 0, 0)
    #frame.rotate(axis=frame.axis, angle=pi / 50, origin=frame.pos)
    #frame_red.rotate(axis=frame_red.axis, angle=-pi / 100, origin=frame_red.pos)



T=np.array(T)
EULER=np.array(EULER)
PQR=np.array(PQR)

print("PQR")
print(PQR)
print("PQR_COL")
print(PQR[:,0,0])

plt.subplot(3,1,1)
plt.plot(T, PQR[:,0,0], label='P')
plt.plot(T, PQR[:,1,0], label='Q')
plt.plot(T, PQR[:,2,0], label='R')
plt.legend()
plt.grid()
plt.xlabel('Time(s)')
plt.ylabel('PQR(rad/s)')

plt.subplot(3,1,2)
plt.plot(T, EULER[:,0,0], label='phi')
plt.plot(T, EULER[:,1,0], label='theta')
plt.plot(T, EULER[:,2,0], label='psi')
plt.legend()
plt.grid()
plt.xlabel('Time(s)')
plt.ylabel('Euler angle(rad)')
plt.show()
