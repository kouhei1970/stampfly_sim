#from vpython import canvas, vector, color, triangle
from vpython import *
from stl import mesh
import numpy as np

# STLファイルを読み込む（ファイルパスを指定）
stl_mesh = mesh.Mesh.from_file('StampFly.stl')
#print(*stl_mesh.vectors[1][0])

# VPythonのシーンを設定
scene = canvas(title='STL Viewer', width=600, height=600)
#Cameraの設定
l=0.1
scene.autoscale = False  # オートスケールを無効
center = vector(0, 0, 0)  # カメラの注視点
scene.camera.pos = vector(1.0*l, 0, -0.5*l)  # カメラの位置
scene.camera.axis = vector(-1.0*l, 0, 0.5*l)  # カメラの向き
scene.up=vector(0,0,-1)

obj=[]
# STLメッシュデータの頂点をVPython用に変換して表示
for i in range(len(stl_mesh.vectors)):
    # 各三角形の頂点を取得
    p0=vector(*stl_mesh.vectors[i][0])/1000
    p1=vector(*stl_mesh.vectors[i][1])/1000
    p2=vector(*stl_mesh.vectors[i][2])/1000
    normal = norm(cross((p1-p0),(p2-p1)))
    v0 = vertex(pos=p0, normal=normal)#, color=color.cyan)
    v1 = vertex(pos=p1, normal=normal)#, color=color.cyan)
    v2 = vertex(pos=p2, normal=normal)#, color=color.cyan)
    #print(v0)
    # VPythonのtriangleオブジェクトとして描画
    tri=triangle(
        v0=v0,
        v1=v1,
        v2=v2,
    )
    obj.append(tri)


obj = compound(obj)
obj.pos = vec(0.0, 0.0, 0.0)
obj.axis = vec(1,0,0)
obj.up = vec(0,0,-1)

sleep(1)
for i in range(361):
    rate(60)
    angle = pi*i/180
    z = cos(-angle)
    y = sin(-angle)
    obj.up = norm(vec(0,y,-z))
