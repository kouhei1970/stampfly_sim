from vpython import *
from stl import mesh

class render_bak():
    def __init__(self, fps):
        #3D描画setteng
        self.scene = canvas(title="StampFly Simulation", width=600, height=600)
        #Tハンドルを作成
        self.frame = box( size=vec(0.14, 0.14, 0.02), pos=vec(0.0, 0.0, 0.0), color=color.green )
        self.prop1 = cylinder( radius=0.127/2, length = 0.005, axis=vec(0, 0, 1), pos=vec(0.07,  0.07, -0.021), color=color.yellow )
        self.prop2 = cylinder( radius=0.127/2, length = 0.005, axis=vec(0, 0, 1), pos=vec(0.07, -0.07, -0.021), color=color.yellow )
        self.prop3 = cylinder( radius=0.127/2, length = 0.005, axis=vec(0, 0, 1), pos=vec(-0.07,-0.07, -0.021), color=color.yellow )
        self.prop4 = cylinder( radius=0.127/2, length = 0.005, axis=vec(0, 0, 1), pos=vec(-0.07, 0.07, -0.021), color=color.yellow )
        self.drone = compound( [self.frame, self.prop1, self.prop2, self.prop3, self.prop4] )
        self.drone.pos = vec(0.0, 0.0, 0.0)
        self.drone.axis = vec(1,0,0)

        #Cameraの設定
        self.scene.autoscale = False  # オートスケールを無効
        #scene.center = vector(0, 0, 0)  # カメラの注視点
        self.scene.camera.pos = vector(1.0*0.5, 0, -0.5*0.5)  # カメラの位置
        self.scene.camera.axis = vector(-1.0*0.5, 0, 0.5*0.5)  # カメラの向き
        self.scene.up=vector(0,0,-1)

        self.fps = fps
        self.anim_time = 0.0

    def rendering(self, sim_time, drone):
        #3D描画
        if(sim_time > self.anim_time):
            rate(self.fps)
            self.drone.axis = vector(drone.body.DCM[0,0], drone.body.DCM[1,0], drone.body.DCM[2,0])
            self.drone.up = vector(drone.body.DCM[0,1], drone.body.DCM[1,1], drone.body.DCM[2,1])
            self.anim_time += 1/self.fps

class render():
    def __init__(self, fps):
        # STLファイルを読み込む（ファイルパスを指定）
        stl_mesh = mesh.Mesh.from_file('StampFly.stl')

        # VPythonのシーンを設定
        self.scene = canvas(title='STL Viewer', width=600, height=600)
        #Cameraの設定
        l=0.1
        self.scene.autoscale = False  # オートスケールを無効
        self.center = vector(0, 0, 0)  # カメラの注視点
        self.scene.camera.pos = vector(1.0*l, 0, -0.5*l)  # カメラの位置
        self.scene.camera.axis = vector(-1.0*l, 0, 0.5*l)  # カメラの向き
        self.scene.up=vector(0,0,-1)

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

        self.obj = compound(obj)
        self.obj.pos = vec(0.0, 0.0, 0.0)
        self.obj.axis = vec(1,0,0)
        self.obj.up = vec(0,1,0)

        self.fps = fps
        self.anim_time = 0.0

    def rendering(self, sim_time, drone):
        #3D描画
        if(sim_time > self.anim_time):
            rate(self.fps)
            self.obj.axis = vector(drone.body.DCM[0,0], drone.body.DCM[1,0], drone.body.DCM[2,0])
            self.obj.up = vector(-drone.body.DCM[0,2], -drone.body.DCM[1,2], -drone.body.DCM[2,2])
            self.anim_time += 1/self.fps
