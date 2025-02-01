from vpython import *
from stl import mesh
from PIL import Image
import numpy as np

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
        # VPythonのシーンを設定
        self.scene = canvas(title='StampFly Simulation', width=600, height=600)
        self.scene.ambient = vec(0.37, 0.37, 0.37)  # 環境光を明るくする
        self.fps = fps
        self.anim_time = 0.0

        #Cameraの設定
        self.camera_init()

        #床面を表示
        self.floor_object()

        #Ringを表示
        ring1=self.ring_object(pos=vec(2.5, 0.0, 0.0))
        ring2=self.ring_object(pos=vec(5.0, 0.0, 0.0))
        
        #StampFly表示
        self.stampfly_object()

    def floor_object(self):
        # 背景のボックスにテクスチャを適用
        #self.make_texture()
        background = box(pos=vector(0.0, 0.0, -4.0), size=vector(0.1, 0.1, 120.0), texture="checkerboard.png")        
        background.pos = vec(0.0, 0.0, 7.0)
        background.axis = vec(0.0, 120.0, 0.0)#物体の回転軸
        angle = 0
        x=sin(radians(angle))
        z=cos(radians(angle))
        background.up = vec(x, 0, z)#axisを回転軸としたときupの向きで回転方向を決める

    def ring_object(self,pos):
        return ring(pos=pos, axis=vec(1, 0, 0), radius = 0.3, thickness = 0.03, color=color.red)

    def make_texture(self):
        # 市松模様の画像を生成
        size = 4096 # 画像のサイズ
        N_size = 64  # 市松模様の1辺のマスの数
        tile_size = size // N_size  # 市松模様の1マスのサイズ
        image = Image.new("RGB", (size, size), "white")
        for i in range(N_size):
            for j in range(N_size):
                if (i + j) % 2 == 0:
                    for x in range(tile_size):
                        for y in range(tile_size):
                            image.putpixel((i * tile_size + x, j * tile_size + y), (7, 179, 41, 255))
        # 画像を保存
        image.save("checkerboard.png")


    def stampfly_object(self):
        # STLファイルを読み込む（ファイルパスを指定）
        stl_mesh = mesh.Mesh.from_file('StampFly.stl')

        obj=[]
        # STLメッシュデータの頂点をVPython用に変換して表示
        for i in range(len(stl_mesh.vectors)):
            #print(i)
            # 各三角形の頂点を取得
            p0=vector(*stl_mesh.vectors[i][0])/1000
            p1=vector(*stl_mesh.vectors[i][1])/1000
            p2=vector(*stl_mesh.vectors[i][2])/1000
            normal = norm(cross((p1-p0),(p2-p1)))
            if i < 4520:
                #フレーム
                r=0.9
                g=0.9
                b=0.8
                opacity = 1.0
            elif i< 4730:
                #モータ1
                r = 0.8
                g = 1.0
                b = 1.0
                opacity = 1.0
            elif i< 5450:
                #プロペラ1
                r = 1.0
                g = 0.2
                b = 0.2
                opacity = 0.5
            elif i< 5660:
                #モータ２
                r = 0.8
                g = 1.0
                b = 1.0
                opacity = 1.0
            elif i< 6050:
                #モータ３　モータ４
                r = 0.8
                g = 1.0
                b = 1.0
                opacity = 1.0
            elif i< 8120:
                #プロペラ２
                r = 1.0
                g = 0.2
                b = 0.2      
                opacity = 0.5      
            elif i< 8411:
                #M5StampS3
                r = 0.9
                g = 0.45
                b = 0.0
                opacity = 1.0
            else:
                r = 0.2
                g = 0.2
                b = 0.2
                opacity = 1.0   
            #print(r,g,b)
            color = vector(r,g,b)
            v0 = vertex(pos=p0, normal=normal, color=color)
            v1 = vertex(pos=p1, normal=normal, color=color)
            v2 = vertex(pos=p2, normal=normal, color=color)
            #print(v0)
            # VPythonのtriangleオブジェクトとして描画
            tri=triangle(
                v0=v0,
                v1=v1,
                v2=v2,
            )
            obj.append(tri)
            #sleep(0.001)

        self.copter = compound(obj)
        self.copter.pos = vec(0.0, 0.0, 0.0)
        self.copter.axis = vec(1,0,0)
        self.copter.up = vec(0,1,0)

    def camera_init(self):
        #Cameraの設定
        #カメラの見たい場所
        xf = 0.0
        yf = 0.0
        zf = 0.0
        
        #カメラの位置
        self.xc =  xf - 0.00 #scene.upが(0,0,-1)のためこれがうまく表示されない。(0,1,0)に変更するとうまくいく
        self.yc =  yf - 0.00
        self.zc =  zf - 1.0

        #カメラの向き
        axis_x = xf - self.xc
        axis_y = yf - self.yc
        axis_z = zf - self.zc
        d = sqrt(axis_x**2 + axis_y**2 + axis_z**2)
        
        #見える奥行き範囲を延長するための処理
        axis_x = axis_x*4
        axis_y = axis_y*4
        axis_z = axis_z*4
        xf = self.xc + axis_x
        yf = self.yc + axis_y
        zf = self.zc + axis_z

        self.scene.autoscale = True  # オートスケールを無効
        self.scene.center = vector(xf, yf, zf)  # カメラの注視点
        self.scene.camera.pos = vector(self.xc, self.yc, self.zc)  # カメラの位置
        self.scene.camera.axis = vector(axis_x, axis_y, axis_z)  # カメラの向き
        self.scene.up=vector(1,0,0)
        
        #FOVの設定
        scene_range = 0.5
        self.scene.fov = 2*atan2(scene_range, d)

        



    def camera_setting(self, drone):
        #Cameraの設定
        #カメラの見たい場所
        xf = drone.body.position[0][0]
        yf = drone.body.position[1][0]
        zf = drone.body.position[2][0]
        
        #カメラの位置
        self.xc =  0#xf - 0.00 #scene.upが(0,0,-1)のためこれがうまく表示されない。(0,1,0)に変更するとうまくいく
        self.yc =  0#yf - 0.00
        self.zc =  -1#zf - 5.0

        #カメラの向き
        axis_x = xf - self.xc
        axis_y = yf - self.yc
        axis_z = zf - self.zc
        d = sqrt(axis_x**2 + axis_y**2 + axis_z**2)
        
        #見える奥行き範囲を延長するための処理
        axis_x = axis_x*4
        axis_y = axis_y*4
        axis_z = axis_z*4
        xf = self.xc + axis_x
        yf = self.yc + axis_y
        zf = self.zc + axis_z

        self.scene.autoscale = True  # オートスケールを無効
        self.scene.center = vector(xf, yf, zf)  # カメラの注視点
        self.scene.camera.pos = vector(self.xc, self.yc, self.zc)  # カメラの位置
        self.scene.camera.axis = vector(axis_x, axis_y, axis_z)  # カメラの向き
        self.scene.up=vector(0,0,-1)

        #FOVの設定
        scene_range = 0.2
        self.scene.fov = 2*atan2(scene_range, d)

        #print(self.scene.camera.axis, degrees(self.scene.fov), self.scene.range)

    def rendering(self, sim_time, drone):
        #3D描画
        if(sim_time > self.anim_time):
            rate(self.fps)
            self.camera_setting(drone)
            self.copter.pos = vector(*drone.body.position )
            self.copter.axis = vector(drone.body.DCM[0,0], drone.body.DCM[1,0], drone.body.DCM[2,0])
            self.copter.up = vector(-drone.body.DCM[0,2], -drone.body.DCM[1,2], -drone.body.DCM[2,2])
            self.anim_time += 1/self.fps
            
            
