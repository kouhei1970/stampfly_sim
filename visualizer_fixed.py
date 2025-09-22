"""
Visualizer (VPython) — stutter-resistant version

主な変更点:
- FOV を固定 (pi/3)
- カメラ追従を指数移動平均で平滑化
- 物理(固定刻み)と描画(rate)を分離する render_frame()
- numpy→VPython vector 変換時に確実に float 化
- autoscale 無効 + range 固定

既存コードからの置換ポイント:
- 物理ループ側では fixed-step で状態更新後に `vis.render_frame(drone, sim_time)` を呼ぶ
- FOV をフレーム毎に再計算していた箇所は削除
"""
from math import pi, cos, sin
from time import time
from typing import Optional

import numpy as np
from vpython import canvas, vector, rate, color, compound, box, sphere


vec = vector  # ショートハンド


class Visualizer:
    def __init__(self,
                 fps: int = 120,
                 win_width: int = 900,
                 win_height: int = 600,
                 bg=color.white,
                 range_fixed: float = 1.8):
        self.fps = int(fps)
        self.scene = canvas(width=win_width, height=win_height, background=bg)
        # カメラ・キャンバス初期化
        self.scene.autoscale = False
        self.scene.fov = pi/3  # ★固定視野角 (≈60°)
        self.scene.range = range_fixed
        self.scene.up = vec(0, 0, -1)

        # 追従スムージング用内部状態
        self._cam_pos = vec(0, -range_fixed*1.2, 0.4)
        self._cam_ctr = vec(0, 0, 0)
        self.scene.camera.pos = self._cam_pos
        self.scene.center = self._cam_ctr

        # ドローン 3D モデル
        self.copter = self._build_copter()

        # UI: テキスト (必要なら任意に)
        try:
            from vpython import label
            self.timer_text = label(pos=vec(0, 0, range_fixed*0.9),
                                    text="Elapsed Time: 0.0 s",
                                    xoffset=0, yoffset=0, height=14, box=False)
        except Exception:
            self.timer_text = None

    # --------------------
    # 3D モデルの組み立て
    # --------------------
    def _build_copter(self):
        arm_len = 0.15
        body = box(pos=vec(0, 0, 0), size=vec(0.08, 0.08, 0.02), color=color.gray(0.6))
        arm_x = box(pos=vec(0, 0, 0), size=vec(2*arm_len, 0.01, 0.01), color=color.gray(0.4))
        arm_y = box(pos=vec(0, 0, 0), size=vec(0.01, 2*arm_len, 0.01), color=color.gray(0.4))
        rotors = []
        for sx, sy in [(1, 1), (1, -1), (-1, 1), (-1, -1)]:
            rotors.append(sphere(pos=vec(sx*arm_len, sy*arm_len, 0), radius=0.015, color=color.blue))
        return compound([body, arm_x, arm_y] + rotors)

    # --------------------
    # カメラ: 平滑追従
    # --------------------
    def _smooth_follow(self, desired_pos: vector, desired_ctr: vector, alpha: float = 0.12):
        # 0<alpha<=1 小さいほどなめらか
        self._cam_pos = self._cam_pos*(1-alpha) + desired_pos*alpha
        self._cam_ctr = self._cam_ctr*(1-alpha) + desired_ctr*alpha
        self.scene.camera.pos = self._cam_pos
        self.scene.center = self._cam_ctr

    def follow_camera_setting(self, pos_xyz: np.ndarray, yaw_rad: float,
                              back: float = 1.0, up: float = 0.15):
        # 機体後方上から追従（スムーズ）
        x, y, z = [float(v) for v in np.ravel(pos_xyz)]
        xc = x - back*cos(yaw_rad)
        yc = y - back*sin(yaw_rad)
        zc = z - up
        self._smooth_follow(vec(xc, yc, zc), vec(x, y, z))

    # --------------------
    # 描画フレームの更新
    # --------------------
    def render_frame(self, drone, sim_time: float, camera_mode: str = "follow"):
        """描画を 1 フレーム進める。
        - 物理は別ループで固定刻み更新済みを想定
        - ここでは rate(self.fps) だけで描画レートを制御
        - `drone` は position(3x1) と DCM(3x3) か quaternion をもつ想定
        """
        rate(self.fps)

        # --- 機体状態を取り出し（安全にスカラー化）
        p = np.ravel(getattr(drone.body, 'position'))
        p = p.astype(float)
        self.copter.pos = vec(p[0], p[1], p[2])

        # 姿勢: DCM を優先、なければ euler から近似
        if hasattr(drone.body, 'DCM'):
            R = np.array(drone.body.DCM, dtype=float)
            # VPython box.axis はローカル x 軸方向、up はローカル z 軸方向を取る
            axis_x = vec(R[0, 0], R[1, 0], R[2, 0])
            axis_z = vec(R[0, 2], R[1, 2], R[2, 2])
            self.copter.axis = axis_x
            self.copter.up = axis_z
        elif hasattr(drone.body, 'euler'):
            # euler = [roll, pitch, yaw] 想定。回転行列へ変換
            roll, pitch, yaw = [float(x) for x in np.ravel(drone.body.euler)]
            cr, sr = np.cos(roll), np.sin(roll)
            cp, sp = np.cos(pitch), np.sin(pitch)
            cy, sy = np.cos(yaw), np.sin(yaw)
            R = np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                          [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                          [-sp,    cp*sr,            cp*cr           ]], dtype=float)
            axis_x = vec(R[0, 0], R[1, 0], R[2, 0])
            axis_z = vec(R[0, 2], R[1, 2], R[2, 2])
            self.copter.axis = axis_x
            self.copter.up = axis_z

        # カメラ更新
        if camera_mode == "follow":
            yaw = None
            if hasattr(drone.body, 'euler'):
                yaw = float(np.ravel(drone.body.euler)[2])
            elif hasattr(drone.body, 'DCM'):
                # yaw の抽出（Z-Y-Xの yaw として近似）
                R = np.array(drone.body.DCM, dtype=float)
                yaw = float(np.arctan2(R[1, 0], R[0, 0]))
            else:
                yaw = 0.0
            self.follow_camera_setting(p, yaw)
        elif camera_mode == "fixed":
            # 固定視点（必要なら外部で self._cam_pos/_ctr を設定）
            self.scene.camera.pos = self._cam_pos
            self.scene.center = self._cam_ctr

        # 時刻表示
        if self.timer_text is not None:
            self.timer_text.text = f"Elapsed Time: {sim_time:.1f} s"


# --------------------
# 参考: 物理ループのひな形
# --------------------
if __name__ == "__main__":
    class DummyBody:
        def __init__(self):
            self.position = np.array([[0.0], [0.0], [0.0]])
            self.euler = np.array([[0.0], [0.0], [0.0]])  # roll, pitch, yaw
            self.DCM = np.eye(3)

    class DummyDrone:
        def __init__(self):
            self.body = DummyBody()

    vis = Visualizer(fps=120)
    drone = DummyDrone()

    # 固定刻み物理 + 描画
    dt_phys = 0.002
    acc = 0.0
    last = time()
    t_sim = 0.0

    while True:
        now = time()
        acc += min(now - last, 0.05)  # スパイククリップ
        last = now

        # 物理更新（ダミーで円運動）
        while acc >= dt_phys:
            t_sim += dt_phys
            r = 0.5
            w = 0.7
            x = r * np.cos(w * t_sim)
            y = r * np.sin(w * t_sim)
            z = 0.2 + 0.1*np.sin(0.8*t_sim)
            drone.body.position[:] = np.array([[x], [y], [z]], dtype=float)
            yaw = np.arctan2(y, x)
            roll = 0.1*np.sin(1.3*t_sim)
            pitch = 0.1*np.cos(1.1*t_sim)
            drone.body.euler[:] = np.array([[roll], [pitch], [yaw]], dtype=float)

            # DCM 再計算（直交化）
            cr, sr = np.cos(roll), np.sin(roll)
            cp, sp = np.cos(pitch), np.sin(pitch)
            cy, sy = np.cos(yaw), np.sin(yaw)
            R = np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                          [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                          [-sp,    cp*sr,            cp*cr           ]], dtype=float)
            # SVD による再直交化（数値誤差対策）
            U, _, Vt = np.linalg.svd(R)
            drone.body.DCM[:] = (U @ Vt)
            acc -= dt_phys

        # 描画は rate ベース
        vis.render_frame(drone, t_sim, camera_mode="follow")
