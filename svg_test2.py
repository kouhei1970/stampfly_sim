from svgpathtools import svg2paths
import matplotlib.pyplot as plt
import csv

# SVGファイルを読み込む
svg_file = "stampfly_airfoil.svg"  # あなたのSVGファイル名
paths, attributes = svg2paths(svg_file)

# サンプリングデータと制御点データを格納
sampled_curve_points = []
control_points = []

# ベジエ曲線データを抽出
for path in paths:
    for segment in path:
        # 制御点を格納
        if segment.__class__.__name__ == 'CubicBezier':
            control_points.append((segment.start.real, segment.start.imag))
            control_points.append((segment.control1.real, segment.control1.imag))
            control_points.append((segment.control2.real, segment.control2.imag))
            control_points.append((segment.end.real, segment.end.imag))
        elif segment.__class__.__name__ == 'QuadraticBezier':
            control_points.append((segment.start.real, segment.start.imag))
            control_points.append((segment.control.real, segment.control.imag))
            control_points.append((segment.end.real, segment.end.imag))

        # 曲線の細かい点をサンプリング
        t_values = [t / 100 for t in range(101)]  # 0から1の範囲で100分割
        sampled_points = [segment.point(t) for t in t_values]
        sampled_curve_points += [(p.real, p.imag) for p in sampled_points]

# 重複した制御点を削除
control_points = list(set(control_points))

# CSVファイルにサンプリング点を保存
curve_csv_file = "bezier_curve_points.csv"
with open(curve_csv_file, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["x", "y"])  # ヘッダー
    writer.writerows(sampled_curve_points)

# CSVファイルに制御点を保存
control_points_csv_file = "control_points.csv"
with open(control_points_csv_file, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["x", "y"])  # ヘッダー
    writer.writerows(control_points)

print(f"サンプリング点をCSVファイルに保存しました: {curve_csv_file}")
print(f"制御点をCSVファイルに保存しました: {control_points_csv_file}")

# 可視化用データを準備
curve_points_x = [p[0] for p in sampled_curve_points]
curve_points_y = [p[1] for p in sampled_curve_points]
control_points_x = [p[0] for p in control_points]
control_points_y = [p[1] for p in control_points]

# 可視化
plt.figure(figsize=(8, 6))

# 曲線を描画
plt.plot(curve_points_x, curve_points_y, label="Bezier Curve", color="blue")

# 制御点を描画
plt.scatter(control_points_x, control_points_y, color="red", label="Control Points")
#plt.plot(control_points_x, control_points_y, '--', color="gray", label="Control Polygon")

# グラフ設定
plt.title("Bezier Curve with Control Points")
plt.xlabel("X")
plt.ylabel("Y")
plt.legend()
plt.grid()
plt.axis('equal')
plt.show()
