from svgpathtools import svg2paths

# SVGファイルを読み込む
svg_file = "stampfly_airfoil.svg"  # SVGファイルのパス
paths, attributes = svg2paths(svg_file)

# ベジエ曲線データを解析
for path in paths:
    for segment in path:
        if segment.__class__.__name__ == 'CubicBezier':
            print("Cubic Bezier Curve:")
            print(f"  Start: {segment.start}")
            print(f"  Control Point 1: {segment.control1}")
            print(f"  Control Point 2: {segment.control2}")
            print(f"  End: {segment.end}")
        elif segment.__class__.__name__ == 'QuadraticBezier':
            print("Quadratic Bezier Curve:")
            print(f"  Start: {segment.start}")
            print(f"  Control Point: {segment.control}")
            print(f"  End: {segment.end}")
        else:
            print(f"Other Segment: {segment}")
