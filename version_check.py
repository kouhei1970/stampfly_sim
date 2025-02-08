import platform
import sys

print("Python バージョン:", sys.version)
print("実行環境:", platform.system(), platform.version())

try:
    from vpython import canvas
    print("VPython 実行環境: ローカル Python")
except ImportError:
    print("VPython 実行環境: GlowScript の可能性あり")