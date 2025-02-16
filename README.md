# StampFly Drone Simulator

## Dependency

### VPython
シミュレーションの可視化にVPythonを使用している。
インストール方法は何種類かある

```conda install -c vpython vpython```
 or
 ```conda install -c conda-forge vpython```
 or
```pip install vpython```

### HID
```pip install hid```

### Numpy
```pip install numpy```

### Matplotlib
```pip install matplotlib```

### PlatformIO
ATOM Joystickをシミュレータのコントローラにするためのプログラムをビルドし書き込む環境をインストールする

```
cd
mkdir -p tmp/pio
cd tmp/pio
wget -O get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 get-platformio.py
```

### ATOM JoyStick

ATOMS3をUSBケーブルでPCに接続する. リセットボタンを緑のLEDの点灯を確認するまで長押しする

```
cd
. .platformio/penv/bin/activate
mkdir -p tmp/github
cd tmp/github
git clone https://github.com/kouhei1970/atomjoystick_for_flighte_simulator.git
cd atomjoystick_for_flighte_simulator
pio run --target upload
```

## Install and Run
AtomJoystickはUSBケーブルでPCに接続しておく。
操縦モードはモード３
制御は簡単な角速度制御モードのみ（アクロモード）

```
cd
mkdir -p tmp/github
cd tmp/github
git clone https://github.com/kouhei1970/stampfly_sim.git
cd stampfly_sim
python test_sim.py
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
