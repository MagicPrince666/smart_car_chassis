# 芒果派MQ Quad (H616) 小车

## 状态
准备中

## 硬件连接
| 连接设备       | name      | PIN | PIN | name     | 连接设备       |
|---------------|---------  |-----|-----|----------|---------------|
| NC            | 3.3v      | 1   | 2   | 5v       | NC            |
| MPU6050 INT   | PI8/SDA1  | 3   | 4   | 5v       | NC            |
| 编码器B相      | PI7/SCL1  | 5   | 6   | GND      | NC            |
| 编码器A相      | PI0       | 7   | 8   | TX0      | 调试串口tx     |
| NC            | GND       | 9   | 10  | RX0      | 调试串口rx     |
| NC            | PH2/TX5   | 11  | 12  | PI1      | BTS7960 RPWM  |
| 遥控接收器SBUS | PH3/RX5   | 13  | 14  | GND      | NC            |
| 舵机PWM       | PI13/PWM3 | 15  | 16  | PI14/PWM4 | BTS7960 EN    |
| NC            | 3.3v      | 17  | 18  | PH4      | BTS7960 LPWM  |
| MOSI          | MOSI      | 19  | 20  | GND      | NC            |
| MISO          | MISO      | 21  | 22  | PI6/RX2  | Lidar RX      |
| SCLK          | SCLK      | 23  | 24  | CS0      | Lcd Cs        |
| NC            | GND       | 25  | 26  | CS1      | Touch Cs      |
| MPU6050 SDA   | PI10/SDA2 | 27  | 28  | PI9/SCL2 | MPU6050 SCL   |
| Lidar PWM     | PI11/PWM1 | 29  | 30  | GND      | NC            |
| Lcd Bl        | PI12/PWM2 | 31  | 32  | PI5/TX2  | NC            |
| SR04 Echo     | PI15      | 33  | 34  | GND      | NC            |
| SR04 Trig     | PI2       | 35  | 36  | PH10     | Lcd Dc        |
| Touch Int     | PI16      | 37  | 38  | PI4      | NC            |
| NC            | GND       | 39  | 40  | PI3      | VL5310 Int    |

## 编译MQ Quad
```
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=cmake/build_for_mqquad.cmake ..
cmake -DCMAKE_TOOLCHAIN_FILE=cmake/build_for_rk3308.cmake ..
cmake -DCMAKE_TOOLCHAIN_FILE=cmake/build_for_mqquad.cmake -DCMAKE_BUILD_TYPE=Debug ..
```

## 连接wifi
```
nmcli dev wifi connect "OpenWrt_R619ac_2.4G" password "67123236"
```

## 安装证书
```
cd /tmp
wget http://archive.ubuntu.com/ubuntu/pool/main/c/ca-certificates/ca-certificates_20211016ubuntu0.20.04.1_all.deb
dpkg -i ca-certificates_20211016ubuntu0.20.04.1_all.deb
```