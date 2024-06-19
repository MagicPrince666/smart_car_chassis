# 鲁班猫zero w 智能小车

## 底盘模型
阿克曼底盘

## 状态
进行中...

## 硬件连接
| 连接设备       | name          | PIN | PIN | name          | 连接设备       |
|---------------|---------      |-----|-----|----------     |---------------|
| NC            | 3.3v          | 1   | 2   | 5v            | NC            |
| 雷达接收       | GPIO1_A0/RX3  | 3   | 4   | 5v            | NC            |
| NC            | GPIO1_A1/TX3  | 5   | 6   | GND           | NC            |
| 编码器B相      | GPIO1_A2      | 7   | 8   | GPIO2_C5/TX8  | 外置IMU RX     |
| NC            | GND           | 9   | 10  | GPIO2_C6/RX8  | 外置IMU TX     |
| 编码器A相      | GPIO1_A3      | 11  | 12  | GPIO0_C2/PWM3 | Lidar PWM     |
| 遥控接收器SBUS   | GPIO1_A4      | 13  | 14  | GND           | NC           |
| BTS7960 RPWM  | GPIO1_A5      | 15  | 16  | GPIO2_C3      | SR04 Trig2    |
| NC            | 3.3v          | 17  | 18  | GPIO2_C4      | BTS7960 LPWM  |
| MOSI          | GPIO4_C3/MOSI | 19  | 20  | GND           | NC            |
| MISO          | GPIO4_C5/MISO | 21  | 22  | GPIO1_B1      | SR04 Echo2    |
| SCLK          | GPIO4_C2/SCLK | 23  | 24  | GPIO4_C6/CS0  | Lcd Cs        |
| NC            | GND           | 25  | 26  | GPIO1_B3/CS1  | Touch Cs      |
| MPU6050 SDA   | GPIO3_B4/SDA5 | 27  | 28  | GPIO3_B3/SCL5 | MPU6050 SCL   |
| SR04 Echo1    | GPIO1_A7      | 29  | 30  | GND           | NC            |
| SR04 Trig1    | GPIO1_B0      | 31  | 32  | GPIO3_B6/PWM11| 舵机PWM       |
| Lcd Bl        | GPIO3_B1/PWM8 | 33  | 34  | GND           | NC            |
| BTS7960 EN    | GPIO3_B2/PWM9 | 35  | 36  | GPIO3_A5      | VL5310 Int    |
| LCD reset     | GPIO1_B1      | 37  | 38  | GPIO3_A6      | Lcd Dc        |
| NC            | GND           | 39  | 40  | GPIO3_A7      | Touch Int     |

## BTS7960驱动说明

VCC接单片机的3.3V电源，GND接单片机的GND。

REN与LEN短路并接输入PWM信号调速。

LPWM，脚输入3.3V电平电机正转。

RPWM，脚输入3.3V电平电机反转。

## 编译Lubancat
```bash
colcon build
```

## 设备树配置
开发需要更新设备树配置

[LubanCat Zero W devicetree](docs/rk3566-lubancat-0.dts)

## 已完成
1. 遥控接收机sbus解码
2. 陀螺仪数据获取
3. 电机正反转IO控制
4. 超声波传感器数据获取
5. 通过配置文件（yaml）适配硬件，与软件代码无关
6. 带码盘电机
7. 激光雷达电机速度控制
8. 激光雷达数据获取
9. 电机支持PID控制

## TO DO
1. 图像识别算法引入
2. opencl加速点云计算

## 网络连接
```bash
sudo nmcli dev wifi connect (WIFI名) password (密码)
```