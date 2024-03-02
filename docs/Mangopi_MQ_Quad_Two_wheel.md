# 芒果派MQ Quad (H616) 小车
双轮差速模型
## 状态
准备中

## 硬件连接
| 连接设备       | name         | PIN | PIN | name      | 连接设备      |
|---------------|---------     |-----|-----|---------- |---------------|
| NC            | 3.3v         | 1   | 2   | 5v        | NC            |
| 电机2使能脚2   | PI8/SDA1     | 3   | 4   | 5v        | NC            |
| 电机2使能脚1   | PI7/SCL1     | 5   | 6   | GND       | NC            |
| 电机1使能脚1   | PI0          | 7   | 8   | TX0       | 调试串口tx     |
| NC            | GND          | 9   | 10  | RX0       | 调试串口rx     |
| 电机1使能脚2   | PH2/TX5/PWM2 | 11  | 12  | PI1       | 雷达速度控制   |
| PWM7          | PH3/RX5/PWM1 | 13  | 14  | GND       | NC            |
| PWM3          | PI13/PWM3    | 15  | 16  | PI14/PWM4 | PWM4          |
| NC            | 3.3v         | 17  | 18  | PH4       | 电机1编码器A相 |
| MOSI          | MOSI         | 19  | 20  | GND       | NC            |
| MISO          | MISO         | 21  | 22  | PI6/RX2   | Lidar RX      |
| SCLK          | SCLK         | 23  | 24  | CS0       | Lcd Cs        |
| NC            | GND          | 25  | 26  | CS1       | Touch Cs      |
| MPU6050 SDA   | PI10/SDA2    | 27  | 28  | PI9/SCL2  | MPU6050 SCL   |
| Lcd Dc        | PI11/PWM1    | 29  | 30  | GND       | NC            |
| PWM2          | PI12/PWM2    | 31  | 32  | PI5/TX2   | TX2           |
| Touch Int     | PI15         | 33  | 34  | GND       | NC            |
| PWM1          | PI2          | 35  | 36  | PH10      | 电机1编码器B相 |
| MPU6050 Int   | PI16         | 37  | 38  | PI4       | 电机2编码器A相 |
| NC            | GND          | 39  | 40  | PI3       | 电机2编码器B相 |

## 编译MQ Quad
```
$ mkdir build
$ cd build
$ cmake -DCMAKE_TOOLCHAIN_FILE=cmake/build_for_mqquad.cmake ..
$ cmake -DCMAKE_TOOLCHAIN_FILE=cmake/build_for_mqquad.cmake -DCMAKE_BUILD_TYPE=Debug ..
```

```
echo 0 > /sys/class/pwm/pwmchip6/export
echo 10000000 > /sys/class/pwm/pwmchip6/pwm0/period
echo 8500000 > /sys/class/pwm/pwmchip6/pwm0/duty_cycle
echo normal > /sys/class/pwm/pwmchip6/pwm0/polarity
echo 1 > /sys/class/pwm/pwmchip6/pwm0/enable
```

## 调试笔记
右电机
```sh
echo 0 > /sys/class/leds/moto1-ena/brightness # 前进 ena=0 enb=1
echo 1 > /sys/class/leds/moto1-enb/brightness
echo 4 > /sys/class/pwm/pwmchip0/export
echo 40000 > /sys/class/pwm/pwmchip0/pwm4/period
echo 30000 > /sys/class/pwm/pwmchip0/pwm4/duty_cycle
echo normal > /sys/class/pwm/pwmchip0/pwm4/polarity
echo 1 > /sys/class/pwm/pwmchip0/pwm4/enable
```

左电机
```sh
echo 0 > /sys/class/leds/moto2-ena/brightness
echo 1 > /sys/class/leds/moto2-enb/brightness
echo 3 > /sys/class/pwm/pwmchip0/export
echo 40000 > /sys/class/pwm/pwmchip0/pwm3/period
echo 30000 > /sys/class/pwm/pwmchip0/pwm3/duty_cycle
echo normal > /sys/class/pwm/pwmchip0/pwm3/polarity
echo 1 > /sys/class/pwm/pwmchip0/pwm3/enable
```