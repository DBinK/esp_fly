import time
import network
from machine import Pin

from modules.now_recv import read_espnow
from modules.motion import MotorESC
from modules.utils import map_value

time.sleep(1)  # 防止点停止按钮后马上再启动导致 Thonny 连接不上

# motor_1 = MotorESC(39)
# motor_2 = MotorESC(37)
# motor_3 = MotorESC(35)
# motor_4 = MotorESC(33)

motor_1 = MotorESC(18)
motor_2 = MotorESC(16)
motor_3 = MotorESC(21)
motor_4 = MotorESC(17)

# 初始化 LED
led = Pin(15, Pin.OUT, value=1)

DEAD_AREA = 20  # 摇杆死区
MAP_COEFF = 58  # 摇杆映射系数 (根据实际需求调整)

while True:

    time.sleep(0.001)

    data = read_espnow()

    if data:

        lx = data[1]  
        ly = data[2]
        rx = data[3]
        ry = data[4]
        other = data[6]

        if other != 0:
            motor_1.reset()
            motor_2.reset()
            motor_3.reset()
            motor_4.reset()
            
            led.value(not led.value())  # 闪烁led
            continue
        

        # print(f"原始数据: lx={lx}, ly={ly}, rx={rx}, ry={ry}")

        lx += 16
        ly += 35
        rx += 6
        ry += 16

        print(f"矫正后数据: lx={lx}, ly={ly}, rx={rx}, ry={ry}")

        # 检查lx, ly, rx, ry中是否至少有一个绝对值超过设定值
        stick_work = (
               abs(lx-127) > DEAD_AREA
            or abs(ly-127) > DEAD_AREA
            or abs(rx-127) > DEAD_AREA
            or abs(ry-127) > DEAD_AREA
        )

        if stick_work:
            led.value(not led.value())  # 闪烁led

            # 底盘控制
            _ly = map_value(ly, (0, 255), (-127, 127))  if abs(ly-127) > DEAD_AREA else 0
            _lx = map_value(lx, (0, 255), (-127, 127))  if abs(lx-127) > DEAD_AREA else 0
            _ry = map_value(ry, (0, 255), (-127, 127))  if abs(ry-127) > DEAD_AREA else 0
            _rx = map_value(rx, (0, 255), (-127, 127))  if abs(rx-127) > DEAD_AREA else 0

            print(f"摇杆映射后数据: lx={_lx}, ly={_ly}, rx={_rx}, ry={_ry}")

            _ly *= 0.1
            _lx *= 0.05
            _ry *= 0.01
            _rx *= 0.05
            
            print(f"摇杆缩放后数据: lx={_lx}, ly={_ly}, rx={_rx}, ry={_ry}")

            motor_1.set_thr_relative(_ly)
            motor_3.set_thr_relative(_lx)
            motor_2.set_thr_relative(_ry)
            motor_4.set_thr_relative(_rx)

        else:
            led.value(0)