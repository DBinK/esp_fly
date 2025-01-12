import time
import network
from machine import Pin

from modules.now_recv import read_espnow
from modules.motion import MotorESC
from modules.utils import map_value, TimeDiff

time.sleep(1)  # 防止点停止按钮后马上再启动导致 Thonny 连接不上

# motor_1 = MotorESC(39)
# motor_2 = MotorESC(37)
# motor_3 = MotorESC(35)
# motor_4 = MotorESC(33)

motor_1 = MotorESC(18)
motor_2 = MotorESC(16) 
motor_3 = MotorESC(21)
motor_4 = MotorESC(17)

loop_dt = TimeDiff()

while True:

    ms = loop_dt.time_diff() / 1_000_000
    Hz = int(1/(ms/1000)) # if ms > 0.0001 else 0

    print(f"循环时间: {ms:.3f}ms, 频率: {Hz}Hz")

    # time.sleep(0.1)
    time.sleep(0.001)

    data , stick_work = read_espnow()

    if data:
        
        if data[6] != 0x0:
            motor_1.reset()
            motor_2.reset()
            motor_3.reset()
            motor_4.reset()

        if stick_work:

            ly = data[1]
            lx = data[2]
            ry = data[4]
            rx = data[3]
            
            # 底盘控制
            _ly = map_value(ly, (0, 255), (-127, 127))  
            _lx = map_value(lx, (0, 255), (-127, 127))  
            _ry = map_value(ry, (0, 255), (-127, 127))  
            _rx = map_value(rx, (0, 255), (-127, 127))

            # print(f"摇杆映射后数据: lx={_lx}, ly={_ly}, rx={_rx}, ry={_ry}")

            _ly *= 0.1
            _lx *= 0.05
            _ry *= 0.08
            _rx *= 0.05
            
            print(f"摇杆缩放后数据: lx={_lx}, ly={_ly}, rx={_rx}, ry={_ry}")

            motor_1.set_thr_relative(_ly)
            motor_2.set_thr_relative(_lx)
            motor_3.set_thr_relative(_ry)
            motor_4.set_thr_relative(_rx)



