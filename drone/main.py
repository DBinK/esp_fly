import time
import network
from machine import Pin

from modules.now_recv import read_espnow, process_data
from modules.motion import MotorController
from modules.utils import map_value, TimeDiff

time.sleep(1)  # 防止点停止按钮后马上再启动导致 Thonny 连接不上

motors = MotorController(18, 16, 21, 17, limit_max_thr=950)

loop_dt = TimeDiff()

while True:

    ms = loop_dt.time_diff() / 1_000_000
    Hz = int(1/(ms/1000)) # if ms > 0.0001 else 0

    print(f"循环时间: {ms:.3f}ms, 频率: {Hz}Hz")

    # time.sleep(0.1)
    time.sleep(0.001)

    data, stick_work = read_espnow()
    data = process_data(data)

    if data:
        
        if data[6] != 0x0:
            motors.reset()

        if stick_work:

            roll_output  = _rx  # 更新滚转输出
            pitch_output = _ry  # 更新俯仰输出
            yaw_output   = _lx  # 更新偏航输出

            z_output     = _ly

            # 综合控制输出
            motor1 = z_output + roll_output + pitch_output + yaw_output  # 电机1输出
            motor2 = z_output - roll_output + pitch_output - yaw_output  # 电机2输出
            motor3 = z_output - roll_output - pitch_output + yaw_output  # 电机3输出
            motor4 = z_output + roll_output - pitch_output - yaw_output  # 电机4输出

            # 设置电机输出
            motors.set_motors_thr([motor1, motor2, motor3, motor4])

"""
*函  数：void Control(FLOAT_ANGLE *att_in,FLOAT_XYZ *gyr_in, RC_TYPE *rc_in, uint8_t armed)
*功  能：姿态控制,角度环控制和角速度环控制
*参  数：att_in：测量值
*        gry_in: MPU6050读取的角速度值
*        rc_in : 遥控器设定值
*        armed记录命令
*返回值：无
*备  注：RoboFly 小四轴机头与电机示意图	
					 机头(Y+)
					   
				  M1    ↑    M2
					\   |   /
					 \  |  /
					  \ | /
			    ————————+————————>X+	
					  / | \
					 /  |  \
					/   |   \
				  M4    |    M3

	
	1. M1 M3电机逆时针旋转, M2 M4电机顺时针旋转
	2. X:是MPU6050的 X 轴, Y:是MPU6050的 Y 轴, Z轴正方向垂直 X-Y 面, 竖直向上
	3. 绕 X 轴旋转为PITCH 角 
	   绕 Y 轴旋转为 ROLL 角 
	   绕 Z 轴旋转为 YAW  角
	4. 自己DIY时进行动力分配可以一个轴一个轴的分配, 切勿三个轴同时分配。"""

