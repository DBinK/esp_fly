
import time
import math
import math

from machine import Pin, SPI

from lib.icm42688 import ICM42688P

from modules.utils import TimeDiff
import modules.fusion as fusion
from modules.filter import MovingAverageFilter

main_dt = TimeDiff()

spi = SPI(1, sck=Pin(3), mosi=Pin(5), miso=Pin(7))

cs_pin = 9  # 替换为实际的CS引脚编号

imu = ICM42688P(spi, cs_pin)
imu.initialize()

led = Pin(15, Pin.OUT)
led.value(1)

fuse = fusion.Fusion()
mva_filter = MovingAverageFilter(100)

# 初始化yaw角
yaw = 0.0

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

if __name__ == "__main__":

    last_gz = 0.0
    times = 0

    while True: 
        accel_x, accel_y, accel_z = imu.read_accelerometer()
        gyro_x, gyro_y, gyro_z = imu.read_gyroscope()
        # temp = imu.read_temperature()
        
        gyro_z += 0.802612

        gz_err = gyro_z - last_gz
        gz_err_avg = mva_filter.filter(gz_err) # 计算零偏
        times += 1

        # 计算时间差
        dt = main_dt.time_diff() / 1_000_000_000  # 将ns转换为s
        Hz = int(1/dt) if dt > 0.0001 else 0  # 避免第一次的数很大
        
        # 对陀螺仪的z轴数据进行积分以计算yaw角
        yaw += gyro_z * dt

        # print(f"Yaw Angle: {yaw} , dt: {dt:.6f}, Hz: {int(1/dt)}")

        # 计算 pitch 和 roll
        pitch = math.atan2(accel_x, math.sqrt(accel_y**2 + accel_z**2)) * RAD2DEG
        roll  = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2)) * RAD2DEG

        # 打印
        print(f"raw: {yaw=:.2f}, {pitch=:.2f}, {roll=:.2f}, {dt=:.3f}, {Hz=}, {gz_err_avg=:.6f}, {times}")

        # 更新融合数据
        #fuse.update_nomag((accel_x, accel_y, accel_z), (gyro_x, gyro_y, gyro_z))
        #print(f"fuse: {fuse.heading:.2f}, {fuse.pitch:.2f}, {(180-fuse.roll):.2f}")

        #print(f"加速度: X={accel_x:.2f} G, Y={accel_y:.2f} G, Z={accel_z:.2f} G")
        #print(f"角速度: X={gyro_x:.2f} DPS, Y={gyro_y:.2f} DPS, Z={gyro_z:.2f} DPS")
        #print(f"温度: {temp:.2f} C")
        
        #time.sleep(0.01)
        time.sleep(0.001)

        led.value(not led.value())   