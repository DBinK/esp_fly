import json  # 导入ujson库，用于处理JSON格式
import network
import espnow
from machine import Pin, ADC, Timer
import time

# 初始化 wifi
sta = network.WLAN(network.STA_IF)  # 或者使用 network.AP_IF
sta.active(True)
sta.disconnect()      # 对于 ESP8266

# 初始化 espnow
e = espnow.ESPNow()
e.active(True)
peer = b'\xff\xff\xff\xff\xff\xff'  # 使用广播地址
e.add_peer(peer)      
# e.send(peer, "Starting...")

# 初始化 adc 摇杆输入
lx = ADC(Pin(4)) 
lx.atten(ADC.ATTN_11DB)  # 开启衰减器，测量量程增大到3.3V 

ry = ADC(Pin(16))
ry.atten(ADC.ATTN_11DB)

# 初始化 开关
rotate_sw = False
def switch_callback(pin):
    global rotate_sw
    # 防抖
    time.sleep_ms(100)
    if pin.value() == 0:
        rotate_sw = not rotate_sw
        print(f"开关: {rotate_sw}")

btn = Pin(1, Pin.IN, Pin.PULL_UP)
btn.irq(switch_callback,Pin.IRQ_FALLING)

def main(tim_callback):

    global rotate_sw
    
    if rotate_sw:
        lx_raw  = 8191 - lx.read()  # 前进速度
        lx_rate = lx_raw / 8191

        ry_raw  = 8191 - ry.read() - 3270  # 转向速度
        ry_rate = ry_raw / 8191
        
        # 计算电机速度
        l_motor = (lx_rate * 0.98 + ry_rate * 0.2) * 1023
        r_motor = (lx_rate * 0.98 - ry_rate * 0.2) * 1023

        # 限位 和 化整
        l_motor = int(max(0, min(1023, l_motor)))
        r_motor = int(max(0, min(1023, r_motor)))

        # 使用 JSON 格式发送数据
        data = {"l_motor": l_motor, "r_motor": r_motor}
        e.send(peer, json.dumps(data))  # 将数据转换为 JSON 字符串并发送
        
        print(f'前进速度:{lx_raw}, 转向速度:{ry_raw}, l_motor:{l_motor}, r_motor:{r_motor}')

    else:
        # 使用 JSON 格式发送数据
        data = {"l_motor": 0, "r_motor": 0}
        e.send(peer, json.dumps(data))  # 将数据转换为 JSON 字符串并发送

# 开启定时器
tim = Timer(1)
tim.init(period=20, mode=Timer.PERIODIC, callback=main)  # 周期200ms