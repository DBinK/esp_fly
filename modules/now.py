
import time
import json
import struct

import espnow
import network


# 初始化 WiFi 和 espnow
sta = network.WLAN(network.STA_IF)
sta.active(True)
sta.disconnect()  # 因为 ESP8266 会自动连接到最后一个接入点

now = espnow.ESPNow()
now.active(True)  # 连接dk广播地址
now.add_peer(b"\xff\xff\xff\xff\xff\xff")



def read_espnow():
    """读取espnow数据并进行解包处理"""
    host, msg = now.recv(5)  # 读取所有可用的数据, 参数: 超时时间ms

    #print("espnow数据:", msg)

    if msg:  # 如果没有数据，则返回
        # 假设 msg 是字节串，需要先解码为字符串
        msg_str = msg.decode('utf-8')
        data = json.loads(msg_str)  # 处理接收到的数据
        return data

    else:
        return None


if __name__ == "__main__":
    print("正在读取espnow数据...")
    while True:
        data = read_espnow()

        if data:
            print(data)

        time.sleep(0.1)


    # GamePad 数据格式

    gamepad_data = {
        "id": 0x01,  # 手柄 ID

        "lx": 0,  # 左摇杆X轴 0x00 ~ 0xFF (0~255)
        "ly": 0,  # 左摇杆Y轴 0x00 ~ 0xFF (0~255)
        "rx": 0,  # 右摇杆X轴 0x00 ~ 0xFF (0~255)
        "ry": 0,  # 右摇杆Y轴 0x00 ~ 0xFF (0~255)

        "abxy & dpad": 0,
        "ls & rs & start & back": 0,

        "mode": 0x06,  # 预留模式位
    }