"""
    Omni Bot 全向轮 运动控制模块
    by: DBin_K
"""

import time
import math
from machine import Pin, PWM  # type: ignore

from modules.utils import limit_value, map_value


class MotorESC:
    def __init__(
        self,
        pin,                    # PWM 引脚号
        freq      = 400,        # PWM 频率 (最大不超过500Hz,即2000us)
        min_us    = 1000,       # 最小脉宽 us
        max_us    = 2000,       # 最大脉宽 us         
        max_thr   = 1000,       # 可达最大油门 (0~2000)
        min_accu  = 1,          # 最小油门控制精度

        target_thr    = 0,    # 初始化目标油门
        limit_min_thr = 0,    # 最小油门限制
        limit_max_thr = 1000  # 最大油门限制
    ):
        self.pin = pin
        self.pwm = PWM(Pin(pin), freq=freq, duty=0)

        self.freq         = freq          # 频率
        self.min_us       = min_us        # 最小脉宽
        self.max_us       = max_us        # 最大脉宽
        self.max_thr      = max_thr       # 最大油门
        self.min_accu     = min_accu      # 最小精度

        self.limit_max_thr = limit_max_thr  # 最大油门限制
        self.limit_min_thr = limit_min_thr  # 最小油门限制

        self.target_thr = target_thr  # 初始化目标油门
        self.set_thr(target_thr)

    def set_limit(self, limit_min_thr, limit_max_thr):  # 设置油门限制
        self.limit_max_thr = limit_max_thr
        self.limit_min_thr = limit_min_thr

    def set_thr(self, target_thr):  # 绝对油门运动控制

        # print(f"set_thr(): 传入 {self.pin} 号电机的目标油门: {target_thr}")

        target_thr = min(max(target_thr, self.limit_min_thr), self.limit_max_thr) # 限制油门

        print(f"set_thr(): 实际 {self.pin} 号电机可达油门: {target_thr}\n")

        self.target_thr = target_thr

        us = self.min_us + (self.max_us - self.min_us) * (target_thr / self.max_thr)
        ns = int(us * 1000)
        print(f"输入脉宽{us}")

        self.pwm.duty_ns(ns) # 设置 PWM 脉宽

    def get_thr(self):  # 查询当前油门
        return self.target_thr

    def set_thr_relative(self, relative_thr):  # 相对油门运动控制
        print(f"set_thr_relative(): 传入 {self.pin} 号电机的相对油门: {relative_thr}")
        self.target_thr += relative_thr
        self.set_thr(self.target_thr)

    def reset(self, target_thr=0):  # 复位
        self.set_thr(target_thr)
        print("reset(): 复位电机")


if __name__ == "__main__":

    motor_1 = MotorESC(39)
    motor_1.set_thr(200)
    print("set_thr(): 设置目标油门为 200")

    time.sleep(5)

    motor_1.set_thr_relative(100)
    print("set_thr_relative(): 增加目标油门为 100")

    time.sleep(5)

    motor_1.reset()
    print("reset(): 复位电机")