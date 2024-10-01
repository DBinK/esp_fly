'''
实验名称：PWM
版本：v1.0
作者：WalnutPi
说明：通过不同频率的PWM信号输出，驱动无源蜂鸣器发出不同频率的声音。
'''

from machine import Pin, PWM # type: ignore
import time


in3 = PWM(Pin(16,Pin.OUT), freq=50, duty=0)

in4 = Pin(17,Pin.OUT, value=0) 

in3.duty(912)
time.sleep(1)

for i in range(10,800,10):
    print(i)
    in3.duty(i)
    time.sleep(0.1)

for i in range(800,10,-10):
    print(i)
    in3.duty(i)
    time.sleep(0.3)
    
in3.duty(1)