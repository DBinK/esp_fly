'''
实验名称：点亮LED蓝灯
版本：v1.0
'''

from machine import Pin,PWM #导入Pin模块

# int3 = Pin(16,Pin.OUT) 
# int3.value(1)


in3 = PWM(Pin(16,Pin.OUT), freq=500, duty=500)


in4 = Pin(17,Pin.OUT, value=0) 
in4.value(0)

time.sleep(3)

in3.duty(0)