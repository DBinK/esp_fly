import network
import espnow
from machine import Pin,SoftI2C,ADC,Timer, PWM
import time

# A WLAN interface must be active to send()/recv()
sta = network.WLAN(network.STA_IF)  # Or network.AP_IF
sta.active(True)
sta.disconnect()      # For ESP8266

e = espnow.ESPNow()
e.active(True)
peer = b'\xff\xff\xff\xff\xff\xff'  # MAC address of peer's wifi interface
e.add_peer(peer)      # Must add_peer() before send()

e.send(peer, "Starting...")


#构建ADC对象
adc = ADC(Pin(15)) 
adc.atten(ADC.ATTN_11DB) #开启衰减器，测量量程增大到3.3V

def ADC_Test(tim):

    raw_data = adc.read()

    rate = raw_data / 8191
    
    #rate_int = int(rate * 100)

    #in3.duty(v_pwm)
    e.send(peer, f"{raw_data}")
    
    #计算电压值，获得的数据0-4095相当于0-3.3V，（'%.2f'%）表示保留2位小数
    print(f'raw data:{adc.read()} rate: {rate:.2}  voltage:{('%.2f'%(rate*3.3))}V')


#开启定时器
tim = Timer(1)
tim.init(period=200, mode=Timer.PERIODIC, callback=ADC_Test) #周期300ms