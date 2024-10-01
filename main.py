import time
import network
import espnow

from machine import Pin, PWM # type: ignore

in1 = PWM(Pin(5,Pin.OUT), freq=100000, duty=0)
in2 = Pin(6,Pin.OUT, value=0)

in3 = PWM(Pin(9,Pin.OUT), freq=100000, duty=0)
in4 = Pin(10,Pin.OUT, value=0) 

in3.duty(912)

time.sleep(1)
in3.duty(100)

# A WLAN interface must be active to send()/recv()
sta = network.WLAN(network.STA_IF)
sta.active(True)
sta.disconnect()   # Because ESP8266 auto-connects to last Access Point

e = espnow.ESPNow()
e.active(True)

peer = b'\xff\xff\xff\xff\xff\xff'   # MAC address of peer's wifi interface
e.add_peer(peer)  

while True:
    host, msg = e.recv()
    if msg:                          # msg == None if timeout in recv()
        msg = int(msg)
        # print(host, int(msg))

        v_pwm = int(msg / 8191 * 1023)
        print(v_pwm)
        
        in1.duty(v_pwm)
        in3.duty(v_pwm)

        if msg == b'end':
            break

