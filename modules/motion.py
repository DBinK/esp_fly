from machine import SoftI2C, Pin, PWM  # type: ignore  # noqa: F401

KEY  = Pin(0,Pin.IN,Pin.PULL_UP)         # 构建KEY对象

lf_bh_back = PWM(Pin(15), freq=50)
lf_bh_go   = PWM(Pin(16), freq=50)

lf_ft_back = PWM(Pin(17), freq=50)
lf_ft_go   = PWM(Pin(18), freq=50)

rt_bh_back = PWM(Pin(21), freq=50)
rt_bh_go   = PWM(Pin(34), freq=50)

rt_ft_back = PWM(Pin(35), freq=50)
rt_ft_go   = PWM(Pin(36), freq=50)


# 四个轮子单独控制函数
def lf_ft(pwm_val):
    if pwm_val > 0:
        lf_ft_go.duty(pwm_val)
        lf_ft_back.duty(0)
    else:
        lf_ft_back.duty(-pwm_val)
        lf_ft_go.duty(0)

def lf_bh(pwm_val):
    if pwm_val > 0:
        lf_bh_go.duty(pwm_val)
        lf_bh_back.duty(0)
    else:
        lf_bh_back.duty(-pwm_val)
        lf_bh_go.duty(0)

def rt_ft(pwm_val):
    if pwm_val > 0:
        rt_ft_go.duty(pwm_val)
        rt_ft_back.duty(0)
    else:
        rt_ft_back.duty(-pwm_val)
        rt_ft_go.duty(0)

def rt_bh(pwm_val):
    if pwm_val > 0:
        rt_bh_go.duty(pwm_val)
        rt_bh_back.duty(0)
    else:
        rt_bh_back.duty(-pwm_val)
        rt_bh_go.duty(0)


# 四个轮子同时控制函数

def move(v_y, v_x, v_w):
    # 四个轮子的速度分解

    v1 = v_y + v_x - v_w
    v2 = v_y - v_x - v_w
    v3 = v_y - v_x + v_w
    v4 = v_y + v_x + v_w

    # 增加限位, 让所有速度不超过(-1023, 1023)
    v1 = max(-1023, min(1023, v1))
    v2 = max(-1023, min(1023, v2))
    v3 = max(-1023, min(1023, v3))
    v4 = max(-1023, min(1023, v4))

    lf_ft(v1)
    lf_bh(v2)

    rt_ft(v3)
    rt_bh(v4)

def go_forward(pwm_val):
    move(pwm_val, 0, 0)

def go_backward(pwm_val):
    move(-pwm_val, 0, 0)

def go_left(pwm_val):
    move(0, -pwm_val, 0)

def go_right(pwm_val):
    move(0, pwm_val, 0)

def turn_left(pwm_val):
    move(0, 0, pwm_val)

def turn_right(pwm_val):
    move(0, 0, -pwm_val)
    
def stop():
    go_forward(0)


