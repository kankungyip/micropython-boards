import time

import machine
from machine import Pin

bat_en = Pin(2, Pin.OUT, value=1)  # 开机状态下保持高电平

btn_pwr = Pin(5, Pin.IN)
pwr_ticks = False


# 电源控制
def pwr_handle(_):
    global pwr_ticks

    if btn_pwr.value() == 0:
        pwr_ticks = time.ticks_ms()
    elif pwr_ticks:
        if time.ticks_diff(time.ticks_ms(), pwr_ticks) < 1000:
            machine.reset()  # 短按重置
        else:
            bat_en.value(0)  # 长按关闭电池


btn_pwr.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=pwr_handle)
