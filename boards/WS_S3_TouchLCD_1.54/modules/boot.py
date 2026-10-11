import time

import machine
from machine import Pin

# 电源控制
BAT_EN_PIN = 2
KEY_PWR_PIN = 5

bat_en = Pin(BAT_EN_PIN, Pin.OUT, value=1)  # 开机状态下保持高电平

key_pwr = Pin(KEY_PWR_PIN, Pin.IN)
pwr_ticks = False


def pwr_handle(_):
    global pwr_ticks

    if key_pwr.value() == 0:
        pwr_ticks = time.ticks_ms()
    elif pwr_ticks:
        if time.ticks_diff(time.ticks_ms(), pwr_ticks) < 1000:
            machine.reset()  # 短按重置
        else:
            bat_en.value(0)  # 长按关闭电池


key_pwr.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=pwr_handle)
