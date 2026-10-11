import os
import time

from cst3xx import Cst3xx
from machine import I2C, RTC, SPI, Pin, SDCard
from pcf85063 import PCF85063A
from qmi8658 import QMI8658
from st77xx import St7789

SD_POINT = "/sd"

_sd_point = SD_POINT
_sd_ready = False

sd = None
i2c = None
rtc = None
imu = None
display = None
touch = None


def __getattr__(name):
    global _sd_ready

    if name == "sd_ready":
        if not _sd_ready:
            return False
        try:
            os.stat(_sd_point)
            return True
        except Exception:
            _sd_ready = False
            return False

    if name == "sd_point":
        # 复用 sd_ready 的判定，保证两者状态一致
        return _sd_point if __getattr__("sd_ready") else None

    raise AttributeError(name)


def init(sd_point=SD_POINT):
    # ============================================================
    # 初始化显示与触摸
    # ============================================================
    global display, touch
    try:
        spi = SPI(1, baudrate=80_000_000, sck=Pin(40), mosi=Pin(45), miso=Pin(46))
        display = St7789(spi=spi, res=(240, 320), rst=39, dc=41, cs=42, bl=5, rot=0)
        touch = Cst3xx(
            i2c_dev=1, scl=3, sda=1, rst_pin=2, irq_pin=4, width=240, height=320
        )
    except Exception as e:
        print("Display or Touch initialization failed", e)
    time.sleep_ms(5)

    # ============================================================
    # 初始化 RTC
    # ============================================================
    global i2c, rtc
    i2c = I2C(0, scl=Pin(11), sda=Pin(10))
    try:
        rtc = PCF85063(i2c, int_pin=5)
        sys_rtc = RTC()
        y, mo, d, wd, h, mi, s = rtc.datetime()
        if rtc.oscillator_stopped:
            print("PCF85063 oscillator stopped")
        sys_rtc.datetime((y, mo, d, wd, h, mi, s, 0))
    except Exception as e:
        print("RTC initialization failed", e)
    time.sleep_ms(5)

    # ============================================================
    # 初始化IMU
    # ============================================================
    global imu
    try:
        imu = QMI8658(i2c, int1_pin=13, int2_pin=12)
    except Exception as e:
        print("IMU initialization failed", e)
    time.sleep_ms(5)

    # ============================================================
    # 初始化 SD 卡
    # ============================================================
    sd_init(sd_point)
    time.sleep_ms(5)


def sd_init(sd_point=SD_POINT):
    """尝试挂载 SD 卡。成功返回 True，失败返回 False。"""
    global _sd_ready, _sd_point, sd

    if _sd_ready:
        return

    # 清理可能残留的挂载点
    try:
        os.umount(_sd_point)
        if sd_point != _sd_point:
            os.umount(sd_point)
    except Exception:
        pass

    try:
        sd = SDCard(
            slot=0,
            width=4,
            sck=Pin(16),
            cmd=Pin(15),
            data=(Pin(17), Pin(18), Pin(13), Pin(14)),
        )
        os.mount(sd, sd_point)
        _sd_point = sd_point
        _sd_ready = True
    except Exception as e:
        sd = None
        _sd_ready = False
        print("SD Card not found:", e)
