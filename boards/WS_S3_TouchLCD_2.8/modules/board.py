import os

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
    if name == "sd_point":
        if not _sd_ready:
            return None
        try:
            os.stat(_sd_point)
            return _sd_point
        except Exception:
            return None
    elif name == "sd_ready":
        if not _sd_ready:
            return False
        try:
            os.stat(_sd_point)
            _sd_ready = True
        except Exception:
            _sd_ready = False
        return _sd_ready
    raise AttributeError(name)


def init(sd_point=SD_POINT):
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

    # ============================================================
    # 初始化IMU
    # ============================================================
    global imu
    try:
        imu = QMI8658(i2c, int1_pin=13, int2_pin=12)
    except Exception as e:
        print("IMU initialization failed", e)

    # ============================================================
    # 初始化显示与触摸
    # ============================================================
    global display, touch
    try:
        spi = SPI(1, baudrate=80_000_000, sck=Pin(40), mosi=Pin(45), miso=Pin(46))
        display = St7789(spi=spi, res=(240, 320), rst=39, dc=41, cs=42, bl=5, rot=0)
        touch = Cst3xx(
            i2c_dev=1, sda=1, scl=3, rst_pin=2, irq_pin=4, width=240, height=320
        )
    except Exception as e:
        print("Display or Touch initialization failed", e)

    # ============================================================
    # 初始化 SD 卡
    # ============================================================
    sd_init(sd_point)


def sd_init(sd_point=SD_POINT):
    global _sd_ready, _sd_point
    if _sd_ready:
        return

    try:
        os.unmount(_sd_point)
    except Exception:
        pass

    try:
        sd = SDCard(slot=2, sck=Pin(14), mosi=Pin(17), miso=Pin(16), cs=Pin(21))
        if _sd_point != sd_point:
            _sd_point = sd_point
        os.mount(sd, _sd_point)
        _sd_ready = True
    except Exception as e:
        print("SD Card not found", e)
