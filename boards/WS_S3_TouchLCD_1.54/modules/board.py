import os

from cst8xx import Cst8xx
from machine import SPI, Pin, SDCard
from st77xx import St7789

SD_POINT = "/sd"

display = None
touch = None
sd = None

_sd_point = SD_POINT
_sd_ready = False


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
    # 初始化显示与触摸
    # ============================================================
    global display, touch
    try:
        spi = SPI(1, baudrate=80_000_000, sck=Pin(38), mosi=Pin(39))
        display = St7789(spi=spi, res=(240, 240), rst=40, dc=45, cs=21, bl=46, rot=0)
        touch = Cst8xx(
            i2c_dev=1, sda=42, scl=41, rst_pin=47, irq_pin=48, width=240, height=240
        )
    except Exception as e:
        print("Display or Touch initialization failed", e)

    # ============================================================
    # 初始化 SD 卡
    # ============================================================
    global _sd_point
    _sd_point = sd_point
    sd_init()


def sd_init():
    if _sd_ready:
        return

    try:
        os.unmount(_sd_point)
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
        os.mount(sd, _sd_point)
        _sd_ready = True
    except Exception as e:
        print("SD Card not found", e)
