import os
import time

from cst8xx import Cst8xx
from machine import I2C, SPI, Pin, SDCard
from qmi8658 import QMI8658, QMI8658_GYR_RANGE_512DPS
from st77xx import St7789

SD_POINT = "/sd"

_sd_point = SD_POINT
_sd_ready = False

sd = None
i2c = None
imu = None
display = None
touch = None

button_key = button_plus = None
button_boot = button_minus = None


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
    # 初始化按钮
    # ============================================================
    global button_key, button_boot, button_plus, button_minus
    button_key = button_plus = Pin(4, Pin.IN, Pin.PULL_UP)
    button_boot = button_minus = Pin(0, Pin.IN, Pin.PULL_UP)

    # ============================================================
    # 初始化显示与触摸
    # ============================================================
    global display, touch
    try:
        spi = SPI(1, baudrate=80_000_000, sck=Pin(38), mosi=Pin(39))
        display = St7789(spi=spi, res=(240, 240), rst=40, dc=45, cs=21, bl=46, rot=0)
        touch = Cst8xx(
            i2c_dev=0, scl=41, sda=42, rst_pin=47, irq_pin=48, width=240, height=240
        )
    except Exception as e:
        print("Display or Touch initialization failed", e)
    time.sleep_ms(5)

    # ============================================================
    # 初始化 IMU
    # ============================================================
    global i2c, imu
    i2c = I2C(0, scl=Pin(41), sda=Pin(42))
    try:
        imu = QMI8658(i2c, gyr_range=QMI8658_GYR_RANGE_512DPS)
    except Exception as e:
        imu = None
        print("IMU initialization failed:", e)
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
