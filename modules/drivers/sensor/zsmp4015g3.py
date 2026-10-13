"""
ZSMP4015G3 环境光与接近传感器 MicroPython 驱动。

芯片: ZSMP4015G3 (数字环境光传感器 + 接近传感器，带 IR VCSEL)
通信: I2C, 7 位地址 0x38
供电: 1.7V ~ 3.6V
目标固件: MicroPython v1.19+ (ESP32 / RP2040 / STM32)
依赖: machine, time, micropython

典型用法:
    from machine import I2C, Pin
    from zsmp4015g3 import ZSMP4015G3

    i2c = I2C(0, scl=Pin(43), sda=Pin(44), freq=400000)
    sensor = ZSMP4015G3(i2c)
    sensor.set_ps_preset('desktop')   # ★ 场景预设
    while True:
        print("ALS=%d PS=%d" % (sensor.read_als(), sensor.read_ps()))
"""

import time

from machine import Pin
from micropython import const

# ---------------------------------------------------------------------------
# 寄存器地址
# ---------------------------------------------------------------------------
_REG_SYSM_CTRL = const(0x00)
_REG_INT_CTRL = const(0x01)
_REG_INT_FLAG = const(0x02)
_REG_WAIT_TIME = const(0x03)
_REG_ALS_GAIN = const(0x04)
_REG_ALS_TIME = const(0x05)
_REG_PS_GAIN = const(0x06)
_REG_LED_CTRL = const(0x07)
_REG_PS_CFG0 = const(0x08)
_REG_PS_TIME = const(0x09)
_REG_PS_FILTER = const(0x0A)
_REG_PERSISTENCE = const(0x0B)
_REG_ALS_THRES_L = const(0x0C)
_REG_ALS_THRES_H = const(0x0E)
_REG_PS_THRES_L = const(0x10)
_REG_PS_THRES_H = const(0x12)
_REG_PS_OFFSET = const(0x14)
_REG_INT_SOURCE = const(0x16)
_REG_ERROR_FLAG = const(0x17)
_REG_PS_DATA_L = const(0x1A)
_REG_ALS_DATA_L = const(0x1C)
_REG_CLR_DATA_L = const(0x1E)
_REG_HIGH_DATA_L = const(0x20)
_REG_CALIB_STAT = const(0x25)
_REG_CALIB_TARG = const(0x27)
_REG_CALIB_CTRL = const(0x29)
_REG_AUTO_CDAT = const(0x2B)
_REG_MANU_CRDAT = const(0x2C)
_REG_AUTO_CRDAT = const(0x2D)
_REG_PS_FILTER_THRES = const(0x2E)
_REG_PROD_ID_L = const(0xBC)
_REG_PROD_ID_H = const(0xBD)

# 芯片身份与默认地址（模块级，勿放类内）
_ZSMP4015G3_I2C_ADDR_DEFAULT = const(0x38)
_ZSMP4015G3_PROD_ID_L = const(0x11)
_ZSMP4015G3_PROD_ID_H = const(0x45)

# ---------------------------------------------------------------------------
# 数据表
# ---------------------------------------------------------------------------
_ALS_GAIN_TABLE = (
    ("x1", 0x00),
    ("x4", 0x01),
    ("x20", 0x02),
    ("x100", 0x04),
    ("x200", 0x08),
    ("x400", 0x10),
)

_ALS_INT_TIME_TABLE = (
    ("1T", 0x00, 1023),
    ("4T", 0x01, 4095),
    ("16T", 0x02, 16383),
    ("64T", 0x03, 65535),
)

_PS_PGA_TABLE = (
    ("x1", 0x00),
    ("x2", 0x01),
    ("x4", 0x02),
    ("x8", 0x04),
    ("x16", 0x08),
    ("x32", 0x10),
    ("x64", 0x20),
)

_PS_PGAE_TABLE = (
    ("x1", 0x00),
    ("x2", 0x01),
    ("x4", 0x02),
)

# PS 场景预设表
# 元组: (名称, PGA_PS, PGAE_PS, IRDR_SEL, TYPE_SEL, PSCONV)
#   PGA_PS   编码见 _PS_PGA_TABLE
#   PGAE_PS  编码见 _PS_PGAE_TABLE
#   IRDR_SEL 0~15, 见数据手册 LED 电流表
#   TYPE_SEL 0=SEL1, 1=SEL2
#   PSCONV   0~63, 脉冲计数单位
_PS_PRESETS = (
    # name           PGA_PS  PGAE_PS IRDR_SEL TYPE_SEL PSCONV
    ("default", 0x00, 0x00, 0x00, 0, 0x00),  # 增益x1, 5.5mA, 1脉冲
    ("desktop", 0x04, 0x00, 0x05, 0, 0x0F),  # 增益x8, 27.5mA, 16脉冲
    ("phone", 0x08, 0x01, 0x0A, 0, 0x1F),  # 增益x32, 50mA, 32脉冲
    ("long_range", 0x20, 0x02, 0x0F, 1, 0x3F),  # 增益x256, 最大, 64脉冲
)


class ZSMP4015G3:
    """
    ZSMP4015G3 环境光与接近传感器驱动。

    中断清除方式：写 0 到 INT_FLAG 寄存器的对应位（写清除）。
    数据字节序：所有 16 位数据均为小端（低字节在前）。

    PS 场景预设:
        构造后默认灵敏度为最低档 (增益 x1, LED 5.5mA, 1 脉冲)，
        桌面 5~15cm 场景下 PS 读数可能一直为 0。
        调用 set_ps_preset('desktop') 可切换到常用场景。
    """

    __slots__ = (
        "_i2c",
        "_addr",
        "_int_pin",
        "_int_pin_obj",
        "_last_error",
        "_error_handler",
        "_int_callbacks",
        "_pending",
        "_deinit",
        "_buf",
    )

    def __init__(
        self, i2c, addr=_ZSMP4015G3_I2C_ADDR_DEFAULT, int_pin=None, error_handler=None
    ):
        """
        Args:
            i2c: machine.I2C 对象，已初始化。
            addr (int): 7 位 I2C 地址，默认 0x38。
            int_pin: 中断引脚，可为 machine.Pin 对象、整数引脚号或 None。
            error_handler: 可选，异常处理回调，签名 fn(exc)。
        Raises:
            OSError: 设备未找到或 ID 不匹配。
        """
        self._i2c = i2c
        self._addr = addr
        self._int_pin = int_pin
        self._int_pin_obj = None
        self._last_error = None
        self._error_handler = error_handler
        self._int_callbacks = []
        self._pending = False

        self._buf = bytearray(2)

        prod_id_l = self._read_u8(_REG_PROD_ID_L)
        prod_id_h = self._read_u8(_REG_PROD_ID_H)
        if prod_id_l != _ZSMP4015G3_PROD_ID_L or prod_id_h != _ZSMP4015G3_PROD_ID_H:
            raise OSError("ZSMP4015G3 not found at I2C address 0x%02X" % addr)

        self._init_sensor()

    # ------------------------------------------------------------------
    # 内部辅助
    # ------------------------------------------------------------------
    def _read_u8(self, reg):
        return self._i2c.readfrom_mem(self._addr, reg, 1)[0]

    def _write_u8(self, reg, val):
        self._i2c.writeto_mem(self._addr, reg, bytes([val]))

    def _read_u16(self, reg_low):
        self._i2c.readfrom_mem_into(self._addr, reg_low, self._buf)
        return self._buf[0] | (self._buf[1] << 8)

    def _check_deinit(self):
        """设备已释放则抛 RuntimeError。"""
        if self._i2c is None:
            raise RuntimeError("device is deinitialized")

    def _call_error_handler(self, exc):
        self._last_error = exc
        handler = self._error_handler
        if handler is not None:
            try:
                handler(exc)
            except Exception as nested:
                self._last_error = nested

    # ------------------------------------------------------------------
    # 初始化
    # ------------------------------------------------------------------
    def _init_sensor(self):
        """软复位并配置默认参数。"""
        self._check_deinit()
        # 软复位
        self._write_u8(_REG_SYSM_CTRL, 0x01)
        time.sleep_ms(20)
        # ALS 增益 x1，积分时间 64T
        self._write_u8(_REG_ALS_GAIN, 0x00)
        self._write_u8(_REG_ALS_TIME, 0x03)
        # PS 默认灵敏度（可通过 set_ps_preset 覆盖）
        self._write_u8(_REG_PS_GAIN, 0x00)
        self._write_u8(_REG_PS_TIME, 0x00)
        self._write_u8(_REG_LED_CTRL, 0x00)
        # 使能所有 PS 光电二极管
        self._write_u8(_REG_PS_CFG0, 0xF0)
        # 等待时间 1 单位 = 10ms
        self._write_u8(_REG_WAIT_TIME, 0x00)
        # 中断使能: ALS + PS
        self._write_u8(_REG_INT_CTRL, 0x03)
        # Persistence = 1
        self._write_u8(_REG_PERSISTENCE, 0x11)
        # 使能 ALS + PS (0x53 = EN_WAIT|EN_SUNALG|EN_PS|EN_ALS)
        self._write_u8(_REG_SYSM_CTRL, 0x53)
        time.sleep_ms(200)

    def init(self):
        """
        重新初始化传感器。幂等：保留已注册的回调和引脚绑定。
        """
        self._check_deinit()
        self._init_sensor()

    # ------------------------------------------------------------------
    # 属性
    # ------------------------------------------------------------------
    @property
    def last_error(self):
        """最近一次 ISR 或回调中的异常对象，无错误时为 None。"""
        return self._last_error

    # ------------------------------------------------------------------
    # 数据读取
    # ------------------------------------------------------------------
    def read_als(self):
        """读取 ALS 数据，返回 int (0~65535)。"""
        self._check_deinit()
        return self._read_u16(_REG_ALS_DATA_L)

    def read_ps(self):
        """读取 PS 数据，返回 int (0~65535)。"""
        self._check_deinit()
        return self._read_u16(_REG_PS_DATA_L)

    def read_clear(self):
        """读取 Clear 通道数据，返回 int (0~65535)。"""
        self._check_deinit()
        return self._read_u16(_REG_CLR_DATA_L)

    def read_high(self):
        """读取高灵敏度通道数据，返回 int (0~65535)。"""
        self._check_deinit()
        return self._read_u16(_REG_HIGH_DATA_L)

    def read_all(self):
        """读取 ALS、PS、Clear、High，返回 tuple (als, ps, clear, high)。"""
        self._check_deinit()
        return (self.read_als(), self.read_ps(), self.read_clear(), self.read_high())

    # ------------------------------------------------------------------
    # PS 场景预设
    # ------------------------------------------------------------------
    def set_ps_preset(self, name):
        """
        一次性配置 PS 灵敏度（增益 + LED 电流 + 脉冲数）。

        桌面 5~15cm 场景下必须调用，否则 PS 读数为 0。

        Args:
            name (str): 预设名，可选
                'default'    - 增益 x1, LED 5.5mA, 1 脉冲（最省电）
                'desktop'    - 增益 x8, LED 27.5mA, 16 脉冲（5~15cm）
                'phone'      - 增益 x32, LED 50mA, 32 脉冲（0~5cm 贴脸）
                'long_range' - 增益 x256, LED 最大, 64 脉冲（远距离）
        Raises:
            ValueError: 名称无效。
        """
        self._check_deinit()
        for preset in _PS_PRESETS:
            if preset[0] == name:
                _, pga_ps, pgae_ps, irdr_sel, type_sel, pscov = preset
                # PS_GAIN = (PGAE_PS << 5) | PGA_PS
                self._write_u8(_REG_PS_GAIN, ((pgae_ps & 0x07) << 5) | (pga_ps & 0x1F))
                # LED_CTRL = (TYPE_SEL << 4) | IRDR_SEL
                self._write_u8(
                    _REG_LED_CTRL, ((type_sel & 0x01) << 4) | (irdr_sel & 0x0F)
                )
                # PS_TIME = PSCONV
                self._write_u8(_REG_PS_TIME, pscov & 0x3F)
                time.sleep_ms(200)
                return
        raise ValueError(
            "ps_preset must be one of: %s" % ", ".join(p[0] for p in _PS_PRESETS)
        )

    # ------------------------------------------------------------------
    # 单项配置
    # ------------------------------------------------------------------
    def set_als_gain(self, gain):
        """
        设置 ALS 增益。
        Args:
            gain (str): 'x1', 'x4', 'x20', 'x100', 'x200', 'x400'。
        Raises:
            ValueError: 参数无效。
        """
        self._check_deinit()
        for name, val in _ALS_GAIN_TABLE:
            if name == gain:
                self._write_u8(_REG_ALS_GAIN, val)
                return
        raise ValueError(
            "ALS gain must be one of: %s" % ", ".join(n for n, _ in _ALS_GAIN_TABLE)
        )

    def set_als_integration_time(self, it):
        """
        设置 ALS 积分时间。
        Args:
            it (str): '1T', '4T', '16T', '64T'。
        Raises:
            ValueError: 参数无效。
        """
        self._check_deinit()
        for name, val, _ in _ALS_INT_TIME_TABLE:
            if name == it:
                self._write_u8(_REG_ALS_TIME, val)
                return
        raise ValueError(
            "ALS integration time must be one of: %s"
            % ", ".join(n for n, _, _ in _ALS_INT_TIME_TABLE)
        )

    def set_ps_gain(self, pga_ps, pgae_ps="x1"):
        """
        设置 PS 增益（单项）。
        Args:
            pga_ps (str): 'x1','x2','x4','x8','x16','x32','x64'。
            pgae_ps (str): 'x1','x2','x4'。
        Raises:
            ValueError: 参数无效。
        """
        self._check_deinit()
        pga_val = None
        for name, val in _PS_PGA_TABLE:
            if name == pga_ps:
                pga_val = val
                break
        if pga_val is None:
            raise ValueError(
                "PS PGA must be one of: %s" % ", ".join(n for n, _ in _PS_PGA_TABLE)
            )
        pgae_val = None
        for name, val in _PS_PGAE_TABLE:
            if name == pgae_ps:
                pgae_val = val
                break
        if pgae_val is None:
            raise ValueError(
                "PS PGAE must be one of: %s" % ", ".join(n for n, _ in _PS_PGAE_TABLE)
            )
        reg_val = ((pgae_val & 0x07) << 5) | (pga_val & 0x1F)
        self._write_u8(_REG_PS_GAIN, reg_val)

    def set_led_current(self, irdr_sel, type_sel=0):
        """
        设置 LED 驱动电流。
        Args:
            irdr_sel (int): 0~15。
            type_sel (int): 0=SEL1, 1=SEL2。
        Raises:
            ValueError: 参数越界。
        """
        self._check_deinit()
        if not 0 <= irdr_sel <= 15:
            raise ValueError("irdr_sel must be 0~15")
        if type_sel not in (0, 1):
            raise ValueError("type_sel must be 0 or 1")
        self._write_u8(_REG_LED_CTRL, ((type_sel & 0x01) << 4) | (irdr_sel & 0x0F))

    def set_ps_integration(self, pscov):
        """
        设置 PS 脉冲计数（单项）。
        Args:
            pscov (int): 0~63。
        Raises:
            ValueError: 参数越界。
        """
        self._check_deinit()
        if not 0 <= pscov <= 63:
            raise ValueError("pscov must be 0~63")
        self._write_u8(_REG_PS_TIME, pscov)

    def set_als_threshold(self, low, high):
        """设置 ALS 中断阈值，范围 0~65535。"""
        self._check_deinit()
        if not 0 <= low <= 0xFFFF:
            raise ValueError("low must be 0~65535")
        if not 0 <= high <= 0xFFFF:
            raise ValueError("high must be 0~65535")
        self._write_u8(_REG_ALS_THRES_L, low & 0xFF)
        self._write_u8(_REG_ALS_THRES_L + 1, (low >> 8) & 0xFF)
        self._write_u8(_REG_ALS_THRES_H, high & 0xFF)
        self._write_u8(_REG_ALS_THRES_H + 1, (high >> 8) & 0xFF)

    def set_ps_threshold(self, low, high):
        """设置 PS 中断阈值，范围 0~65535。"""
        self._check_deinit()
        if not 0 <= low <= 0xFFFF:
            raise ValueError("low must be 0~65535")
        if not 0 <= high <= 0xFFFF:
            raise ValueError("high must be 0~65535")
        self._write_u8(_REG_PS_THRES_L, low & 0xFF)
        self._write_u8(_REG_PS_THRES_L + 1, (low >> 8) & 0xFF)
        self._write_u8(_REG_PS_THRES_H, high & 0xFF)
        self._write_u8(_REG_PS_THRES_H + 1, (high >> 8) & 0xFF)

    def set_persistence(self, prs_als, prs_ps):
        """
        设置 ALS 和 PS 的 persistence。
        Args:
            prs_als (int): 0~15。
            prs_ps (int): 0~15。
        Raises:
            ValueError: 参数越界。
        """
        self._check_deinit()
        if not 0 <= prs_als <= 15:
            raise ValueError("prs_als must be 0~15")
        if not 0 <= prs_ps <= 15:
            raise ValueError("prs_ps must be 0~15")
        self._write_u8(_REG_PERSISTENCE, ((prs_ps & 0x0F) << 4) | (prs_als & 0x0F))

    # ------------------------------------------------------------------
    # 中断
    # ------------------------------------------------------------------
    def read_interrupt_status(self):
        """读取 INT_FLAG 寄存器，返回 int。"""
        self._check_deinit()
        return self._read_u8(_REG_INT_FLAG)

    def clear_interrupt(self):
        """清除 ALS/PS 中断标志（写清除）。"""
        self._check_deinit()
        flag = self._read_u8(_REG_INT_FLAG)
        self._write_u8(_REG_INT_FLAG, flag & 0xFC)

    def attach_interrupt(self, pin, callback, hard=False):
        """
        绑定中断引脚和回调。
        Args:
            pin: machine.Pin 对象、整数引脚号或 None。
            callback: 回调函数，签名 fn(status)。
            hard (bool): True 硬中断，False 软中断。
        """
        self._check_deinit()
        if isinstance(pin, int):
            pin_obj = Pin(pin, Pin.IN, Pin.PULL_UP)
        elif pin is None:
            pin_obj = None
        else:
            pin_obj = pin
        self._int_pin_obj = pin_obj
        self._int_callbacks.append((callback, hard))
        if pin_obj is not None:
            if hard:

                def _hard_isr(_pin):
                    self._pending = True

                pin_obj.irq(trigger=Pin.IRQ_FALLING, handler=_hard_isr, hard=True)
            else:

                def _soft_isr(_pin):
                    try:
                        status = self.read_interrupt_status()
                        self.clear_interrupt()
                        for cb, h in self._int_callbacks:
                            if not h:
                                try:
                                    cb(status)
                                except Exception as exc:
                                    self._call_error_handler(exc)
                    except Exception as exc:
                        self._call_error_handler(exc)

                pin_obj.irq(trigger=Pin.IRQ_FALLING, handler=_soft_isr, hard=False)

    def detach_interrupt(self):
        """解绑中断引脚和回调。"""
        self._check_deinit()
        if self._int_pin_obj is not None:
            self._int_pin_obj.irq(handler=None)
            self._int_pin_obj = None
        self._int_callbacks = []

    def process_pending(self):
        """硬中断模式下主循环调用，处理挂起的中断。"""
        self._check_deinit()
        if not self._pending:
            return
        self._pending = False
        try:
            status = self.read_interrupt_status()
            self.clear_interrupt()
            for callback, hard in self._int_callbacks:
                if not hard:
                    continue
                try:
                    callback(status)
                except Exception as exc:
                    self._call_error_handler(exc)
        except Exception as exc:
            self._call_error_handler(exc)

    # ------------------------------------------------------------------
    # 资源释放
    # ------------------------------------------------------------------
    def deinit(self):
        """释放资源，幂等。"""
        if self._i2c is None:
            return
        self.detach_interrupt()
        self._i2c = None
