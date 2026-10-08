"""
PCF85063/PCF85063ATL RTC MicroPython Driver
- INT 引脚支持: 只需传引脚编号或 Pin 对象, 内部自动初始化为输入
- 统一的中断绑定 API: attach_interrupt / detach_interrupt
- attach_interrupt 默认自动使能闹钟中断输出 (CTRL2.AIE), 避免忘开
- 高级场景可单独使用 enable_alarm_interrupt 运行时暂停/恢复中断输出

用法:
    rtc = PCF85063(i2c, int_pin=5)   # 直接传引脚编号
    rtc.set_alarm(hour=7, minute=30)
    rtc.attach_interrupt(on_alarm)   # 自动使能 AIE + 绑定引脚
"""

import time

from machine import Pin
from micropython import const

DEFAULT_ADDRESS = const(0x51)

REG_CTRL1 = const(0x00)
REG_CTRL2 = const(0x01)
REG_SECONDS = const(0x04)
REG_ALARM_SECONDS = const(0x0B)

ALARM_DISABLE = const(0x80)  # 闹钟字段禁用标志（bit7=1）

CTRL2_AIE = const(0x80)  # bit7: 闹钟中断使能
CTRL2_AF = const(0x40)  # bit6: 闹钟标志


def bcd_to_dec(value):
    """BCD → 十进制"""
    return (value >> 4) * 10 + (value & 0x0F)


def dec_to_bcd(value):
    """十进制 → BCD"""
    return ((value // 10) << 4) | (value % 10)


def _alarm_value(value, low, high):
    """闹钟字段处理：禁用标志直通，否则钳位 + 转 BCD 并清除 bit7。"""
    if value == ALARM_DISABLE:
        return ALARM_DISABLE
    if value < low:
        value = low
    elif value > high:
        value = high
    return dec_to_bcd(value) & 0x7F


class PCF85063:
    """
    PCF85063/PCF85063ATL 低功耗 RTC 驱动类（含 INT 引脚支持）。

    属性:
        _i2c                外部注入的 I2C 实例
        _addr               设备 I2C 地址
        _year_base          年份基准偏移
        _int_pin            构造时归一化的 INT Pin（未配置时为 None）
        _int_handler        用户中断回调
        _pending            硬中断模式下的待处理标志
        oscillator_stopped  最近一次读取时振荡器是否曾停过
    """

    __slots__ = (
        "_i2c",
        "_addr",
        "_year_base",
        "_int_pin",
        "_int_handler",
        "_pending",
        "oscillator_stopped",
    )

    def __init__(self, i2c, address=None, year_base=1970, int_pin=None):
        """
        Args:
            i2c (I2C): I2C 实例（外部注入，不可为 None）
            address (int): I2C 地址，默认 0x51
            year_base (int): 年份基准，寄存器存 (实际年份 - year_base)，0..99
            int_pin (Pin | int | None): INT 引脚。
                传 None 表示不使用中断；
                传 int 编号或 machine.Pin 对象时，内部统一归一化为
                Pin.IN + Pin.PULL_UP。
        """
        if i2c is None:
            raise ValueError("i2c must not be None")

        addr = DEFAULT_ADDRESS if address is None else address
        if not isinstance(addr, int) or not (0x00 <= addr <= 0x7F):
            raise ValueError("address must be int in 0..0x7F")

        if not isinstance(year_base, int) or not (0 <= year_base <= 9999):
            raise ValueError("year_base must be int in 0..9999")

        self._i2c = i2c
        self._addr = addr
        self._year_base = year_base
        self._int_pin = self._normalize_pin(int_pin)
        self._int_handler = None
        self._pending = False
        self.oscillator_stopped = False

    # ================================================================
    #  内部工具方法
    # ================================================================
    def _read_reg(self, reg, nbytes, retries=2, delay_ms=5):
        """读取寄存器；先重试 retries 次，末次异常直接向上抛出。"""
        i2c = self._i2c
        addr = self._addr
        buf = bytearray(nbytes)
        for _ in range(retries):
            try:
                i2c.readfrom_mem_into(addr, reg, buf)
                return buf
            except OSError:
                time.sleep_ms(delay_ms)
        i2c.readfrom_mem_into(addr, reg, buf)
        return buf

    def _write_reg(self, reg, data, retries=2, delay_ms=5):
        """写入寄存器；先重试 retries 次，末次异常直接向上抛出。"""
        i2c = self._i2c
        addr = self._addr
        for _ in range(retries):
            try:
                i2c.writeto_mem(addr, reg, data)
                return
            except OSError:
                time.sleep_ms(delay_ms)
        i2c.writeto_mem(addr, reg, data)

    @staticmethod
    def _normalize_pin(pin):
        """
        将参数归一化为 machine.Pin 输入引脚：
          - None         -> None
          - int          -> Pin(num, Pin.IN, Pin.PULL_UP)
          - machine.Pin  -> 强制重新配置为 IN + PULL_UP
        """
        if pin is None:
            return None
        if isinstance(pin, int):
            return Pin(pin, Pin.IN, Pin.PULL_UP)
        try:
            pin.init(Pin.IN, Pin.PULL_UP)
        except Exception:
            pass
        return pin

    # ================================================================
    #  时间 API
    # ================================================================
    def datetime(self, value=None):
        """
        兼容 machine.RTC 接口：
            value=None → 返回 (y, m, d, wd, h, mi, s)
            否则设置，返回 None
        """
        if value is None:
            return self.read_datetime()
        self.set_datetime(value)

    def set_datetime(self, value):
        """
        设置日期时间。

        Args:
            value (tuple): (year, month, day, weekday, hour, minute, second)

        Raises:
            ValueError: 参数长度不足或字段超出范围
        """
        if value is None:
            raise ValueError("value must not be None")
        if len(value) < 7:
            raise ValueError("value must have at least 7 elements")

        year, month, day, weekday, hour, minute, second = value[:7]

        if not (
            1 <= month <= 12
            and 1 <= day <= 31
            and 0 <= weekday <= 6
            and 0 <= hour <= 23
            and 0 <= minute <= 59
            and 0 <= second <= 59
        ):
            raise ValueError("datetime field out of range")

        y = year - self._year_base
        if not (0 <= y <= 99):
            raise ValueError("year offset out of 0..99")

        data = bytes(
            (
                dec_to_bcd(second),
                dec_to_bcd(minute),
                dec_to_bcd(hour),
                dec_to_bcd(day),
                dec_to_bcd(weekday),
                dec_to_bcd(month),
                dec_to_bcd(y),
            )
        )
        self._write_reg(REG_SECONDS, data)

    def read_datetime(self):
        """
        读取当前日期时间。

        Returns:
            tuple: (year, month, day, weekday, hour, minute, second)

        Notes:
            若秒寄存器 OS 位为 1（振荡器曾停走），oscillator_stopped 置 True，
            并自动清 OS 位以便下次独立检测。
        """
        b = self._read_reg(REG_SECONDS, 7)

        if b[0] & 0x80:
            self.oscillator_stopped = True
            self._write_reg(REG_SECONDS, bytes((b[0] & 0x7F,)))
        else:
            self.oscillator_stopped = False

        return (
            bcd_to_dec(b[6]) + self._year_base,  # year
            bcd_to_dec(b[5] & 0x1F),  # month
            bcd_to_dec(b[3] & 0x3F),  # day
            bcd_to_dec(b[4] & 0x07),  # weekday
            bcd_to_dec(b[2] & 0x3F),  # hour
            bcd_to_dec(b[1] & 0x7F),  # minute
            bcd_to_dec(b[0] & 0x7F),  # second
        )

    # ================================================================
    #  闹钟 API
    # ================================================================
    def set_alarm(
        self,
        second=ALARM_DISABLE,
        minute=ALARM_DISABLE,
        hour=ALARM_DISABLE,
        day=ALARM_DISABLE,
        weekday=ALARM_DISABLE,
    ):
        """
        配置闹钟。未指定字段保持 ALARM_DISABLE(0x80)，即不参与匹配；
        超出范围的字段会被自动钳位。
        """
        data = bytes(
            (
                _alarm_value(second, 0, 59),
                _alarm_value(minute, 0, 59),
                _alarm_value(hour, 0, 23),
                _alarm_value(day, 1, 31),
                _alarm_value(weekday, 0, 6),
            )
        )
        self._write_reg(REG_ALARM_SECONDS, data)

    def enable_alarm_interrupt(self, enable=True):
        """
        [硬件层开关] 使能/禁止闹钟中断输出（CTRL2.AIE）。
        使能时同时清除 AF 挂起标志，避免历史中断立即触发。

        Notes:
            通常无需手动调用，attach_interrupt() 会在绑定时自动使能。
            如需在运行时暂停/恢复中断输出，可单独调用本方法。
        """
        v = self._read_reg(REG_CTRL2, 1)[0]
        if enable:
            v = (v | CTRL2_AIE) & ~CTRL2_AF  # AIE=1, AF=0
        else:
            v &= ~CTRL2_AIE  # AIE=0
        self._write_reg(REG_CTRL2, bytes((v,)))

    def alarm_triggered(self):
        """轮询模式：检查 AF 标志是否置位。"""
        return bool(self._read_reg(REG_CTRL2, 1)[0] & CTRL2_AF)

    def clear_alarm_flag(self):
        """清除 AF 标志（释放 INT 引脚）。"""
        v = self._read_reg(REG_CTRL2, 1)[0] & ~CTRL2_AF
        self._write_reg(REG_CTRL2, bytes((v,)))

    # ---------- 清除中断状态的统一别名 ----------
    def clear_interrupt(self):
        """[统一接口] 清除中断标志。等价于 clear_alarm_flag()。"""
        self.clear_alarm_flag()

    # ================================================================
    #  中断 API（与 QMI8658C 统一命名）
    # ================================================================
    def attach_interrupt(
        self, handler=None, trigger=None, hard=False, auto_enable=True
    ):
        """
        [统一接口] 在构造时配置的 INT 引脚上注册回调。

        Args:
            handler (callable): 用户回调，签名 fn(rtc)
            trigger:            触发边沿，默认 Pin.IRQ_FALLING
                                （INT 开漏低电平有效）
            hard (bool):        True  = 硬中断（回调中禁止 I2C/print）
                                False = 软中断（可安全操作，默认）
            auto_enable (bool): True(默认)  绑定时自动调用
                                enable_alarm_interrupt(True)，使 AIE 与回调
                                注册保持一致；
                                False 表示由用户自行管理 AIE，只做引脚绑定。

        Returns:
            bool: 是否成功注册。
                  构造时未配置 int_pin → 静默返回 False，不抛异常。

        Notes:
            - 硬中断模式下，中断仅置 _pending 标志，需在主循环调用
              process_pending() 完成 AF 清除与用户回调。
            - 软中断模式下，回调内会自动清 AF 后再调用用户 handler。
            - 想运行时暂停中断输出时，调用 enable_alarm_interrupt(False)；
              想恢复，调用 enable_alarm_interrupt(True)。
        """
        if self._int_pin is None:
            return False
        if trigger is None:
            trigger = Pin.IRQ_FALLING

        # 自动使能闹钟中断输出
        if auto_enable:
            try:
                self.enable_alarm_interrupt(True)
            except Exception:
                pass

        self._int_handler = handler

        if hard:

            def _isr(p):
                self._pending = True

            self._int_pin.irq(trigger=trigger, handler=_isr, hard=True)
        else:

            def _isr(p):
                try:
                    self.clear_interrupt()
                except Exception:
                    pass
                if handler is not None:
                    handler(self)

            self._int_pin.irq(trigger=trigger, handler=_isr, hard=False)
        return True

    def detach_interrupt(self):
        """
        [统一接口] 解除 INT 引脚中断绑定及用户回调。
        未配置或未绑定时为空操作。

        Notes:
            本方法只解除引脚绑定，不会关闭 CTRL2.AIE。
            如需一并关闭中断输出，请显式调用 enable_alarm_interrupt(False)。
        """
        if self._int_pin is not None:
            try:
                self._int_pin.irq(handler=None)
            except Exception:
                pass
        self._int_handler = None
        self._pending = False

    def process_pending(self):
        """
        硬中断模式下在主循环中调用：
        若 _pending 置位，则清除 AF 并调用用户回调。

        Returns:
            bool: 本次是否处理了中断
        """
        if not self._pending:
            return False
        self._pending = False
        try:
            self.clear_interrupt()
        except Exception:
            pass
        if self._int_handler is not None:
            self._int_handler(self)
        return True

    # ================================================================
    #  其他 API
    # ================================================================
    def reset(self):
        """
        软件复位：写 CTRL1 = 0x58，所有寄存器恢复默认值。
        复位为异步过程，等待 5ms 后再允许后续 I2C 操作。
        """
        self._write_reg(REG_CTRL1, b"\x58")
        time.sleep_ms(5)

    def deinit(self):
        """
        释放资源：解除中断绑定 → 关闭闹钟中断输出 → 清空硬件引用。
        """
        try:
            self.detach_interrupt()
        except Exception:
            pass
        try:
            self.enable_alarm_interrupt(False)
        except Exception:
            pass
        self._i2c = None
        self._addr = None
