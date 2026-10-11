"""
PCF85063/PCF85063ATL RTC MicroPython 驱动

依赖:
    machine.Pin, micropython.const, time
目标固件:
    MicroPython v1.19+（ESP32 / RP2040 / STM32 / ESP8266）

关键语义:
    - 日期时间: BCD 编码，API 层十进制。
    - 仅做字段范围校验，不校验日历合法性（如 2 月 30 日）。
    - 星期基准 0..6，含义由用户约定，驱动不隐含假设。
    - OS 位（秒寄存器 bit7）: 为 1 表示振荡器曾停走；
      读取后自动清除 OS 位，通过 oscillator_stopped 属性暴露。
    - 闹钟字段: ALARM_DISABLE=0x80 表示不参与匹配；
      其他值超范围时自动钳位到有效范围。
    - 中断清除: AF 标志为写 0 清除（读-改-写 CTRL2）。
    - 软复位后需等待 5ms 才能继续 I2C 操作。
    - 回调签名: fn(rtc)
    - 硬/软中断: hard=True 仅置 _pending，需主循环 process_pending()；
                 hard=False（默认）回调内可安全 I2C。

用法（最小示例）:
    from machine import I2C, Pin
    i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
    rtc = PCF85063(i2c, int_pin=5)
    rtc.set_datetime((2025, 1, 15, 3, 12, 30, 0))
    print(rtc.read_datetime())

用法（闹钟 + 中断）:
    rtc.set_alarm(hour=7, minute=30)
    rtc.attach_interrupt(lambda rtc: print("Alarm!"))
    while True:
        time.sleep_ms(100)

用法（硬中断）:
    rtc.attach_interrupt(handler=on_alarm, hard=True)
    while True:
        rtc.process_pending()
        time.sleep_ms(50)
"""

import time

from machine import Pin
from micropython import const

__all__ = ("PCF85063", "bcd_to_dec", "dec_to_bcd")

DEFAULT_ADDRESS = const(0x51)

REG_CTRL1 = const(0x00)
REG_CTRL2 = const(0x01)
REG_SECONDS = const(0x04)
REG_ALARM_SECONDS = const(0x0B)

# CTRL1 软复位值
CTRL1_SOFT_RESET = const(0x58)

ALARM_DISABLE = const(0x80)  # 闹钟字段禁用标志（bit7=1）
CTRL2_AIE = const(0x80)  # bit7: 闹钟中断使能
CTRL2_AF = const(0x40)  # bit6: 闹钟标志

_SOFT_RESET_DELAY_MS = const(5)
_OS_FLAG_MASK = const(0x80)


def bcd_to_dec(value):
    """BCD → 十进制。"""
    return (value >> 4) * 10 + (value & 0x0F)


def dec_to_bcd(value):
    """十进制 → BCD。"""
    return ((value // 10) << 4) | (value % 10)


def _encode_alarm_field(value, low, high):
    """
    闹钟字段编码: 禁用标志直通，否则钳位 [low, high] 并转 BCD，清 bit7。
    """
    if value == ALARM_DISABLE:
        return ALARM_DISABLE
    if value < low:
        value = low
    elif value > high:
        value = high
    return dec_to_bcd(value) & 0x7F


class PCF85063:
    """
    PCF85063/PCF85063ATL 低功耗 RTC 驱动类。

    属性:
        oscillator_stopped (bool): 最近一次读取时振荡器是否曾停过。
        last_error (Exception | None): 最近一次中断/后台错误。
    """

    __slots__ = (
        "_i2c",
        "_addr",
        "_year_base",
        "_int_pin",
        "_int_handler",
        "_pending",
        "_oscillator_stopped",
        "_last_error",
        "_error_handler",
        "_retries",
        "_retry_delay_ms",
        "_deinit",
        "_time_fn",
        "_ticks_diff_fn",
        "_buf1",
    )

    def __init__(
        self,
        i2c,
        address=None,
        year_base=1970,
        int_pin=None,
        error_handler=None,
        retries=2,
        retry_delay_ms=5,
        time_fn=time.ticks_ms,
        ticks_diff_fn=time.ticks_diff,
    ):
        """
        Args:
            i2c (I2C): I2C 实例（外部注入，不可为 None）。
            address (int | None): I2C 地址，默认 0x51，范围 0x00..0x7F。
            year_base (int): 年份基准，寄存器存 (year - year_base)，0..99。
            int_pin (int | Pin | None): INT 引脚，归一化为 IN + PULL_UP。
            error_handler (callable | None): ISR 异常回调，签名 fn(exc)。
            retries (int): I2C 重试次数，默认 2（总共尝试 retries+1 次）。
            retry_delay_ms (int): 重试间隔 ms，默认 5。
            time_fn (callable): 时间源，默认 time.ticks_ms。
            ticks_diff_fn (callable): 时间差函数，默认 time.ticks_diff。

        Raises:
            ValueError: 参数非法。
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
        self._oscillator_stopped = False
        self._last_error = None
        self._error_handler = error_handler
        self._retries = retries
        self._retry_delay_ms = retry_delay_ms
        self._deinit = False
        self._time_fn = time_fn
        self._ticks_diff_fn = ticks_diff_fn
        self._buf1 = bytearray(1)

    # ---------------- 只读属性 ----------------
    @property
    def oscillator_stopped(self):
        """最近一次读取时振荡器是否曾停过。"""
        return self._oscillator_stopped

    @property
    def last_error(self):
        """最近一次中断/后台错误。"""
        return self._last_error

    # ---------------- 内部工具 ----------------
    def _check_deinit(self):
        if self._deinit:
            raise RuntimeError("device is deinitialized")

    def _read_reg(self, reg, nbytes):
        """读 nbytes 字节；复用内部缓冲区；带重试。"""
        self._check_deinit()
        if nbytes == 1:
            buf = self._buf1
        else:
            buf = bytearray(nbytes)
        last_err = None
        for attempt in range(self._retries + 1):
            try:
                self._i2c.readfrom_mem_into(self._addr, reg, buf)
                return buf
            except OSError as exc:
                last_err = exc
                if attempt < self._retries:
                    time.sleep_ms(self._retry_delay_ms)
        raise last_err

    def _write_reg(self, reg, data):
        """写寄存器；带重试。"""
        self._check_deinit()
        last_err = None
        for attempt in range(self._retries + 1):
            try:
                self._i2c.writeto_mem(self._addr, reg, data)
                return
            except OSError as exc:
                last_err = exc
                if attempt < self._retries:
                    time.sleep_ms(self._retry_delay_ms)
        raise last_err

    @staticmethod
    def _normalize_pin(pin):
        """
        归一化为 machine.Pin 输入引脚（IN + PULL_UP）。
        None -> None; int -> Pin(num, IN, PULL_UP); Pin -> 重配。
        """
        if pin is None:
            return None
        if isinstance(pin, int):
            return Pin(pin, Pin.IN, Pin.PULL_UP)
        pin.init(Pin.IN, Pin.PULL_UP)
        return pin

    def _call_error_handler(self, exc):
        """保存并分发错误。"""
        self._last_error = exc
        handler = self._error_handler
        if handler is not None:
            try:
                handler(exc)
            except Exception as nested:
                self._last_error = nested

    # ---------------- 时间 API ----------------
    def datetime(self, value=None):
        """
        兼容 machine.RTC 接口。

        Args:
            value (tuple | None): None → 读；否则设置。

        Returns:
            tuple | None: value=None 返回 (y, m, d, wd, h, mi, s)；
                          value!=None 设置后返回 None。
        """
        if value is None:
            return self.read_datetime()
        self.set_datetime(value)
        return None

    def set_datetime(self, value):
        """
        设置日期时间。

        Args:
            value (tuple): (year, month, day, weekday, hour, minute, second)

        Raises:
            ValueError: 参数长度不足或字段超出范围。

        Notes:
            仅做字段范围校验，不校验日历合法性（如 2 月 30 日）。
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
            OS 位为 1 时 oscillator_stopped 置 True，并自动清 OS 位。
        """
        b = self._read_reg(REG_SECONDS, 7)
        if b[0] & _OS_FLAG_MASK:
            self._oscillator_stopped = True
            self._write_reg(REG_SECONDS, bytes((b[0] & 0x7F,)))
        else:
            self._oscillator_stopped = False
        return (
            bcd_to_dec(b[6]) + self._year_base,
            bcd_to_dec(b[5] & 0x1F),
            bcd_to_dec(b[3] & 0x3F),
            bcd_to_dec(b[4] & 0x07),
            bcd_to_dec(b[2] & 0x3F),
            bcd_to_dec(b[1] & 0x7F),
            bcd_to_dec(b[0] & 0x7F),
        )

    # ---------------- 闹钟 API ----------------
    def set_alarm(
        self,
        second=ALARM_DISABLE,
        minute=ALARM_DISABLE,
        hour=ALARM_DISABLE,
        day=ALARM_DISABLE,
        weekday=ALARM_DISABLE,
    ):
        """
        配置闹钟。未指定字段保持 ALARM_DISABLE(0x80)，不参与匹配；
        超范围字段自动钳位到有效范围。

        Args:
            second (int): 0..59 或 ALARM_DISABLE。
            minute (int): 0..59 或 ALARM_DISABLE。
            hour (int): 0..23 或 ALARM_DISABLE。
            day (int): 1..31 或 ALARM_DISABLE。
            weekday (int): 0..6 或 ALARM_DISABLE。
        """
        data = bytes(
            (
                _encode_alarm_field(second, 0, 59),
                _encode_alarm_field(minute, 0, 59),
                _encode_alarm_field(hour, 0, 23),
                _encode_alarm_field(day, 1, 31),
                _encode_alarm_field(weekday, 0, 6),
            )
        )
        self._write_reg(REG_ALARM_SECONDS, data)

    def enable_alarm_interrupt(self, enable=True):
        """
        [硬件层开关] 使能/禁止 AIE，使能时清 AF 避免历史中断立即触发。

        Notes:
            通常无需手动调用，attach_interrupt() 绑定时自动使能。
            如需在运行时暂停/恢复中断输出，可单独调用本方法。
        """
        v = self._read_reg(REG_CTRL2, 1)[0]
        if enable:
            v = (v | CTRL2_AIE) & ~CTRL2_AF
        else:
            v &= ~CTRL2_AIE
        self._write_reg(REG_CTRL2, bytes((v,)))

    def alarm_triggered(self):
        """轮询模式: 检查 AF 是否置位。"""
        return bool(self._read_reg(REG_CTRL2, 1)[0] & CTRL2_AF)

    def clear_alarm_flag(self):
        """清除 AF 标志（释放 INT 引脚）。"""
        v = self._read_reg(REG_CTRL2, 1)[0] & ~CTRL2_AF
        self._write_reg(REG_CTRL2, bytes((v,)))

    def clear_interrupt(self):
        """[统一接口] 清除中断标志，等价 clear_alarm_flag()。"""
        self.clear_alarm_flag()

    # ---------------- 中断 API ----------------
    def attach_interrupt(
        self, handler=None, trigger=None, hard=False, auto_enable=True
    ):
        """
        在构造时配置的 INT 引脚注册回调。

        Args:
            handler (callable | None): fn(rtc)
            trigger: 触发沿，默认 Pin.IRQ_FALLING（INT 开漏低有效）。
            hard (bool): True 硬中断（仅置 _pending，需 process_pending()）；
                         False 软中断（默认，回调内可安全操作）。
            auto_enable (bool): True(默认) 自动 enable_alarm_interrupt(True)。

        Returns:
            bool: 成功返回 True；构造时 int_pin=None 返回 False。

        Notes:
            - 回调签名：fn(rtc)
            - 硬中断模式需主循环调用 process_pending()。
        """
        self._check_deinit()
        if self._int_pin is None:
            return False
        if trigger is None:
            trigger = Pin.IRQ_FALLING

        if auto_enable:
            try:
                self.enable_alarm_interrupt(True)
            except Exception as exc:
                self._call_error_handler(exc)

        self._int_handler = handler

        if hard:

            def _isr(_pin):
                self._pending = True

            self._int_pin.irq(trigger=trigger, handler=_isr, hard=True)
        else:

            def _isr(_pin):
                try:
                    self.clear_interrupt()
                except Exception as exc:
                    self._call_error_handler(exc)
                if handler is not None:
                    try:
                        handler(self)
                    except Exception as exc:
                        self._call_error_handler(exc)

            self._int_pin.irq(trigger=trigger, handler=_isr, hard=False)
        return True

    def detach_interrupt(self):
        """
        解除 INT 引脚中断绑定及用户回调；未绑定时为空操作。

        Notes:
            本方法不关闭 CTRL2.AIE。如需关闭请调用 enable_alarm_interrupt(False)。
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
        硬中断模式下主循环调用。

        Returns:
            bool: 本次是否处理了中断。
        """
        self._check_deinit()
        if not self._pending:
            return False
        self._pending = False
        try:
            self.clear_interrupt()
        except Exception as exc:
            self._call_error_handler(exc)
        if self._int_handler is not None:
            try:
                self._int_handler(self)
            except Exception as exc:
                self._call_error_handler(exc)
        return True

    # ---------------- 其他 ----------------
    def reset(self):
        """
        软件复位: 写 CTRL1 = 0x58，所有寄存器恢复默认值。
        复位为异步过程，等待 5ms 后再允许后续 I2C 操作。
        """
        self._check_deinit()
        self._write_reg(REG_CTRL1, bytes((CTRL1_SOFT_RESET,)))
        time.sleep_ms(_SOFT_RESET_DELAY_MS)

    def deinit(self):
        """
        释放资源: 解绑中断 → 关闭 AIE → 清空硬件引用。幂等。

        Notes:
            deinit 后调用任何 I2C 方法抛 RuntimeError。
        """
        if self._deinit:
            return
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
        self._deinit = True
