"""
QMI8658C 6轴 IMU MicroPython 驱动

依赖:
    machine.Pin, math, time, micropython.const
目标固件:
    MicroPython v1.19+（ESP32 / RP2040 / STM32 / ESP8266）

坐标系与换算（右手系，具体轴向以芯片安装方向为准）:
    acc_g   = raw / acc_lsb_div     单位 g
    gyr_dps = raw / gyr_lsb_div     单位 dps
    temp_c  = raw / 256.0           单位 ℃

FIFO 协议:
    每样本 14 字节，小端序，字段顺序:
        [T_MSB T_LSB | AX_MSB AX_LSB | AY_MSB AY_LSB | AZ_MSB AZ_LSB |
         GX_MSB GX_LSB | GY_MSB GY_LSB | GZ_MSB GZ_LSB]
    读取后自动复位 FIFO。

中断清除:
    STATUSINT (0x2D) 为读清除。

硬/软中断:
    hard=True  回调仅置 _pending，需主循环 process_pending()
    hard=False 回调内可直接 I2C（默认）

回调签名:
    fn(events: list[str], status: int)

用法（最小示例）:
    from machine import I2C, Pin
    import time
    from qmi8658 import QMI8658
    i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
    imu = QMI8658(i2c, int1_pin=5, int2_pin=4)
    while True:
        acc_g, gyr_dps, temp_c = imu.read()
        print(acc_g, gyr_dps, temp_c)
        time.sleep_ms(200)
"""

import math
import time

from machine import Pin
from micropython import const

__all__ = ("QMI8658",)

# ==================== 寄存器地址 ====================
QMI8658_REG_DEVICE_ID = const(0x00)
QMI8658_REG_CTRL1 = const(0x02)
QMI8658_REG_CTRL2 = const(0x03)
QMI8658_REG_CTRL3 = const(0x04)
QMI8658_REG_CTRL5 = const(0x06)
QMI8658_REG_CTRL7 = const(0x08)
QMI8658_REG_CTRL8 = const(0x09)
QMI8658_REG_CTRL9 = const(0x0A)
QMI8658_REG_CAL1_L = const(0x0B)
QMI8658_REG_CAL1_H = const(0x0C)
QMI8658_REG_CAL3_L = const(0x0F)
QMI8658_REG_CAL3_H = const(0x10)
QMI8658_REG_FIFO_WTM_TH = const(0x13)
QMI8658_REG_FIFO_CTRL = const(0x14)
QMI8658_REG_FIFO_SMPL_CNT = const(0x15)
QMI8658_REG_FIFO_STATUS = const(0x16)
QMI8658_REG_FIFO_DATA = const(0x17)
QMI8658_REG_STATUS_INT = const(0x2D)
QMI8658_REG_STEP_CNT_LOW = const(0x45)
QMI8658_REG_STEP_CNT_MID = const(0x46)
QMI8658_REG_STEP_CNT_HIGH = const(0x47)
QMI8658_REG_TAP_STATUS = const(0x59)
QMI8658_REG_DATA = const(0x33)

QMI8658_CHIP_ID = const(0x05)
QMI8658_DEFAULT_ADDR = const(0x6B)

# ==================== 量程常量 ====================
QMI8658_ACC_RANGE_2G = const(0)
QMI8658_ACC_RANGE_4G = const(1)
QMI8658_ACC_RANGE_8G = const(2)
QMI8658_ACC_RANGE_16G = const(3)

QMI8658_GYR_RANGE_16DPS = const(0)
QMI8658_GYR_RANGE_32DPS = const(1)
QMI8658_GYR_RANGE_64DPS = const(2)
QMI8658_GYR_RANGE_128DPS = const(3)
QMI8658_GYR_RANGE_256DPS = const(4)
QMI8658_GYR_RANGE_512DPS = const(5)
QMI8658_GYR_RANGE_1024DPS = const(6)
QMI8658_GYR_RANGE_2048DPS = const(7)

QMI8658_ODR_8000HZ = const(0x00)
QMI8658_ODR_4000HZ = const(0x01)
QMI8658_ODR_2000HZ = const(0x02)
QMI8658_ODR_1000HZ = const(0x03)
QMI8658_ODR_500HZ = const(0x04)
QMI8658_ODR_250HZ = const(0x05)
QMI8658_ODR_125HZ = const(0x06)
QMI8658_ODR_62_5HZ = const(0x07)
QMI8658_ODR_31_25HZ = const(0x08)

# ==================== 中断引脚 / FIFO ====================
INT_PIN_1 = const(1)
INT_PIN_2 = const(2)

FIFO_MODE_BYPASS = const(0x00)
FIFO_MODE_FIFO = const(0x01)
FIFO_MODE_STREAM = const(0x02)

# ==================== CTRL9 命令 ====================
CTRL9_CMD_RST_FIFO = const(0x04)
CTRL9_CMD_REQ_FIFO = const(0x0D)
CTRL9_CMD_WRITE_WOM_SET = const(0x08)
CTRL9_CMD_RESET_PEDOMETER = const(0x0F)
CTRL9_CMD_SOFT_RESET = const(0xA2)

# ==================== STATUSINT 位 ====================
STATUSINT_BIT_DRDY = const(1 << 0)
STATUSINT_BIT_TAP = const(1 << 1)
STATUSINT_BIT_WOM = const(1 << 2)
STATUSINT_BIT_SIGMOTION = const(1 << 3)
STATUSINT_BIT_ANYMOTION = const(1 << 4)
STATUSINT_BIT_FIFO = const(1 << 5)
STATUSINT_BIT_CMD_DONE = const(1 << 7)

# 内部常量
_FIFO_SAMPLE_BYTES = const(14)
_DATA_BYTES = const(14)


class QMI8658:
    """
    QMI8658C 6轴 IMU 驱动类。

    属性:
        last_error (Exception | None): 最近一次中断/后台错误。
        acc_lsb_div (int): 加速度换算系数，init() 后有效。
        gyr_lsb_div (int): 角速度换算系数，init() 后有效。
    """

    __slots__ = (
        "_i2c",
        "_addr",
        "_int_pins",
        "_int_callbacks",
        "_int_triggers",
        "_int_bound",
        "_ctrl8_val",
        "_drdy_int_pin",
        "_last_error",
        "_error_handler",
        "_retries",
        "_retry_delay_ms",
        "_deinit",
        "_pending",
        "_acc_range",
        "_gyr_range",
        "_acc_gyr_odr",
        "_buf",
        "_fifo_buf",
        "acc_lsb_div",
        "gyr_lsb_div",
    )

    def __init__(
        self,
        i2c_bus,
        addr=QMI8658_DEFAULT_ADDR,
        int1_pin=None,
        int2_pin=None,
        acc_range=QMI8658_ACC_RANGE_8G,
        gyr_range=QMI8658_GYR_RANGE_2048DPS,
        acc_gyr_odr=QMI8658_ODR_500HZ,
        error_handler=None,
        retries=2,
        retry_delay_ms=5,
    ):
        """
        Args:
            i2c_bus (I2C): 已初始化的 I2C 实例（外部注入，不可为 None）。
            addr (int): I2C 地址，默认 0x6B。
            int1_pin / int2_pin (int | Pin | None): INT 引脚，
                内部归一化为 Pin.IN + Pin.PULL_UP。
            acc_range (int): 加速度量程，默认 8g。
            gyr_range (int): 角速度量程，默认 2048dps。
            acc_gyr_odr (int): 加速度/角速度输出速率，默认 500Hz。
            error_handler (callable | None): ISR 异常回调，签名 fn(exc)。
            retries (int): I2C 重试次数，默认 2（总共尝试 retries+1 次）。
            retry_delay_ms (int): 重试间隔 ms，默认 5。

        Raises:
            ValueError: i2c 为 None。
            OSError: 芯片 ID 不匹配或 I2C 通信失败。
        """
        if i2c_bus is None:
            raise ValueError("i2c_bus must not be None")

        self._i2c = i2c_bus
        self._addr = addr
        self._acc_range = acc_range
        self._gyr_range = gyr_range
        self._acc_gyr_odr = acc_gyr_odr

        self._int_pins = {
            INT_PIN_1: self._normalize_pin(int1_pin),
            INT_PIN_2: self._normalize_pin(int2_pin),
        }
        self._int_callbacks = {INT_PIN_1: [], INT_PIN_2: []}
        self._int_triggers = {
            INT_PIN_1: Pin.IRQ_FALLING,
            INT_PIN_2: Pin.IRQ_FALLING,
        }
        self._int_bound = {INT_PIN_1: False, INT_PIN_2: False}
        self._ctrl8_val = 0x00
        self._drdy_int_pin = INT_PIN_2
        self._last_error = None
        self._error_handler = error_handler
        self._retries = retries
        self._retry_delay_ms = retry_delay_ms
        self._deinit = False
        self._pending = False
        self._buf = bytearray(_DATA_BYTES)
        self._fifo_buf = bytearray(_FIFO_SAMPLE_BYTES)

        # 换算系数默认值，init() 会覆盖
        self.acc_lsb_div = 1 << (14 - acc_range)
        self.gyr_lsb_div = 2 ** (11 - gyr_range)

        if self._chip_id() != QMI8658_CHIP_ID:
            raise OSError("QMI8658 not found at I2C address 0x%02X" % addr)

        self.init(acc_range, gyr_range, acc_gyr_odr)

    # ==================== 只读属性 ====================
    @property
    def last_error(self):
        """最近一次中断/后台错误。"""
        return self._last_error

    # ==================== 引脚归一化 ====================
    @staticmethod
    def _normalize_pin(pin):
        """
        归一化为 machine.Pin 输入引脚（IN + PULL_UP）。

        Args:
            pin (int | machine.Pin | None)

        Returns:
            machine.Pin | None
        """
        if pin is None:
            return None
        if isinstance(pin, int):
            return Pin(pin, Pin.IN, Pin.PULL_UP)
        pin.init(Pin.IN, Pin.PULL_UP)
        return pin

    # ==================== 底层 I2C ====================
    def _check_deinit(self):
        if self._deinit:
            raise RuntimeError("device is deinitialized")

    def _write_reg(self, reg, val):
        """单字节写寄存器；带重试。失败抛 OSError。"""
        self._check_deinit()
        data = bytes((val,))
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

    def _read_reg(self, reg, nbytes=1):
        """
        读寄存器。
        Args:
            reg (int): 寄存器地址。
            nbytes (int): 字节数。
        Returns:
            int (nbytes==1) 或 bytes (nbytes>1)
        Raises:
            OSError: I2C 失败。
        """
        self._check_deinit()
        buf = bytearray(nbytes)
        last_err = None
        for attempt in range(self._retries + 1):
            try:
                self._i2c.readfrom_mem_into(self._addr, reg, buf)
                return buf[0] if nbytes == 1 else bytes(buf)
            except OSError as exc:
                last_err = exc
                if attempt < self._retries:
                    time.sleep_ms(self._retry_delay_ms)
        raise last_err

    def _read_reg_into(self, reg, buf):
        """读寄存器到预分配缓冲区；带重试。"""
        self._check_deinit()
        last_err = None
        for attempt in range(self._retries + 1):
            try:
                self._i2c.readfrom_mem_into(self._addr, reg, buf)
                return
            except OSError as exc:
                last_err = exc
                if attempt < self._retries:
                    time.sleep_ms(self._retry_delay_ms)
        raise last_err

    def _chip_id(self):
        return self._read_reg(QMI8658_REG_DEVICE_ID)

    # ==================== 初始化 ====================
    def init(
        self,
        acc_range=QMI8658_ACC_RANGE_8G,
        gyr_range=QMI8658_GYR_RANGE_2048DPS,
        acc_gyr_odr=QMI8658_ODR_500HZ,
        drdy_int_pin=INT_PIN_2,
        int_pushpull=True,
    ):
        """
        初始化传感器。可重复调用。

        Args:
            acc_range (int): 加速度量程。
            gyr_range (int): 角速度量程。
            acc_gyr_odr (int): 输出速率。
            drdy_int_pin (int): DRDY 中断路由引脚（INT_PIN_1/2）。
            int_pushpull (bool): True 推挽输出，False 高阻。

        Notes:
            - 重新配置 CTRL8 会清空已使能的事件位（Tap/Motion 等）；
              需重新调用对应 xxx_interrupt() 以恢复硬件中断。
            - 软件回调列表（_int_callbacks）保留。
            - 软复位后等待 100ms 再继续 I2C 操作。
        """
        self._check_deinit()
        self._write_reg(QMI8658_REG_CTRL9, CTRL9_CMD_SOFT_RESET)
        time.sleep_ms(100)

        ctrl1_val = 0x40 | 0x02
        if int_pushpull:
            ctrl1_val |= (1 << 3) | (1 << 4)
        self._write_reg(QMI8658_REG_CTRL1, ctrl1_val)
        self._write_reg(QMI8658_REG_CTRL7, 0x03)

        ctrl8_val = 0x00
        if drdy_int_pin == INT_PIN_1:
            ctrl8_val |= 1 << 6
        self._write_reg(QMI8658_REG_CTRL8, ctrl8_val)
        self._ctrl8_val = ctrl8_val

        self._write_reg(QMI8658_REG_CTRL2, (acc_range << 4) | acc_gyr_odr)
        self._write_reg(QMI8658_REG_CTRL3, (gyr_range << 4) | acc_gyr_odr)
        self._write_reg(QMI8658_REG_CTRL5, 0x75)

        self.gyr_lsb_div = 2 ** (11 - gyr_range)
        self.acc_lsb_div = 1 << (14 - acc_range)
        self._drdy_int_pin = drdy_int_pin
        self._acc_range = acc_range
        self._gyr_range = gyr_range
        self._acc_gyr_odr = acc_gyr_odr

    # ================================================================
    #  中断 API
    # ================================================================
    def _call_error_handler(self, exc):
        """保存并分发错误。"""
        self._last_error = exc
        handler = self._error_handler
        if handler is not None:
            try:
                handler(exc)
            except Exception as nested:
                self._last_error = nested

    def attach_interrupt(
        self, pin_num, handler=None, trigger=None, event_mask=0xFF, hard=False
    ):
        """
        在指定引脚注册中断回调。

        Args:
            pin_num (int): INT_PIN_1 或 INT_PIN_2。
            handler (callable | None): 回调 fn(events, status)，
                events 为事件名列表，status 为 STATUSINT 原始值。
            trigger: 触发沿，None 表示沿用当前设置。
            event_mask (int): 只在这些事件位被置位时才调用回调（0xFF=全部）。
            hard (bool): True 硬中断（仅置 _pending，需 process_pending()）；
                         False 软中断（默认，回调内可安全 I2C）。

        Returns:
            bool: 成功绑定返回 True；引脚未配置返回 False。

        Notes:
            - 同一引脚重复调用只追加回调，不重复绑定。
            - 回调签名文档：fn(events: list[str], status: int)
        """
        self._check_deinit()
        if handler is not None:
            self._int_callbacks[pin_num].append((event_mask, handler))

        if trigger is not None:
            self._int_triggers[pin_num] = trigger

        if self._int_bound[pin_num]:
            return True

        pin = self._int_pins.get(pin_num)
        if pin is None:
            return False

        if hard:

            def _isr(_pin_obj):
                # ISR 内仅置标志
                self._pending = True

            pin.irq(
                trigger=self._int_triggers[pin_num],
                handler=_isr,
                hard=True,
            )
        else:

            def _isr(_pin_obj):
                # 软中断：读取状态 + 分发回调；异常走 error_handler
                try:
                    events, status = self.parse_interrupt_status()
                except Exception as exc:
                    self._call_error_handler(exc)
                    return
                for mask, cb in list(self._int_callbacks[pin_num]):
                    if mask == 0 or (status & mask):
                        try:
                            cb(events, status)
                        except Exception as exc:
                            self._call_error_handler(exc)

            pin.irq(
                trigger=self._int_triggers[pin_num],
                handler=_isr,
                hard=False,
            )

        self._int_bound[pin_num] = True
        return True

    def process_pending(self):
        """
        硬中断模式下在主循环中调用，处理挂起的中断。

        Returns:
            bool: 本次是否处理了中断。
        """
        self._check_deinit()
        if not self._pending:
            return False
        self._pending = False
        try:
            events, status = self.parse_interrupt_status()
        except Exception as exc:
            self._call_error_handler(exc)
            return True
        # 遍历两个引脚上的回调，按 event_mask 过滤
        for pin_num in (INT_PIN_1, INT_PIN_2):
            for mask, cb in list(self._int_callbacks[pin_num]):
                if mask == 0 or (status & mask):
                    try:
                        cb(events, status)
                    except Exception as exc:
                        self._call_error_handler(exc)
        return True

    def detach_interrupt(self, pin_num=None):
        """
        解除引脚中断绑定及所有回调。

        Args:
            pin_num (int | None): INT_PIN_1 / INT_PIN_2；None 表示全部。

        Notes:
            本方法只解绑引脚，不关闭硬件事件使能位（CTRL8）。
        """
        if pin_num is None:
            self.detach_interrupt(INT_PIN_1)
            self.detach_interrupt(INT_PIN_2)
            return

        pin = self._int_pins.get(pin_num)
        if pin is not None:
            try:
                pin.irq(handler=None)
            except Exception:
                pass
        self._int_callbacks[pin_num] = []
        self._int_bound[pin_num] = False

    # ==================== 数据读取 ====================
    def status(self):
        """读取 STATUSINT 寄存器（读清除）。"""
        return self._read_reg(QMI8658_REG_STATUS_INT)

    @staticmethod
    def u2s(n):
        """无符号 16 位转有符号。"""
        return n if n < (1 << 15) else n - (1 << 16)

    def read_raw(self):
        """
        读取原始传感器数据（无 float 运算，适合高频场景）。

        Returns:
            tuple: (acc_raw, gyr_raw, temp_raw)
                   acc_raw = (ax, ay, az) 原始 int
                   gyr_raw = (gx, gy, gz) 原始 int
                   temp_raw = 原始 int
        """
        self._read_reg_into(QMI8658_REG_DATA, self._buf)
        b = self._buf
        u2s = self.u2s
        return (
            (u2s((b[3] << 8) | b[2]), u2s((b[5] << 8) | b[4]), u2s((b[7] << 8) | b[6])),
            (
                u2s((b[9] << 8) | b[8]),
                u2s((b[11] << 8) | b[10]),
                u2s((b[13] << 8) | b[12]),
            ),
            u2s((b[1] << 8) | b[0]),
        )

    def read(self):
        """
        读取并换算为物理量。

        Returns:
            tuple: (acc_g, gyr_dps, temp_c)
                   acc_g = (ax, ay, az) 单位 g
                   gyr_dps = (gx, gy, gz) 单位 dps
                   temp_c = 温度 ℃（保留 2 位小数）

        Raises:
            OSError: I2C 读取失败。
        """
        acc_raw, gyr_raw, temp_raw = self.read_raw()
        acc_div = self.acc_lsb_div
        gyr_div = self.gyr_lsb_div
        acc = (acc_raw[0] / acc_div, acc_raw[1] / acc_div, acc_raw[2] / acc_div)
        gyr = (gyr_raw[0] / gyr_div, gyr_raw[1] / gyr_div, gyr_raw[2] / gyr_div)
        return acc, gyr, round(temp_raw / 256.0, 2)

    # 兼容别名
    def get_data(self):
        """[兼容别名] 请使用 read()。将在 v2.0 移除。"""
        return self.read()

    def get_accelerometer(self):
        return self.read()[0]

    def get_gyroscope(self):
        return self.read()[1]

    def get_temperature(self):
        return self.read()[2]

    def get_strength(self):
        x, y, z = self.get_accelerometer()
        return math.sqrt(x * x + y * y + z * z)

    def get_pitch_roll(self):
        """返回 (pitch_deg, roll_deg)，俯仰角正值表示机头抬起。"""
        x, y, z = self.get_accelerometer()
        pitch = math.atan2(-x, math.sqrt(y * y + z * z))
        roll = math.atan2(y, z)
        return math.degrees(pitch), math.degrees(roll)

    # ================================================================
    #  专用事件配置（内部调用 attach_interrupt 完成绑定）
    # ================================================================
    def drdy_interrupt(
        self, enable=True, handler=None, trigger=Pin.IRQ_RISING, hard=False
    ):
        """数据就绪中断，固定路由到 INT2（FIFO 需处于 Bypass 模式）。"""
        ctrl7 = self._read_reg(QMI8658_REG_CTRL7)
        if enable:
            ctrl7 &= ~(1 << 5)
            self._write_reg(QMI8658_REG_FIFO_CTRL, FIFO_MODE_BYPASS)
        else:
            ctrl7 |= 1 << 5
        self._write_reg(QMI8658_REG_CTRL7, ctrl7)
        self.attach_interrupt(INT_PIN_2, handler, trigger, STATUSINT_BIT_DRDY, hard)

    def fifo_interrupt(
        self,
        int_pin=INT_PIN_2,
        watermark=0x20,
        fifo_mode=FIFO_MODE_FIFO,
        handler=None,
        trigger=Pin.IRQ_FALLING,
        hard=False,
    ):
        """FIFO 水位中断。"""
        self._write_reg(QMI8658_REG_FIFO_WTM_TH, watermark)
        self._write_reg(QMI8658_REG_FIFO_CTRL, (fifo_mode & 0x03) | (1 << 2))
        ctrl1 = self._read_reg(QMI8658_REG_CTRL1)
        if int_pin == INT_PIN_1:
            ctrl1 |= 1 << 2
        else:
            ctrl1 &= ~(1 << 2)
        self._write_reg(QMI8658_REG_CTRL1, ctrl1)
        self.attach_interrupt(int_pin, handler, trigger, STATUSINT_BIT_FIFO, hard)

    def read_fifo(self, max_samples=32):
        """
        读取 FIFO 数据。

        Args:
            max_samples (int): 最多读取样本数。

        Returns:
            list: [(acc_g, gyr_dps, temp_c), ...]

        Notes:
            每样本 14 字节（温度 + 加速度 + 角速度），小端序。
            读取后自动复位 FIFO。
        """
        self._write_reg(QMI8658_REG_CTRL9, CTRL9_CMD_REQ_FIFO)
        time.sleep_ms(10)
        smpl_cnt = self._read_reg(QMI8658_REG_FIFO_SMPL_CNT)
        count = min(smpl_cnt, max_samples)

        results = []
        buf = self._fifo_buf
        u2s = self.u2s
        acc_div = self.acc_lsb_div
        gyr_div = self.gyr_lsb_div
        for _ in range(count):
            self._read_reg_into(QMI8658_REG_FIFO_DATA, buf)
            tmp = u2s((buf[1] << 8) | buf[0]) / 256.0
            acc = (
                u2s((buf[3] << 8) | buf[2]) / acc_div,
                u2s((buf[5] << 8) | buf[4]) / acc_div,
                u2s((buf[7] << 8) | buf[6]) / acc_div,
            )
            gyr = (
                u2s((buf[9] << 8) | buf[8]) / gyr_div,
                u2s((buf[11] << 8) | buf[10]) / gyr_div,
                u2s((buf[13] << 8) | buf[12]) / gyr_div,
            )
            results.append((acc, gyr, round(tmp, 2)))

        self._write_reg(QMI8658_REG_CTRL9, CTRL9_CMD_RST_FIFO)
        time.sleep_ms(10)
        return results

    def motion_interrupt(
        self,
        enable_any=True,
        enable_sig=True,
        int_pin=INT_PIN_1,
        sig_window=0x64,
        handler=None,
        trigger=Pin.IRQ_FALLING,
        hard=False,
    ):
        """Any/Sig-Motion 中断。"""
        ctrl8 = self._ctrl8_val
        if int_pin == INT_PIN_1:
            ctrl8 |= 1 << 6
        else:
            ctrl8 &= ~(1 << 6)
        ctrl8 = (ctrl8 | (1 << 0)) if enable_any else (ctrl8 & ~(1 << 0))
        ctrl8 = (ctrl8 | (1 << 1)) if enable_sig else (ctrl8 & ~(1 << 1))
        self._write_reg(QMI8658_REG_CAL3_L, sig_window & 0xFF)
        self._write_reg(QMI8658_REG_CAL3_H, (sig_window >> 8) & 0xFF)
        self._write_reg(QMI8658_REG_CTRL8, ctrl8)
        self._ctrl8_val = ctrl8
        self.attach_interrupt(
            int_pin,
            handler,
            trigger,
            STATUSINT_BIT_ANYMOTION | STATUSINT_BIT_SIGMOTION,
            hard,
        )

    def tap_interrupt(
        self,
        enable=True,
        int_pin=INT_PIN_1,
        threshold=0x40,
        double_tap=True,
        handler=None,
        trigger=Pin.IRQ_FALLING,
        hard=False,
    ):
        """敲击检测中断。"""
        ctrl8 = self._ctrl8_val
        if int_pin == INT_PIN_1:
            ctrl8 |= 1 << 6
        else:
            ctrl8 &= ~(1 << 6)
        ctrl8 = (ctrl8 | (1 << 3)) if enable else (ctrl8 & ~(1 << 3))
        self._write_reg(QMI8658_REG_CTRL8, ctrl8)
        self._ctrl8_val = ctrl8
        self._write_reg(QMI8658_REG_CAL1_L, threshold & 0xFF)
        cal1_h = self._read_reg(QMI8658_REG_CAL1_H)
        cal1_h = (cal1_h | (1 << 6)) if double_tap else (cal1_h & ~(1 << 6))
        self._write_reg(QMI8658_REG_CAL1_H, cal1_h)
        self.attach_interrupt(int_pin, handler, trigger, STATUSINT_BIT_TAP, hard)

    def read_tap_status(self):
        """返回 (axis, polarity, taps)。"""
        val = self._read_reg(QMI8658_REG_TAP_STATUS)
        return val & 0x03, (val >> 2) & 0x01, (val >> 3) & 0x03

    def pedometer_interrupt(
        self,
        enable=True,
        int_pin=INT_PIN_1,
        handler=None,
        trigger=Pin.IRQ_FALLING,
        hard=False,
    ):
        """计步器中断。"""
        ctrl8 = self._ctrl8_val
        if int_pin == INT_PIN_1:
            ctrl8 |= 1 << 6
        else:
            ctrl8 &= ~(1 << 6)
        ctrl8 = (ctrl8 | (1 << 4)) if enable else (ctrl8 & ~(1 << 4))
        self._write_reg(QMI8658_REG_CTRL8, ctrl8)
        self._ctrl8_val = ctrl8
        # 计步器无独立 statusint 位，用 0xFF 保证回调总被调用
        self.attach_interrupt(int_pin, handler, trigger, 0xFF, hard)

    def read_step_count(self):
        low = self._read_reg(QMI8658_REG_STEP_CNT_LOW)
        mid = self._read_reg(QMI8658_REG_STEP_CNT_MID)
        high = self._read_reg(QMI8658_REG_STEP_CNT_HIGH)
        return (high << 16) | (mid << 8) | low

    def reset_step_count(self):
        self._write_reg(QMI8658_REG_CTRL9, CTRL9_CMD_RESET_PEDOMETER)
        time.sleep_ms(10)

    def wom_interrupt(
        self,
        enable=True,
        int_pin=INT_PIN_1,
        threshold=0x10,
        handler=None,
        trigger=Pin.IRQ_RISING,
        hard=False,
    ):
        """唤醒运动（WoM）中断。"""
        self._write_reg(QMI8658_REG_CAL1_L, threshold & 0xFF)
        cal1_h = self._read_reg(QMI8658_REG_CAL1_H)
        if int_pin == INT_PIN_1:
            cal1_h |= 1 << 6
            cal1_h &= ~(1 << 7)
        else:
            cal1_h &= ~(1 << 6)
            cal1_h &= ~(1 << 7)
        self._write_reg(QMI8658_REG_CAL1_H, cal1_h)
        if enable:
            self._write_reg(QMI8658_REG_CTRL9, CTRL9_CMD_WRITE_WOM_SET)
            time.sleep_ms(50)
        else:
            self._write_reg(QMI8658_REG_CAL1_L, 0x00)
        self.attach_interrupt(int_pin, handler, trigger, STATUSINT_BIT_WOM, hard)

    def cmd_done_interrupt(
        self,
        route_to_statusint=False,
        int_pin=INT_PIN_1,
        handler=None,
        trigger=Pin.IRQ_FALLING,
        hard=False,
    ):
        """CmdDone 中断。"""
        ctrl8 = self._read_reg(QMI8658_REG_CTRL8)
        ctrl8 = (ctrl8 | (1 << 7)) if route_to_statusint else (ctrl8 & ~(1 << 7))
        self._write_reg(QMI8658_REG_CTRL8, ctrl8)
        self._ctrl8_val = ctrl8
        self.attach_interrupt(int_pin, handler, trigger, STATUSINT_BIT_CMD_DONE, hard)

    def parse_interrupt_status(self, status=None):
        """解析 STATUSINT，返回 (events, status)。"""
        if status is None:
            status = self.status()
        events = []
        if status & STATUSINT_BIT_DRDY:
            events.append("DRDY")
        if status & STATUSINT_BIT_FIFO:
            events.append("FIFO")
        if status & STATUSINT_BIT_ANYMOTION:
            events.append("AnyMotion")
        if status & STATUSINT_BIT_SIGMOTION:
            events.append("SigMotion")
        if status & STATUSINT_BIT_TAP:
            events.append("Tap")
        if status & STATUSINT_BIT_WOM:
            events.append("WoM")
        if status & STATUSINT_BIT_CMD_DONE:
            events.append("CmdDone")
        return events, status

    def clear_interrupt(self):
        """[统一接口] 读取 STATUSINT 以清除所有中断标志。"""
        return self._read_reg(QMI8658_REG_STATUS_INT)

    def deinit(self):
        """
        释放资源：解绑所有中断，置 deinit 标志。幂等。

        Notes:
            deinit 后调用任何 I2C 方法抛 RuntimeError("device is deinitialized")。
        """
        if self._deinit:
            return
        self.detach_interrupt()
        self._i2c = None
        self._deinit = True
