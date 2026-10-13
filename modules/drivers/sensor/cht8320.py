"""
CHT8320 温湿度传感器 MicroPython 驱动

依赖:
    time, micropython.const
目标固件:
    MicroPython v1.19+（ESP32 / RP2040 / STM32 / ESP8266）

协议:
    CHT8320 使用命令协议（非寄存器协议）:
        发 2 字节命令 → 等待 → 读 6 字节结果
    结果格式: [T_MSB T_LSB T_CRC H_MSB H_LSB H_CRC]
    CRC 算法: Sensirion CRC-8（多项式 0x31，初值 0xFF）

换算:
    temperature_c = -45.0 + 175.0 * t_raw / 65535.0
    humidity_rh  = 100.0 * h_raw / 65535.0

缓存策略（cache_timeout_ms）:
    0    -> 禁用缓存，每次访问属性都重新测量
    None -> 永久缓存，仅首次访问或 invalidate() 后测量
    >0   -> 缓存有效期 ms

用法（最小示例）:
    from machine import I2C, Pin
    import time
    from cht8320 import CHT8320

    i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
    sensor = CHT8320(i2c)
    while True:
        print(sensor.temperature, sensor.humidity)
        time.sleep_ms(1000)

用法（不缓存）:
    sensor = CHT8320(i2c, cache_timeout_ms=0)

用法（永久缓存）:
    sensor = CHT8320(i2c, cache_timeout_ms=None)

用法（手动控制测量时机）:
    sensor.measure("high")       # 触发测量，不阻塞
    time.sleep_ms(20)
    t, h = sensor.read()         # 读取结果
"""

import time

from micropython import const

__all__ = ("CHT8320",)

_CHT8320_DEFAULT_ADDR = const(0x44)

# 重复性 -> (单次测量命令, 建议等待毫秒)
# 数据手册典型值 + 20%~30% 余量
_REPEATABILITY = {
    "high": (const(0x2400), 20),
    "medium": (const(0x240B), 10),
    "low": (const(0x2416), 5),
}

_SOFT_RESET = const(0x30A2)
_HEATER_ENABLE = const(0x306D)
_HEATER_DISABLE = const(0x3066)

_READ_SIZE = const(6)
_CRC_POLY = const(0x31)


class CHT8320:
    """
    CHT8320 温湿度传感器驱动类。

    属性:
        temperature (float): 温度 ℃。
        humidity (float): 相对湿度 %RH。
        last_error (Exception | None): 最近一次后台错误。
    """

    __slots__ = (
        "_i2c",
        "_addr",
        "_repeatability",
        "_cache_timeout_ms",
        "_temperature",
        "_humidity",
        "_last_read_ms",
        "_last_error",
        "_retries",
        "_retry_delay_ms",
        "_deinit",
        "_buf",
    )

    def __init__(
        self,
        i2c,
        addr=_CHT8320_DEFAULT_ADDR,
        repeatability="high",
        cache_timeout_ms=1000,
        retries=2,
        retry_delay_ms=5,
    ):
        """
        Args:
            i2c (I2C): 已初始化的 I2C 对象（外部注入）。
            addr (int): I2C 地址，默认 0x44。
            repeatability (str): 'high' / 'medium' / 'low'。
            cache_timeout_ms (int | None): 缓存有效期 ms；
                0=禁用，None=永久，>0=有效期。
            retries (int): I2C 重试次数，默认 2。
            retry_delay_ms (int): 重试间隔 ms，默认 5。

        Raises:
            ValueError: 参数非法。
        """
        if repeatability not in _REPEATABILITY:
            raise ValueError("repeatability must be 'high', 'medium' or 'low'")
        if cache_timeout_ms is not None and (
            not isinstance(cache_timeout_ms, int) or cache_timeout_ms < 0
        ):
            raise ValueError("cache_timeout_ms must be None or int >= 0")

        self._i2c = i2c
        self._addr = addr
        self._repeatability = repeatability
        self._cache_timeout_ms = cache_timeout_ms
        self._temperature = None
        self._humidity = None
        self._last_read_ms = None
        self._last_error = None
        self._retries = retries
        self._retry_delay_ms = retry_delay_ms

        self._buf = bytearray(_READ_SIZE)

        # 上电后软复位到确定状态
        time.sleep_ms(2)
        self._write_cmd(_SOFT_RESET)
        time.sleep_ms(2)

    # ---------------- 只读属性 ----------------
    @property
    def temperature(self):
        """温度 ℃。缓存过期或缺失时自动测量。"""
        self._ensure_data()
        return self._temperature

    @property
    def humidity(self):
        """相对湿度 %RH。缓存过期或缺失时自动测量。"""
        self._ensure_data()
        return self._humidity

    @property
    def last_error(self):
        """最近一次后台错误。"""
        return self._last_error

    # ---------------- 内部 ----------------
    def _check_deinit(self):
        if self._i2c is None:
            raise RuntimeError("device is deinitialized")

    def _ensure_data(self):
        if self._is_cache_valid():
            return
        self.measure_and_read(self._repeatability)

    def _is_cache_valid(self):
        if self._temperature is None or self._humidity is None:
            return False
        if self._cache_timeout_ms is None:
            return True
        if self._cache_timeout_ms == 0:
            return False
        if self._last_read_ms is None:
            return False
        return (
            time.ticks_diff(time.ticks_ms(), self._last_read_ms)
            < self._cache_timeout_ms
        )

    # ---------------- 底层 ----------------
    def _write_cmd(self, cmd):
        """发送 2 字节命令，大端；带重试。"""
        self._check_deinit()
        data = cmd.to_bytes(2, "big")
        last_err = None
        for attempt in range(self._retries + 1):
            try:
                self._i2c.writeto(self._addr, data)
                return
            except OSError as exc:
                last_err = exc
                if attempt < self._retries:
                    time.sleep_ms(self._retry_delay_ms)
        raise last_err

    def _read_raw(self):
        """
        从设备读取 6 字节原始数据（命令协议，无寄存器地址）。

        Returns:
            tuple: (t_raw, h_raw) 已 CRC 校验通过的原始值。

        Raises:
            OSError: I2C 失败或 CRC 校验失败。
        """
        self._check_deinit()
        last_err = None
        for attempt in range(self._retries + 1):
            try:
                self._i2c.readfrom_into(self._addr, self._buf)
                break
            except OSError as exc:
                last_err = exc
                if attempt < self._retries:
                    time.sleep_ms(self._retry_delay_ms)
        else:
            raise last_err

        b = self._buf
        if self._crc8(b[0:2]) != b[2]:
            raise OSError("Temperature CRC error")
        if self._crc8(b[3:5]) != b[5]:
            raise OSError("Humidity CRC error")

        return (b[0] << 8) | b[1], (b[3] << 8) | b[4]

    @staticmethod
    def _crc8(data):
        """Sensirion CRC-8（poly=0x31, init=0xFF）。"""
        crc = 0xFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ _CRC_POLY) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
        return crc

    # ---------------- 公开 API ----------------
    def reset(self):
        """软复位传感器并清空缓存。"""
        self._check_deinit()
        self._write_cmd(_SOFT_RESET)
        time.sleep_ms(2)
        self.invalidate()

    def measure(self, repeatability="high"):
        """
        触发一次测量，非阻塞；之后需等待再调用 read()。

        Args:
            repeatability (str): 'high' / 'medium' / 'low'。

        Raises:
            ValueError: repeatability 非法。
        """
        self._check_deinit()
        try:
            cmd, _ = _REPEATABILITY[repeatability]
        except KeyError:
            raise ValueError("repeatability must be 'high', 'medium' or 'low'")
        self._write_cmd(cmd)

    def read(self):
        """
        读取上一次测量的结果。

        Returns:
            tuple: (temperature_c, humidity_rh)

        Raises:
            OSError: CRC 或 I2C 失败。

        Notes:
            成功时更新缓存与时间戳；CRC 失败时缓存保持不变。
        """
        self._check_deinit()
        t_raw, h_raw = self._read_raw()

        # 先计算再赋值，保证原子性
        temperature = -45.0 + 175.0 * t_raw / 65535.0
        humidity = 100.0 * h_raw / 65535.0
        self._temperature = temperature
        self._humidity = humidity
        self._last_read_ms = time.ticks_ms()
        return temperature, humidity

    def measure_and_read(self, repeatability="high"):
        """
        触发测量、等待、读取，返回 (temperature_c, humidity_rh) 并更新缓存。
        阻塞方法，等待时间见 _REPEATABILITY。

        Args:
            repeatability (str): 'high' / 'medium' / 'low'。

        Returns:
            tuple: (temperature_c, humidity_rh)

        Raises:
            ValueError: repeatability 非法。
            OSError: CRC 或 I2C 失败。
        """
        self._check_deinit()
        try:
            cmd, delay = _REPEATABILITY[repeatability]
        except KeyError:
            raise ValueError("repeatability must be 'high', 'medium' or 'low'")
        self._write_cmd(cmd)
        time.sleep_ms(delay)
        return self.read()

    def invalidate(self):
        """清空缓存与时间戳，下次访问属性时重新测量。"""
        self._temperature = None
        self._humidity = None
        self._last_read_ms = None

    def heater_enable(self, enable=True):
        """启用/禁用内部加热器，用于冷凝环境。"""
        self._check_deinit()
        self._write_cmd(_HEATER_ENABLE if enable else _HEATER_DISABLE)

    def deinit(self):
        """释放资源：置 deinit 标志。幂等。"""
        if self._i2c is None:
            return
        self._i2c = None
