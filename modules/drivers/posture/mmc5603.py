"""
MMC5603 三轴磁力计驱动（MicroPython）。

用途:
    通过 I2C 读取 MMC5603 三轴磁力计数据，提供原始计数、毫高斯值、
    磁场模长、磁航向角与硬磁校准接口。

依赖:
    machine（I2C 由调用方注入）、time、math、micropython.const。

目标固件:
    MicroPython v1.19+，使用 readfrom_mem_into / ticks_diff。

单位与坐标系:
    - 输出单位：毫高斯（mG）。value_mg = 0.0625 * (raw - offset)。
    - 原始值范围 0 .. 2**20-1，零点 2**19 = 524288。
    - get_heading() 使用 X/Y 平面，仅适用于传感器水平放置：
          板子 +Y 指向磁北 -> 0°
          板子 +X 指向磁北 -> 90°
          板子 -Y 指向磁北 -> 180°
          板子 -X 指向磁北 -> 270°
      计算式 heading = (degrees(atan2(-x_mg, y_mg)) + declination) % 360。

中断:
    本驱动不使用硬件中断。ISR 内不要调用 read() / read_raw()。
    如需后台采样，请在主循环或定时器中周期性调用 read()。

版本:
    v1.0.0

变更日志:
    [FIX]      deinit() 后调用 I/O 方法抛 RuntimeError，不再触发 AttributeError。
    [ADD]      read_strength() 磁场模长（此前仅在模块 docstring 中出现）。
    [CHANGE]   read_raw() 内部解析逻辑抽到 _parse_raw() 静态方法。

典型用法:
    from machine import I2C, Pin
    from mmc5603 import MMC5603
    mag = MMC5603(I2C(0, scl=Pin(1), sda=Pin(0), freq=400_000))
    mag.read()                            # (x, y, z) mG
    mag.read_raw()                        # (x, y, z) 原始计数
    mag.read_strength()                   # 磁场模长 mG
    mag.get_heading()                     # 磁航向 [0, 360)
    mag.get_heading(declination=-3.5)     # 真北方向
    mag.calibrate(); mag.save_calibration("mag_cal.txt")
"""

import math
import time

from micropython import const

__all__ = ("MMC5603",)


# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------

_I2C_ADDR_DEFAULT = const(0x30)
_DEVICE_ID = const(0x10)

_REG_DATA = const(0x00)
_REG_TMP = const(0x09)
_REG_ODR = const(0x1A)
_REG_CTRL0 = const(0x1B)
_REG_CTRL1 = const(0x1C)
_REG_CTRL2 = const(0x1D)
_REG_X_THD = const(0x1E)
_REG_Y_THD = const(0x1F)
_REG_Z_THD = const(0x20)
_REG_ST_X_VAL = const(0x27)
_REG_DEVICE_ID = const(0x39)

_DATA_BYTES = const(9)
_ST_TRIM_BYTES = const(3)

_CTRL0_SET = const(0x08)
_CTRL0_CONTINUOUS = const(0xA0)
_CTRL2_CMM_FREQ_EN = const(0x10)

_BANDWIDTH_DEFAULT = const(0x00)
_ODR_HZ_DEFAULT = const(50)

_LSB_MG = 0.0625
_OFFSET_CENTER = const(524288)

_ST_TRIM_CENTER = const(128)
_ST_TRIM_SCALE = const(32)
_ST_THD_MAX = const(255)

_STARTUP_DELAY_MS = const(50)
_SET_DELAY_MS = const(5)

_HEADING_FULL_TURN_DEG = 360.0


class MMC5603:
    """
    MMC5603 三轴磁力计驱动。

    不变量:
        - 构造后处于连续测量模式（带宽 0x00，ODR 50 Hz）。
        - read() 值已扣硬磁偏移，直接使用芯片输出符号。
        - deinit() 后所有 I/O 方法抛 RuntimeError("device is deinitialized")。

    本驱动不使用硬件中断；ISR 内不要调用 read() / read_raw()。
    """

    __slots__ = (
        "_i2c",
        "_addr",
        "_retries",
        "_retry_delay_ms",
        "_tx_buf",
        "_rx_buf",
        "_data_buf",
        "_st_buf",
        "_offset_x",
        "_offset_y",
        "_offset_z",
        "_raw_x",
        "_raw_y",
        "_raw_z",
    )

    def __init__(
        self,
        i2c,
        addr=_I2C_ADDR_DEFAULT,
        retries=2,
        retry_delay_ms=5,
    ):
        """
        Args:
            i2c: 已初始化的 machine.I2C 实例。
            addr (int): 7 位 I2C 地址，默认 0x30。
            retries (int): I2C 失败重试次数，默认 2。
            retry_delay_ms (int): 重试间隔 ms，默认 5。

        Raises:
            ValueError: 参数非法。
            OSError: 找不到 MMC5603 或 ID 不匹配。

        Side effects:
            写 CTRL0/CTRL1/CTRL2/ODR/ST 寄存器，阻塞约 55 ms，不读写文件。
        """
        if not 0 <= addr <= 0x7F:
            raise ValueError("addr must be a 7-bit I2C address (0..0x7F)")
        if retries < 0:
            raise ValueError("retries must be >= 0")
        if retry_delay_ms < 0:
            raise ValueError("retry_delay_ms must be >= 0")

        self._i2c = i2c
        self._addr = addr
        self._retries = retries
        self._retry_delay_ms = retry_delay_ms

        self._tx_buf = bytearray(1)
        self._rx_buf = bytearray(1)
        self._data_buf = bytearray(_DATA_BYTES)
        self._st_buf = bytearray(_ST_TRIM_BYTES)

        self._offset_x = float(_OFFSET_CENTER)
        self._offset_y = float(_OFFSET_CENTER)
        self._offset_z = float(_OFFSET_CENTER)

        self._raw_x = 0
        self._raw_y = 0
        self._raw_z = 0

        if self._chip_id() != _DEVICE_ID:
            raise OSError("MMC5603 not found at I2C address 0x%02X" % self._addr)

        self._auto_selftest()
        self._set()
        self._start_continuous()
        time.sleep_ms(_STARTUP_DELAY_MS)

    # ------------------------------------------------------------------
    # 公开 API
    # ------------------------------------------------------------------

    def read_raw(self):
        """
        读三轴 20 位原始计数，不做换算、不扣偏移。

        Returns:
            tuple: (raw_x, raw_y, raw_z)，每个 0..2**20-1。

        Raises:
            OSError: I2C 读取失败。
            RuntimeError: 设备已 deinit。
        """
        self._read_into(_REG_DATA, self._data_buf)
        raw_x, raw_y, raw_z = self._parse_raw(self._data_buf)
        self._raw_x = raw_x
        self._raw_y = raw_y
        self._raw_z = raw_z
        return (raw_x, raw_y, raw_z)

    def read(self):
        """
        读三轴磁场强度，单位 mG（已扣硬磁偏移）。

        每次调用都会重新从设备读取，不做缓存。

        Returns:
            tuple: (x_mg, y_mg, z_mg)，float，单位 mG。

        Raises:
            OSError: I2C 读取失败。
            RuntimeError: 设备已 deinit。
        """
        raw_x, raw_y, raw_z = self.read_raw()
        x_mg = _LSB_MG * (raw_x - self._offset_x)
        y_mg = _LSB_MG * (raw_y - self._offset_y)
        z_mg = _LSB_MG * (raw_z - self._offset_z)
        return (x_mg, y_mg, z_mg)

    def read_strength(self):
        """
        读磁场模长，单位 mG，等于 sqrt(x² + y² + z²)。

        Returns:
            float: 磁场模长，单位 mG，非负。

        Raises:
            OSError: I2C 读取失败。
            RuntimeError: 设备已 deinit。
        """
        x_mg, y_mg, z_mg = self.read()
        return math.sqrt(x_mg * x_mg + y_mg * y_mg + z_mg * z_mg)

    def get_heading(self, declination=0.0):
        """
        计算水平面内航向角，顺时针为正。

        坐标：板子 +Y 指向磁北 -> 0°，+X -> 90°，-Y -> 180°，-X -> 270°。
        仅在传感器水平放置时有效；倾斜时需先做 tilt compensation。

        Args:
            declination (float): 磁偏角，单位 度，默认 0.0（输出相对磁北）。
                非 0 时输出相对真北；东偏为正，西偏为负；建议 [-180, 180]。

        Returns:
            float: 航向角，单位 度，范围 [0, 360)。

        Raises:
            OSError: I2C 读取失败。
            RuntimeError: 设备已 deinit。
        """
        x_mg, y_mg, _z_mg = self.read()
        heading = math.degrees(math.atan2(-x_mg, y_mg)) + declination
        if heading < 0.0:
            heading += _HEADING_FULL_TURN_DEG
        elif heading >= _HEADING_FULL_TURN_DEG:
            heading -= _HEADING_FULL_TURN_DEG
        return heading

    def calibrate(self, duration_ms=20000, sample_delay_ms=20, progress_fn=None):
        """
        采集硬磁偏移（各轴 raw 的 min/max 中值）。

        调用方式：在 duration_ms 内缓慢旋转板子"8"字。

        Args:
            duration_ms (int): 采集时长 ms，默认 20000，必须 > 0。
            sample_delay_ms (int): 采样间隔 ms，默认 20，必须 > 0。
            progress_fn: 可选回调 progress_fn(percent 0..100)。

        Returns:
            tuple: (offset_x, offset_y, offset_z)，float，单位 LSB。

        Raises:
            ValueError: 时长或间隔不为正。
            OSError: 采集期间 I2C 读取失败。
            RuntimeError: 设备已 deinit。

        Side effects:
            阻塞 duration_ms；更新偏移。不写文件。
        """
        if duration_ms <= 0:
            raise ValueError("duration_ms must be > 0")
        if sample_delay_ms <= 0:
            raise ValueError("sample_delay_ms must be > 0")

        raw_x, raw_y, raw_z = self.read_raw()
        min_x = max_x = raw_x
        min_y = max_y = raw_y
        min_z = max_z = raw_z

        start_ms = time.ticks_ms()
        while True:
            elapsed_ms = time.ticks_diff(time.ticks_ms(), start_ms)
            if elapsed_ms >= duration_ms:
                break

            raw_x, raw_y, raw_z = self.read_raw()
            if raw_x < min_x:
                min_x = raw_x
            if raw_x > max_x:
                max_x = raw_x
            if raw_y < min_y:
                min_y = raw_y
            if raw_y > max_y:
                max_y = raw_y
            if raw_z < min_z:
                min_z = raw_z
            if raw_z > max_z:
                max_z = raw_z

            if progress_fn is not None:
                progress_fn(elapsed_ms * 100 // duration_ms)
            time.sleep_ms(sample_delay_ms)

        if progress_fn is not None:
            progress_fn(100)

        self._offset_x = (min_x + max_x) * 0.5
        self._offset_y = (min_y + max_y) * 0.5
        self._offset_z = (min_z + max_z) * 0.5
        return (self._offset_x, self._offset_y, self._offset_z)

    def save_calibration(self, path):
        """
        把当前偏移写入文本文件（一行三个逗号分隔浮点数，单位 LSB）。

        不涉及 I2C，deinit 后仍可调用。

        Args:
            path (str): 目标文件路径。

        Raises:
            OSError: 文件无法创建或写入。
        """
        with open(path, "w") as handle:
            handle.write(
                "%s,%s,%s\n" % (self._offset_x, self._offset_y, self._offset_z)
            )

    def load_calibration(self, path):
        """
        从 save_calibration() 写出的文件读回偏移。

        不涉及 I2C，deinit 后仍可调用。

        Args:
            path (str): 源文件路径。

        Returns:
            tuple: (offset_x, offset_y, offset_z)，float。

        Raises:
            OSError: 文件不存在或无法读取。
            ValueError: 文件内容不是三个逗号分隔的数字。
        """
        with open(path) as handle:
            parts = handle.read().strip().split(",")
        if len(parts) != 3:
            raise ValueError("calibration file must contain 3 comma-separated numbers")
        offsets = (float(parts[0]), float(parts[1]), float(parts[2]))
        self.calibration = offsets
        return offsets

    def deinit(self):
        """
        关闭连续测量并释放 I2C 引用。幂等（重复调用直接返回）。

        deinit 之后任何 I/O 方法（read / read_raw / read_strength /
        get_heading / calibrate）会抛 RuntimeError("device is deinitialized")。
        纯计算与文件方法（calibration 属性、save_calibration、
        load_calibration）仍可调用。
        """
        if self._i2c is None:
            return
        try:
            self._wreg(_REG_CTRL0, 0x00)
            self._wreg(_REG_CTRL2, 0x00)
        finally:
            self._i2c = None

    # ------------------------------------------------------------------
    # 属性
    # ------------------------------------------------------------------

    @property
    def calibration(self):
        """
        当前硬磁偏移 (offset_x, offset_y, offset_z)，单位 LSB。

        未校准时为 2**19 = 524288。不涉及 I2C。
        """
        return (self._offset_x, self._offset_y, self._offset_z)

    @calibration.setter
    def calibration(self, offsets):
        """
        设置硬磁偏移。

        Args:
            offsets: 长度 3 的序列，单位 LSB。

        Raises:
            ValueError: 长度不为 3 或元素非数字。
        """
        if len(offsets) != 3:
            raise ValueError("calibration must be a 3-element sequence")
        try:
            offset_x = float(offsets[0])
            offset_y = float(offsets[1])
            offset_z = float(offsets[2])
        except (TypeError, ValueError):
            raise ValueError("calibration elements must be numbers")
        self._offset_x = offset_x
        self._offset_y = offset_y
        self._offset_z = offset_z

    @property
    def raw(self):
        """
        最近一次读取的原始 20 位计数 (raw_x, raw_y, raw_z)。

        未读取过时为 (0, 0, 0)。不涉及 I2C。
        """
        return (self._raw_x, self._raw_y, self._raw_z)

    # ------------------------------------------------------------------
    # 内部：解析（无副作用，便于单测）
    # ------------------------------------------------------------------

    @staticmethod
    def _parse_raw(buf):
        """
        解析 MMC5603 的 9 字节 20 位数据布局。无副作用。

        字节布局（参考数据手册寄存器 0x00..0x08）：
            X[19:12] = buf[0]
            X[11:4]  = buf[1]
            X[3:0]   = buf[6] >> 4
            Y[19:12] = buf[2]
            Y[11:4]  = buf[3]
            Y[3:0]   = buf[7] >> 4
            Z[19:12] = buf[4]
            Z[11:4]  = buf[5]
            Z[3:0]   = buf[8] >> 4

        Args:
            buf (bytearray | bytes): 长度 >= 9 的缓冲区。

        Returns:
            tuple: (raw_x, raw_y, raw_z)，每个 0..2**20-1。
        """
        raw_x = (buf[0] << 12) | (buf[1] << 4) | (buf[6] >> 4)
        raw_y = (buf[2] << 12) | (buf[3] << 4) | (buf[7] >> 4)
        raw_z = (buf[4] << 12) | (buf[5] << 4) | (buf[8] >> 4)
        return (raw_x, raw_y, raw_z)

    # ------------------------------------------------------------------
    # 内部：I2C 原语（带重试 + deinit 检查）
    # ------------------------------------------------------------------

    def _check_deinit(self):
        """
        若设备已 deinit 则抛 RuntimeError。

        Raises:
            RuntimeError: 设备已 deinit。
        """
        if self._i2c is None:
            raise RuntimeError("device is deinitialized")

    def _wreg(self, reg, value):
        """
        写单字节寄存器，失败按 retries 重试。

        Raises:
            OSError: 连续 retries+1 次失败。
            RuntimeError: 设备已 deinit。
        """
        self._check_deinit()
        self._tx_buf[0] = value
        last_error = None
        for attempt in range(self._retries + 1):
            try:
                self._i2c.writeto_mem(self._addr, reg, self._tx_buf)
                return
            except OSError as exc:
                last_error = exc
                if attempt < self._retries:
                    time.sleep_ms(self._retry_delay_ms)
        raise last_error

    def _read_into(self, reg, buf):
        """
        读 len(buf) 字节到 buf（复用缓冲区，无临时分配）。

        Raises:
            OSError: 连续 retries+1 次失败。
            RuntimeError: 设备已 deinit。
        """
        self._check_deinit()
        last_error = None
        for attempt in range(self._retries + 1):
            try:
                self._i2c.readfrom_mem_into(self._addr, reg, buf)
                return buf
            except OSError as exc:
                last_error = exc
                if attempt < self._retries:
                    time.sleep_ms(self._retry_delay_ms)
        raise last_error

    def _read_reg(self, reg):
        """读单个寄存器，返回 int。"""
        return self._read_into(reg, self._rx_buf)[0]

    # ------------------------------------------------------------------
    # 内部：设备配置
    # ------------------------------------------------------------------

    def _chip_id(self):
        """读器件 ID 寄存器（0x39）。"""
        return self._read_reg(_REG_DEVICE_ID)

    def _auto_selftest(self):
        """
        按出厂 trim 值配置 ST 阈值寄存器 0x1E/0x1F/0x20。

        算法与 MEMSIC 官方示例一致：
            delta = |trim - 128| * 32
            threshold = (delta - delta // 5) // 8，上限 255
        """
        self._read_into(_REG_ST_X_VAL, self._st_buf)
        for axis in range(_ST_TRIM_BYTES):
            delta = (self._st_buf[axis] - _ST_TRIM_CENTER) * _ST_TRIM_SCALE
            if delta < 0:
                delta = -delta
            threshold = (delta - delta // 5) // 8
            if threshold > _ST_THD_MAX:
                threshold = _ST_THD_MAX
            self._wreg(_REG_X_THD + axis, threshold)

    def _set(self):
        """执行一次手动 SET，复位磁芯磁化方向（阻塞 _SET_DELAY_MS）。"""
        self._wreg(_REG_CTRL0, _CTRL0_SET)
        time.sleep_ms(_SET_DELAY_MS)

    def _start_continuous(self, bandwidth=_BANDWIDTH_DEFAULT, odr_hz=_ODR_HZ_DEFAULT):
        """
        进入连续测量模式。

        Args:
            bandwidth (int): CTRL1 带宽档位，默认 0x00。
            odr_hz (int): ODR 寄存器值，默认 50。
        """
        self._wreg(_REG_CTRL1, bandwidth)
        self._wreg(_REG_ODR, odr_hz)
        self._wreg(_REG_CTRL0, _CTRL0_CONTINUOUS)
        self._wreg(_REG_CTRL2, _CTRL2_CMM_FREQ_EN)
