"""
MMC5603 三轴磁力计 MicroPython 驱动

依赖:
    math, time, micropython.const
目标固件:
    MicroPython v1.19+（ESP32 / RP2040 / STM32 / ESP8266）

坐标系与换算（右手系，芯片安装方向以数据手册为准）:
    raw20  = (MSB << 12) | (MID << 4) | (LSB >> 4)     # 20 位无符号
    gauss  = _SENSITIVITY * (raw20 - offset)
             _SENSITIVITY = 0.0625（厂商示例系数，物理单位详见数据手册）
    offset 默认 1 << 19 = 524288，为 20 位输出中点（硬铁零点）。

Yaw / Heading:
    水平安装（upright=False）: yaw 基于 XY 平面，atan2(y, -x)
    竖直安装（upright=True）:  yaw 基于 XZ 平面，atan2(z, -x)
    declination 为磁偏角（度，东正西负）。
    get_yaw 返回 -180..180；get_heading 返回 0..360。

用法（最小示例）:
    from machine import I2C, Pin
    import time
    from mmc5603 import MMC5603

    i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
    mag = MMC5603(i2c)
    while True:
        gx, gy, gz = mag.read()
        print("mag=%.3f,%.3f,%.3f  strength=%.3f  heading=%.1f"
              % (gx, gy, gz, mag.get_strength(), mag.get_heading()))
        time.sleep_ms(200)

用法（校准 + 持久化）:
    offsets = mag.calibrate(duration_ms=20000,
                            progress_callback=lambda p: print(p, "%"))
    print("offsets:", offsets)
    # 调用方自行持久化:
    with open("magnetic_cal.py", "w") as f:
        f.write("_offset_x=%r\n_offset_y=%r\n_offset_z=%r\n" % offsets)

用法（下次启动加载）:
    try:
        import magnetic_cal
        cal = (magnetic_cal._offset_x,
               magnetic_cal._offset_y,
               magnetic_cal._offset_z)
    except ImportError:
        cal = None
    mag = MMC5603(i2c, calibration=cal)

用法（错误处理）:
    def on_error(exc):
        print("MMC5603 error:", exc)
    mag = MMC5603(i2c, error_handler=on_error)

用法（低层，高频）:
    rx, ry, rz = mag.read_raw()     # 原始 20 位整数，无 float
"""

import math
import time

from micropython import const

__all__ = ("MMC5603",)

# ==================== I2C 地址 ====================
MMC5603_DEFAULT_ADDR = const(0x30)

# ==================== 寄存器地址 ====================
MMC5603_REG_DATA = const(0x00)
MMC5603_REG_TMP = const(0x09)
MMC5603_REG_ODR = const(0x1A)
MMC5603_REG_CTRL0 = const(0x1B)
MMC5603_REG_CTRL1 = const(0x1C)
MMC5603_REG_CTRL2 = const(0x1D)
MMC5603_REG_X_THD = const(0x1E)
MMC5603_REG_Y_THD = const(0x1F)
MMC5603_REG_Z_THD = const(0x20)
MMC5603_REG_ST_X_VAL = const(0x27)
MMC5603_REG_DEVICE_ID = const(0x39)

# ==================== 芯片 ID ====================
MMC5603_CHIP_ID = const(0x10)

# ==================== CTRL0 位 ====================
CTRL0_SET = const(0x08)  # SET 线圈操作
CTRL0_TAKE_MEAS = const(0x01)  # 单次测量
CTRL0_AUTO_SR = const(0x20)  # 自动置位/复位
CTRL0_CMM_FREQ = const(0x80)  # 连续测量模式

# ==================== CTRL2 位 ====================
CTRL2_HPOWER = const(0x80)  # 高功率
CTRL2_AUTO_ST = const(0x10)  # 自动自检

# ==================== 换算 / 内部常量 ====================
# 20 位输出，零点偏移 = 2^19
_OFFSET_DEFAULT = const(1 << 19)  # 524288

# 灵敏度系数（厂商示例公式系数）；MicroPython const() 不支持 float
_SENSITIVITY = 0.0625

# 自检阈值计算
_ST_TRIM_BIAS = const(128)
_ST_TRIM_SCALE = const(32)
_ST_DIV_5 = const(5)
_ST_DIV_8 = const(8)
_ST_MAX = const(0xFF)

# 缓冲区 / 长度
_DATA_LEN = const(9)
_ST_TRIM_LEN = const(3)

# SET 线圈操作后等待（ms）
_SET_DELAY_MS = const(5)

# 进入连续模式后等待（ms）
_START_DELAY_MS = const(50)

# 自检阈值寄存器按 X/Y/Z 顺序
_ST_THD_REGS = (MMC5603_REG_X_THD, MMC5603_REG_Y_THD, MMC5603_REG_Z_THD)


class MMC5603:
    """
    MMC5603 三轴磁力计驱动类。

    属性:
        raw_x, raw_y, raw_z (int): 最近一次读到的 20 位原始值。
        last_error (Exception | None): 最近一次后台错误。
    """

    __slots__ = (
        "_i2c",
        "_addr",
        "_offset_x",
        "_offset_y",
        "_offset_z",
        "_raw_x",
        "_raw_y",
        "_raw_z",
        "_last_error",
        "_error_handler",
        "_retries",
        "_retry_delay_ms",
        "_deinit",
        "_buf",
    )

    def __init__(
        self,
        i2c_bus,
        addr=MMC5603_DEFAULT_ADDR,
        calibration=None,
        error_handler=None,
        retries=2,
        retry_delay_ms=5,
    ):
        """
        Args:
            i2c_bus (I2C): 已初始化的 I2C 实例（外部注入，不可为 None）。
            addr (int): I2C 地址，默认 0x30。
            calibration (tuple | None): 硬铁偏移 (ox, oy, oz)；
                None 使用默认零点 1 << 19。
            error_handler (callable | None): 错误回调 fn(exc)。
            retries (int): I2C 重试次数，默认 2（总共尝试 retries+1 次）。
            retry_delay_ms (int): 重试间隔 ms，默认 5。

        Raises:
            ValueError: i2c 为 None 或 calibration 长度不为 3。
            OSError: 芯片 ID 不匹配或 I2C 通信失败。
        """
        if i2c_bus is None:
            raise ValueError("i2c_bus must not be None")

        self._i2c = i2c_bus
        self._addr = addr

        if calibration is None:
            self._offset_x = _OFFSET_DEFAULT
            self._offset_y = _OFFSET_DEFAULT
            self._offset_z = _OFFSET_DEFAULT
        else:
            if len(calibration) != 3:
                raise ValueError("calibration must be (offset_x, offset_y, offset_z)")
            self._offset_x = calibration[0]
            self._offset_y = calibration[1]
            self._offset_z = calibration[2]

        self._raw_x = 0
        self._raw_y = 0
        self._raw_z = 0

        self._last_error = None
        self._error_handler = error_handler
        self._retries = retries
        self._retry_delay_ms = retry_delay_ms
        self._deinit = False
        self._buf = bytearray(_DATA_LEN)

        if self._chip_id() != MMC5603_CHIP_ID:
            raise OSError("MMC5603 not found at I2C address 0x%02X" % addr)

        self._auto_selftest()
        self._set_coil()
        self._configure_continuous()
        time.sleep_ms(_START_DELAY_MS)

    # ==================== 只读属性 ====================
    @property
    def raw_x(self):
        """最近一次读到的 X 轴 20 位原始值。"""
        return self._raw_x

    @property
    def raw_y(self):
        """最近一次读到的 Y 轴 20 位原始值。"""
        return self._raw_y

    @property
    def raw_z(self):
        """最近一次读到的 Z 轴 20 位原始值。"""
        return self._raw_z

    @property
    def last_error(self):
        """最近一次后台错误。"""
        return self._last_error

    # ==================== 内部工具 ====================
    def _check_deinit(self):
        if self._deinit:
            raise RuntimeError("device is deinitialized")

    def _call_error_handler(self, exc):
        """保存并分发错误。"""
        self._last_error = exc
        handler = self._error_handler
        if handler is not None:
            try:
                handler(exc)
            except Exception as nested:
                self._last_error = nested

    # ==================== 底层 I2C ====================
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
            int (nbytes==1) 或 bytes (nbytes>1)。
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
        return self._read_reg(MMC5603_REG_DEVICE_ID)

    # ==================== 初始化辅助 ====================
    def _auto_selftest(self):
        """
        自动配置 SET/RESET 自检阈值（写 X/Y/Z_THD）。

        Notes:
            阈值由芯片内部 trim 数据推导，用于 SET/RESET 线圈驱动的
            阈值判断。阈值超上限时钳位到 0xFF。
        """
        trim = self._read_reg(MMC5603_REG_ST_X_VAL, _ST_TRIM_LEN)
        for i in range(_ST_TRIM_LEN):
            delta = (trim[i] - _ST_TRIM_BIAS) * _ST_TRIM_SCALE
            if delta < 0:
                delta = -delta
            st_thd = (delta - delta // _ST_DIV_5) // _ST_DIV_8
            if st_thd > _ST_MAX:
                st_thd = _ST_MAX
            self._write_reg(_ST_THD_REGS[i], st_thd)

    def _set_coil(self):
        """执行 SET 线圈操作（磁滞复位），并等待线圈稳定。"""
        self._write_reg(MMC5603_REG_CTRL0, CTRL0_SET)
        time.sleep_ms(_SET_DELAY_MS)

    def _configure_continuous(self, bandwidth=0x00, sample_rate_hz=50):
        """
        配置连续测量模式。

        Args:
            bandwidth (int): CTRL1 带宽设置（0x00 为默认）。
            sample_rate_hz (int): ODR 寄存器值（0..255，值越大速率越高）。
        """
        self._write_reg(MMC5603_REG_CTRL1, bandwidth)
        self._write_reg(MMC5603_REG_ODR, sample_rate_hz)
        self._write_reg(MMC5603_REG_CTRL0, CTRL0_CMM_FREQ | CTRL0_AUTO_SR)
        self._write_reg(MMC5603_REG_CTRL2, CTRL2_AUTO_ST)

    # ==================== 读取 API ====================
    def read_raw(self):
        """
        读取原始 20 位三轴数据并缓存。

        Returns:
            tuple: (raw_x, raw_y, raw_z) 原始整数。

        Raises:
            OSError: I2C 读取失败。
        """
        self._read_reg_into(MMC5603_REG_DATA, self._buf)
        b = self._buf
        self._raw_x = (b[0] << 12) | (b[1] << 4) | (b[6] >> 4)
        self._raw_y = (b[2] << 12) | (b[3] << 4) | (b[7] >> 4)
        self._raw_z = (b[4] << 12) | (b[5] << 4) | (b[8] >> 4)
        return self._raw_x, self._raw_y, self._raw_z

    def read(self):
        """
        读取并换算为物理量（减偏移、乘灵敏度系数）。

        Returns:
            tuple: (gx, gy, gz)，单位参见模块 docstring。

        Raises:
            OSError: I2C 读取失败。
        """
        rx, ry, rz = self.read_raw()
        return (
            -_SENSITIVITY * (rx - self._offset_x),
            -_SENSITIVITY * (ry - self._offset_y),
            -_SENSITIVITY * (rz - self._offset_z),
        )

    def get_data(self):
        """[兼容别名] 请使用 read()。将在 v2.0 移除。"""
        return self.read()

    # ==================== 校准 API ====================
    def set_calibration(self, offset_x, offset_y, offset_z):
        """
        设置硬铁偏移。

        Args:
            offset_x, offset_y, offset_z (int | float): 20 位原始值域偏移。
        """
        self._offset_x = offset_x
        self._offset_y = offset_y
        self._offset_z = offset_z

    def get_calibration(self):
        """
        返回当前硬铁偏移。

        Returns:
            tuple: (offset_x, offset_y, offset_z)
        """
        return self._offset_x, self._offset_y, self._offset_z

    def calibrate(
        self, duration_ms=20000, sample_interval_ms=20, progress_callback=None
    ):
        """
        采集球面样本并计算硬铁偏移（8 字校准）。

        Args:
            duration_ms (int): 校准持续时间 ms，默认 20000。
            sample_interval_ms (int): 采样间隔 ms，默认 20。
            progress_callback (callable | None): 进度回调 fn(progress)，
                progress 为 0..100 的整数；异常走 error_handler。

        Returns:
            tuple: (offset_x, offset_y, offset_z)，同时写入实例。

        Notes:
            - 校准期间需在空气中缓慢转动板子形成 8 字。
            - 本方法不写文件、不 print；持久化由调用方负责。
            - 采样间隔应 ≥ 20ms，避免超过 I2C 总线吞吐。
        """
        self._check_deinit()

        self.read_raw()
        min_x = max_x = self._raw_x
        min_y = max_y = self._raw_y
        min_z = max_z = self._raw_z

        start = time.ticks_ms()
        last_report = -1
        while time.ticks_diff(time.ticks_ms(), start) < duration_ms:
            self.read_raw()
            if self._raw_x < min_x:
                min_x = self._raw_x
            if self._raw_x > max_x:
                max_x = self._raw_x
            if self._raw_y < min_y:
                min_y = self._raw_y
            if self._raw_y > max_y:
                max_y = self._raw_y
            if self._raw_z < min_z:
                min_z = self._raw_z
            if self._raw_z > max_z:
                max_z = self._raw_z
            time.sleep_ms(sample_interval_ms)

            if progress_callback is not None:
                elapsed = time.ticks_diff(time.ticks_ms(), start)
                progress = elapsed * 100 // duration_ms
                if progress != last_report:
                    last_report = progress
                    try:
                        progress_callback(progress)
                    except Exception as exc:
                        self._call_error_handler(exc)

        self._offset_x = (max_x + min_x) / 2
        self._offset_y = (max_y + min_y) / 2
        self._offset_z = (max_z + min_z) / 2
        return self._offset_x, self._offset_y, self._offset_z

    # ==================== 物理量计算 ====================
    def get_strength(self):
        """
        返回磁场强度（单位与 read() 一致）。

        Returns:
            float: sqrt(gx^2 + gy^2 + gz^2)。
        """
        x, y, z = self.read()
        return math.sqrt(x * x + y * y + z * z)

    def get_yaw(self, upright=False, declination=0.0):
        """
        计算偏航角。

        Args:
            upright (bool): True 使用 XZ 平面（竖直安装）；
                            False 使用 XY 平面（水平安装）。
            declination (float): 磁偏角（度，东正西负）。

        Returns:
            float: yaw 角，范围 -180..180（度）。
        """
        x, y, z = self.read()
        if upright:
            angle = math.degrees(math.atan2(z, -x)) + 180.0
        else:
            angle = math.degrees(math.atan2(y, -x)) + 180.0
        return (angle + declination + 180.0) % 360.0 - 180.0

    def get_heading(self, upright=False, declination=0.0):
        """
        计算航向角。

        Args:
            upright (bool): 同 get_yaw。
            declination (float): 磁偏角（度）。

        Returns:
            float: 航向角，范围 0..360（度）。
        """
        return self.get_yaw(upright, declination) % 360.0

    # ==================== 资源释放 ====================
    def deinit(self):
        """
        释放资源：清空 I2C 引用，置 deinit 标志。幂等。

        Notes:
            deinit 后调用任何 I2C 方法抛 RuntimeError。
        """
        if self._deinit:
            return
        self._i2c = None
        self._deinit = True
