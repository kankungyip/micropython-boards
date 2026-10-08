"""
QMI8658C 6-axis IMU MicroPython Driver
- 引脚中断完全集成: 只需传引脚编号或 Pin 对象, 内部自动初始化为输入
- 统一的中断绑定 API: attach_interrupt / detach_interrupt
- 专用事件配置: xxx_interrupt(..., handler, trigger)
                内部调用 attach_interrupt

用法:
    imu = QMI8658(i2c, int1_pin=5, int2_pin=4)   # 直接传引脚编号
    imu.tap_interrupt(handler=on_event)   # 配置 + 绑定
    # 或通用绑定:
    imu.attach_interrupt(INT_PIN_1, on_event)
"""

import math
import time

import machine
from micropython import const

# ==================== 寄存器地址 ====================
QMI8658_REG_DEVICE_ID = const(0x00)
QMI8658_REG_Ctrl1 = const(0x02)
QMI8658_REG_Ctrl2 = const(0x03)
QMI8658_REG_Ctrl3 = const(0x04)
QMI8658_REG_Ctrl5 = const(0x06)
QMI8658_REG_Ctrl7 = const(0x08)
QMI8658_REG_Ctrl8 = const(0x09)
QMI8658_REG_Ctrl9 = const(0x0A)
QMI8658_REG_Cal1_L = const(0x0B)
QMI8658_REG_Cal1_H = const(0x0C)
QMI8658_REG_Cal3_L = const(0x0F)
QMI8658_REG_Cal3_H = const(0x10)
QMI8658_REG_FifoWtmTh = const(0x13)
QMI8658_REG_FifoCtrl = const(0x14)
QMI8658_REG_FifoSmplCnt = const(0x15)
QMI8658_REG_FifoStatus = const(0x16)
QMI8658_REG_FifoData = const(0x17)
QMI8658_REG_StatusInt = const(0x2D)
QMI8658_REG_StepCntLow = const(0x45)
QMI8658_REG_StepCntMid = const(0x46)
QMI8658_REG_StepCntHigh = const(0x47)
QMI8658_REG_TapStatus = const(0x59)
QMI8658_REG_DATA = const(0x33)

# ==================== 量程常量 ====================
Qmi8658AccRange_2g = 0
Qmi8658AccRange_4g = 1
Qmi8658AccRange_8g = 2
Qmi8658AccRange_16g = 3

Qmi8658GyrRange_16dps = 0
Qmi8658GyrRange_32dps = 1
Qmi8658GyrRange_64dps = 2
Qmi8658GyrRange_128dps = 3
Qmi8658GyrRange_256dps = 4
Qmi8658GyrRange_512dps = 5
Qmi8658GyrRange_1024dps = 6
Qmi8658GyrRange_2048dps = 7

Qmi8658Acc_Gyr_Odr_8000Hz = 0x00
Qmi8658Acc_Gyr_Odr_4000Hz = 0x01
Qmi8658Acc_Gyr_Odr_2000Hz = 0x02
Qmi8658Acc_Gyr_Odr_1000Hz = 0x03
Qmi8658Acc_Gyr_Odr_500Hz = 0x04
Qmi8658Acc_Gyr_Odr_250Hz = 0x05
Qmi8658Acc_Gyr_Odr_125Hz = 0x06
Qmi8658Acc_Gyr_Odr_62_5Hz = 0x07
Qmi8658Acc_Gyr_Odr_31_25Hz = 0x08

# ==================== 中断引脚常量 ====================
INT_PIN_1 = const(1)
INT_PIN_2 = const(2)

# ==================== FIFO 模式 ====================
FIFO_MODE_BYPASS = const(0x00)
FIFO_MODE_FIFO = const(0x01)
FIFO_MODE_STREAM = const(0x02)

# ==================== CTRL9 命令 ====================
CTRL9_CMD_RST_FIFO = const(0x04)
CTRL9_CMD_REQ_FIFO = const(0x0D)
CTRL9_CMD_WRITE_WOM_SETTING = const(0x08)
CTRL9_CMD_RESET_PEDOMETER = const(0x0F)

# ==================== STATUSINT 位定义 ====================
STATUSINT_BIT_DRDY = const(1 << 0)
STATUSINT_BIT_TAP = const(1 << 1)
STATUSINT_BIT_WOM = const(1 << 2)
STATUSINT_BIT_SIGMOTION = const(1 << 3)
STATUSINT_BIT_ANYMOTION = const(1 << 4)
STATUSINT_BIT_FIFO = const(1 << 5)
STATUSINT_BIT_CMD_DONE = const(1 << 7)


class QMI8658:
    def __init__(
        self,
        i2c_bus,
        addr=0x6B,
        int1_pin=None,
        int2_pin=None,
        AccRange=Qmi8658AccRange_8g,
        GyrRange=Qmi8658GyrRange_2048dps,
        Acc_Gyr_Odr=Qmi8658Acc_Gyr_Odr_500Hz,
    ):
        self._device = i2c_bus
        self._address = addr

        # ---- 中断引脚: 支持 int / Pin / None ----
        self._int_pins = {
            INT_PIN_1: self._normalize_pin(int1_pin),
            INT_PIN_2: self._normalize_pin(int2_pin),
        }

        # 每个引脚上注册的回调列表: [(event_mask, callback), ...]
        self._int_callbacks = {INT_PIN_1: [], INT_PIN_2: []}
        # 每个引脚的触发沿
        self._int_triggers = {
            INT_PIN_1: machine.Pin.IRQ_FALLING,
            INT_PIN_2: machine.Pin.IRQ_FALLING,
        }
        # 引脚是否已绑定 Pin.irq
        self._int_bound = {INT_PIN_1: False, INT_PIN_2: False}

        if self._chip_id() != 0x05:
            raise AttributeError("Cannot find a QMI8658")
        self.init(AccRange, GyrRange, Acc_Gyr_Odr)

    # ==================== 引脚归一化 ====================
    @staticmethod
    def _normalize_pin(pin):
        """
        将参数归一化为 machine.Pin 输入引脚:
          - None         -> None
          - int          -> machine.Pin(num, Pin.IN, Pin.PULL_UP)
          - machine.Pin  -> 若未配置为输入, 重新配置为 IN + PULL_UP
        """
        if pin is None:
            return None
        if isinstance(pin, int):
            return machine.Pin(pin, machine.Pin.IN, machine.Pin.PULL_UP)
        try:
            pin.init(machine.Pin.IN, machine.Pin.PULL_UP)
        except Exception:
            pass
        return pin

    # ==================== 底层 I2C 读写 ====================
    def _wreg(self, reg, val):
        self._device.writeto_mem(self._address, reg, val.to_bytes(1, "little"))

    def _rreg(self, reg, nbytes=1):
        if nbytes <= 1:
            return self._device.readfrom_mem(self._address, reg, 1)[0]
        return self._device.readfrom_mem(self._address, reg, nbytes)

    def _chip_id(self):
        return self._rreg(QMI8658_REG_DEVICE_ID)

    # ==================== 初始化 ====================
    def init(
        self,
        AccRange=Qmi8658AccRange_8g,
        GyrRange=Qmi8658GyrRange_2048dps,
        Acc_Gyr_Odr=Qmi8658Acc_Gyr_Odr_500Hz,
        drdy_int_pin=INT_PIN_2,
        int_pushpull=True,
    ):
        # 软复位
        self._wreg(QMI8658_REG_Ctrl9, 0xA2)
        time.sleep(0.1)

        # --- CTRL1: 中断引脚输出模式 ---
        # bit3: INT1 推挽(1)/高阻(0)
        # bit4: INT2 推挽(1)/高阻(0)
        # bit2: FIFO 中断路由(1=INT1, 0=INT2)
        # bit1: 地址自动递增
        ctrl1_val = 0x40 | 0x02
        if int_pushpull:
            ctrl1_val |= (1 << 3) | (1 << 4)
        self._wreg(QMI8658_REG_Ctrl1, ctrl1_val)

        # --- CTRL7: 传感器 + DRDY 使能 ---
        self._wreg(QMI8658_REG_Ctrl7, 0x03)

        # --- CTRL8: 运动事件中断路由 ---
        ctrl8_val = 0x00
        if drdy_int_pin == INT_PIN_1:
            ctrl8_val |= 1 << 6
        self._wreg(QMI8658_REG_Ctrl8, ctrl8_val)
        self._ctrl8_val = ctrl8_val

        # --- CTRL2/3/5 ---
        self._wreg(QMI8658_REG_Ctrl2, (AccRange << 4) | Acc_Gyr_Odr)
        self._wreg(QMI8658_REG_Ctrl3, (GyrRange << 4) | Acc_Gyr_Odr)
        self._wreg(QMI8658_REG_Ctrl5, 0x75)

        # 换算系数
        self.gyr_lsb_div = 2 ** (11 - GyrRange)
        self.acc_lsb_div = 1 << (14 - AccRange)
        self._drdy_int_pin = drdy_int_pin

    # ================================================================
    #  中断 API（与 PCF85063 统一命名）
    # ================================================================
    def attach_interrupt(self, pin_num, handler=None, trigger=None, event_mask=0xFF):
        """
        [统一接口] 在指定引脚上注册回调。

        Args:
            pin_num:    INT_PIN_1 或 INT_PIN_2
            handler:    用户回调 fn(events, status)
                        events = ["Tap", "DRDY", ...] 事件名列表
                        status = STATUSINT 原始值
            trigger:    触发沿, None 表示沿用当前设置
            event_mask: 只在这些事件位被置位时才调用回调 (0xFF=全部)

        Returns:
            bool: 是否成功绑定 (引脚未配置时为 False)

        Notes:
            - 若该引脚此前已绑定过 ISR, 再次调用只追加回调, 不重复绑定。
            - 可在 xxx_interrupt() 之外单独调用, 用于通用场景。
        """
        if handler is not None:
            self._int_callbacks[pin_num].append((event_mask, handler))

        if trigger is not None:
            self._int_triggers[pin_num] = trigger

        # 已绑定就不再重复绑定
        if self._int_bound[pin_num]:
            return True

        pin = self._int_pins.get(pin_num)
        if pin is None:
            return False  # 用户没提供这个引脚, 静默跳过

        pin.irq(
            trigger=self._int_triggers[pin_num],
            handler=self._make_isr(pin_num),
        )
        self._int_bound[pin_num] = True
        return True

    def _make_isr(self, pin_num):
        """为指定引脚生成 ISR, 内部读 status 并分派给已注册回调"""

        def _isr(_pin_obj):
            try:
                events, status = self.parse_interrupt_status()
            except Exception:
                return
            for mask, cb in list(self._int_callbacks[pin_num]):
                if mask == 0 or (status & mask):
                    try:
                        cb(events, status)
                    except Exception:
                        pass

        return _isr

    def detach_interrupt(self, pin_num=None):
        """
        [统一接口] 解除引脚中断绑定及所有回调。

        Args:
            pin_num: INT_PIN_1 / INT_PIN_2，None 表示解绑所有引脚。

        Notes:
            未配置或未绑定时为空操作。
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
        return self._rreg(QMI8658_REG_StatusInt)

    def u2s(self, n):
        return n if n < (1 << 15) else n - (1 << 16)

    def get_data(self):
        try:
            _buffer = self._rreg(QMI8658_REG_DATA, 14)
            tmp = float(self.u2s(_buffer[1] << 8 | _buffer[0])) / 256.0
            acc_x = float(self.u2s(_buffer[3] << 8 | _buffer[2])) / self.acc_lsb_div
            acc_y = float(self.u2s(_buffer[5] << 8 | _buffer[4])) / self.acc_lsb_div
            acc_z = float(self.u2s(_buffer[7] << 8 | _buffer[6])) / self.acc_lsb_div
            gyr_x = float(self.u2s(_buffer[9] << 8 | _buffer[8])) / self.gyr_lsb_div
            gyr_y = float(self.u2s(_buffer[11] << 8 | _buffer[10])) / self.gyr_lsb_div
            gyr_z = float(self.u2s(_buffer[13] << 8 | _buffer[12])) / self.gyr_lsb_div
            return (acc_x, acc_y, acc_z), (gyr_x, gyr_y, gyr_z), round(tmp, 2)
        except Exception:
            return (0, 0, 9.8), (0, 0, 0), 0

    def get_accelerometer(self):
        return self.get_data()[0]

    def get_gyroscope(self):
        return self.get_data()[1]

    def get_temperature(self):
        return self.get_data()[2]

    def get_strength(self):
        _x, _y, _z = self.get_accelerometer()
        return math.sqrt(_x * _x + _y * _y + _z * _z)

    def get_pitch_roll(self):
        acc_x, acc_y, acc_z = self.get_accelerometer()
        pitch = math.atan2(-acc_x, math.sqrt(acc_y * acc_y + acc_z * acc_z))
        roll = math.atan2(acc_y, acc_z)
        return math.degrees(pitch), math.degrees(roll)

    # ================================================================
    #  专用事件配置（内部调用 attach_interrupt 完成引脚绑定）
    # ================================================================

    # ---------- 1. DRDY ----------
    def drdy_interrupt(
        self,
        enable=True,
        handler=None,
        trigger=machine.Pin.IRQ_RISING,
    ):
        """数据就绪中断, 固定输出到 INT2 (FIFO 需处于 Bypass 模式)"""
        ctrl7 = self._rreg(QMI8658_REG_Ctrl7)
        if enable:
            ctrl7 &= ~(1 << 5)
            self._wreg(QMI8658_REG_FifoCtrl, FIFO_MODE_BYPASS)
        else:
            ctrl7 |= 1 << 5
        self._wreg(QMI8658_REG_Ctrl7, ctrl7)

        self.attach_interrupt(INT_PIN_2, handler, trigger, STATUSINT_BIT_DRDY)

    # ---------- 2. FIFO ----------
    def fifo_interrupt(
        self,
        int_pin=INT_PIN_2,
        watermark=0x20,
        fifo_mode=FIFO_MODE_FIFO,
        handler=None,
        trigger=machine.Pin.IRQ_FALLING,
    ):
        self._wreg(QMI8658_REG_FifoWtmTh, watermark)

        fifo_ctrl = (fifo_mode & 0x03) | (1 << 2)
        self._wreg(QMI8658_REG_FifoCtrl, fifo_ctrl)

        ctrl1 = self._rreg(QMI8658_REG_Ctrl1)
        if int_pin == INT_PIN_1:
            ctrl1 |= 1 << 2
        else:
            ctrl1 &= ~(1 << 2)
        self._wreg(QMI8658_REG_Ctrl1, ctrl1)

        self.attach_interrupt(int_pin, handler, trigger, STATUSINT_BIT_FIFO)

    def read_fifo(self, max_samples=32):
        self._wreg(QMI8658_REG_Ctrl9, CTRL9_CMD_REQ_FIFO)
        time.sleep(0.01)

        smpl_cnt = self._rreg(QMI8658_REG_FifoSmplCnt)
        count = min(smpl_cnt, max_samples)

        results = []
        for _ in range(count):
            buf = self._rreg(QMI8658_REG_FifoData, 12)
            tmp = float(self.u2s(buf[1] << 8 | buf[0])) / 256.0
            acc = (
                float(self.u2s(buf[3] << 8 | buf[2])) / self.acc_lsb_div,
                float(self.u2s(buf[5] << 8 | buf[4])) / self.acc_lsb_div,
                float(self.u2s(buf[7] << 8 | buf[6])) / self.acc_lsb_div,
            )
            gyr = (
                float(self.u2s(buf[9] << 8 | buf[8])) / self.gyr_lsb_div,
                float(self.u2s(buf[11] << 8 | buf[10])) / self.gyr_lsb_div,
                float(self.u2s(buf[13] << 8 | buf[12])) / self.gyr_lsb_div,
            )
            results.append((acc, gyr, round(tmp, 2)))

        self._wreg(QMI8658_REG_Ctrl9, CTRL9_CMD_RST_FIFO)
        time.sleep(0.01)
        return results

    # ---------- 3. 运动事件 (Any/Sig-Motion) ----------
    def motion_interrupt(
        self,
        enable_any=True,
        enable_sig=True,
        int_pin=INT_PIN_1,
        sig_window=0x64,
        handler=None,
        trigger=machine.Pin.IRQ_FALLING,
    ):
        ctrl8 = self._ctrl8_val

        if int_pin == INT_PIN_1:
            ctrl8 |= 1 << 6
        else:
            ctrl8 &= ~(1 << 6)

        if enable_any:
            ctrl8 |= 1 << 0
        else:
            ctrl8 &= ~(1 << 0)

        if enable_sig:
            ctrl8 |= 1 << 1
        else:
            ctrl8 &= ~(1 << 1)

        self._wreg(QMI8658_REG_Cal3_L, sig_window & 0xFF)
        self._wreg(QMI8658_REG_Cal3_H, (sig_window >> 8) & 0xFF)

        self._wreg(QMI8658_REG_Ctrl8, ctrl8)
        self._ctrl8_val = ctrl8

        mask = STATUSINT_BIT_ANYMOTION | STATUSINT_BIT_SIGMOTION
        self.attach_interrupt(int_pin, handler, trigger, mask)

    # ---------- 4. 敲击检测 (Tap) ----------
    def tap_interrupt(
        self,
        enable=True,
        int_pin=INT_PIN_1,
        threshold=0x40,
        double_tap=True,
        handler=None,
        trigger=machine.Pin.IRQ_FALLING,
    ):
        ctrl8 = self._ctrl8_val

        if int_pin == INT_PIN_1:
            ctrl8 |= 1 << 6
        else:
            ctrl8 &= ~(1 << 6)

        if enable:
            ctrl8 |= 1 << 3
        else:
            ctrl8 &= ~(1 << 3)

        self._wreg(QMI8658_REG_Ctrl8, ctrl8)
        self._ctrl8_val = ctrl8

        self._wreg(QMI8658_REG_Cal1_L, threshold & 0xFF)

        cal1_h = self._rreg(QMI8658_REG_Cal1_H)
        if double_tap:
            cal1_h |= 1 << 6
        else:
            cal1_h &= ~(1 << 6)
        self._wreg(QMI8658_REG_Cal1_H, cal1_h)

        self.attach_interrupt(int_pin, handler, trigger, STATUSINT_BIT_TAP)

    def read_tap_status(self):
        val = self._rreg(QMI8658_REG_TapStatus)
        axis = val & 0x03
        polarity = (val >> 2) & 0x01
        taps = (val >> 3) & 0x03
        return axis, polarity, taps

    # ---------- 5. 计步器 ----------
    def pedometer_interrupt(
        self,
        enable=True,
        int_pin=INT_PIN_1,
        handler=None,
        trigger=machine.Pin.IRQ_FALLING,
    ):
        ctrl8 = self._ctrl8_val

        if int_pin == INT_PIN_1:
            ctrl8 |= 1 << 6
        else:
            ctrl8 &= ~(1 << 6)

        if enable:
            ctrl8 |= 1 << 4
        else:
            ctrl8 &= ~(1 << 4)

        self._wreg(QMI8658_REG_Ctrl8, ctrl8)
        self._ctrl8_val = ctrl8

        # 计步器没有独立 statusint 位, 用 0xFF 保证回调总被调用
        self.attach_interrupt(int_pin, handler, trigger, 0xFF)

    def read_step_count(self):
        low = self._rreg(QMI8658_REG_StepCntLow)
        mid = self._rreg(QMI8658_REG_StepCntMid)
        high = self._rreg(QMI8658_REG_StepCntHigh)
        return (high << 16) | (mid << 8) | low

    def reset_step_count(self):
        self._wreg(QMI8658_REG_Ctrl9, CTRL9_CMD_RESET_PEDOMETER)
        time.sleep(0.01)

    # ---------- 6. 唤醒运动 (WoM) ----------
    def wom_interrupt(
        self,
        enable=True,
        int_pin=INT_PIN_1,
        threshold=0x10,
        handler=None,
        trigger=machine.Pin.IRQ_RISING,
    ):
        self._wreg(QMI8658_REG_Cal1_L, threshold & 0xFF)

        cal1_h = self._rreg(QMI8658_REG_Cal1_H)
        if int_pin == INT_PIN_1:
            cal1_h |= 1 << 6
            cal1_h &= ~(1 << 7)
        else:
            cal1_h &= ~(1 << 6)
            cal1_h &= ~(1 << 7)
        self._wreg(QMI8658_REG_Cal1_H, cal1_h)

        if enable:
            self._wreg(QMI8658_REG_Ctrl9, CTRL9_CMD_WRITE_WOM_SETTING)
            time.sleep(0.05)
        else:
            self._wreg(QMI8658_REG_Cal1_L, 0x00)

        self.attach_interrupt(int_pin, handler, trigger, STATUSINT_BIT_WOM)

    # ---------- 7. CmdDone ----------
    def cmd_done_interrupt(
        self,
        route_to_statusint=False,
        int_pin=INT_PIN_1,
        handler=None,
        trigger=machine.Pin.IRQ_FALLING,
    ):
        ctrl8 = self._rreg(QMI8658_REG_Ctrl8)
        if route_to_statusint:
            ctrl8 |= 1 << 7
        else:
            ctrl8 &= ~(1 << 7)
        self._wreg(QMI8658_REG_Ctrl8, ctrl8)
        self._ctrl8_val = ctrl8

        self.attach_interrupt(int_pin, handler, trigger, STATUSINT_BIT_CMD_DONE)

    # ---------- 状态解析 / 清除 ----------
    def parse_interrupt_status(self, status=None):
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
        return self._rreg(QMI8658_REG_StatusInt)
