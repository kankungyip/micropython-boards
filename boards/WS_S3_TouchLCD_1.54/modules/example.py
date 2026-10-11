import gc
import math
import time

import board
import lvgl as lv

# ============================================================
# 配置
# ============================================================
FRAME_WIDTH = 240
FRAME_HEIGHT = 240
CUBE_UPDATE_MS = 40

# 立方体参数
CUBE_HALF = 52.0
CUBE_CX = FRAME_WIDTH / 2
CUBE_CY = FRAME_HEIGHT / 2
CAMERA_DIST = 300.0

# 陀螺仪死区（dps）
DEADZONE_DPS = 0.6

# 加速度计修正系数（互补滤波）
ALPHA_ACC = 0.03

# ------------------------------------------------------------
# 芯片贴装方向（背贴，从芯片正面看顺时针 90°）
#
# 开发板坐标系（面向屏幕）: bx=屏幕右, by=屏幕上, bz=屏幕朝外
#
# 实测映射:
#     bx = +芯片 y
#     by = +芯片 x
#     bz = -芯片 z
# ------------------------------------------------------------

# ============================================================
# 1. 板级初始化
# ============================================================
board.init()

lcd = board.display
if lcd:
    lcd.set_backlight(100)

scr = lv.screen_active()


# ============================================================
# 2. 传感器 → pitch / roll
# ============================================================
def _deadzone(v):
    return 0.0 if -DEADZONE_DPS < v < DEADZONE_DPS else v


def _acc_pitch_roll():
    """
    从加速度计计算 (pitch, roll)，单位弧度。
        pitch: 板子向后仰为正（绕 bx）
        roll:  板子向右倾为正（绕 by）
    失败返回 None。
    """
    if board.imu is None:
        return None
    try:
        acc, _, _ = board.imu.read()
        ax_c, ay_c, az_c = acc
        # 芯片 → 开发板
        ax = +ax_c
        ay = -ay_c
        az = -az_c
        # 开发板水平朝上时 (ax, ay, az) ≈ (0, 0, +1)
        pitch = math.atan2(ay, az)
        roll = math.atan2(-ax, az)
        return pitch, roll
    except Exception as e:
        print("加速度计读取失败:", e)
        return None


def _gyro_pitch_roll():
    """
    从陀螺仪读取角速度，映射到开发板坐标系。
    返回 (pitch_rate, roll_rate)，单位 dps。
    失败返回 (0.0, 0.0)。
    """
    if board.imu is None:
        return 0.0, 0.0
    try:
        _, gyr, _ = board.imu.read()
        gx_c, gy_c, _ = gyr
        # 芯片 → 开发板
        gx = +gx_c  # 绕 bx 的角速度
        gy = -gy_c  # 绕 by 的角速度
        return gx, gy
    except Exception as e:
        print("陀螺仪读取失败:", e)
        return 0.0, 0.0


# ============================================================
# 3. 线框立方体
# ============================================================
CUBE_VERTS = (
    (-1, -1, -1),
    (1, -1, -1),
    (1, 1, -1),
    (-1, 1, -1),
    (-1, -1, 1),
    (1, -1, 1),
    (1, 1, 1),
    (-1, 1, 1),
)
CUBE_EDGES = (
    (0, 1),
    (1, 2),
    (2, 3),
    (3, 0),
    (4, 5),
    (5, 6),
    (6, 7),
    (7, 4),
    (0, 4),
    (1, 5),
    (2, 6),
    (3, 7),
)

cube_lines = []
for _ in range(len(CUBE_EDGES)):
    line = lv.line(scr)
    line.set_pos(0, 0)
    line.set_size(FRAME_WIDTH, FRAME_HEIGHT)
    line.set_style_line_width(2, 0)
    line.set_style_line_color(lv.color_hex(0x00E0FF), 0)
    try:
        line.set_style_line_rounded(True, 0)
    except Exception:
        pass
    line.set_points([{"x": 0, "y": 0}, {"x": 0, "y": 0}], 2)
    cube_lines.append(line)

angle_pitch = 0.0
angle_roll = 0.0
last_cube_ms = 0


def _rotate(x, y, z, ax, ay):
    """按 X→Y 顺序旋转（不支持 yaw）。"""
    cx, sx = math.cos(ax), math.sin(ax)
    y, z = y * cx - z * sx, y * sx + z * cx
    cy, sy = math.cos(ay), math.sin(ay)
    x, z = x * cy + z * sy, -x * sy + z * cy
    return x, y, z


def _project(x, y, z):
    """立方体坐标(+y 朝上) → 屏幕像素(+y 朝下)。"""
    k = CAMERA_DIST / (CAMERA_DIST + z)
    return CUBE_CX + x * k, CUBE_CY - y * k


def draw_cube():
    pts = []
    for vx, vy, vz in CUBE_VERTS:
        rx, ry, rz = _rotate(
            vx * CUBE_HALF,
            vy * CUBE_HALF,
            vz * CUBE_HALF,
            angle_pitch,
            angle_roll,
        )
        px, py = _project(rx, ry, rz)
        pts.append((px, py))

    for i, (a, b) in enumerate(CUBE_EDGES):
        cube_lines[i].set_points(
            [
                {"x": int(pts[a][0]), "y": int(pts[a][1])},
                {"x": int(pts[b][0]), "y": int(pts[b][1])},
            ],
            2,
        )


def update_cube(_t):
    """陀螺仪积分 + 加速度计修正 → 重绘。"""
    global angle_pitch, angle_roll, last_cube_ms

    now = time.ticks_ms()
    dt = time.ticks_diff(now, last_cube_ms) / 1000.0
    last_cube_ms = now
    if dt <= 0 or dt > 0.5:
        dt = CUBE_UPDATE_MS / 1000.0

    # ---- 1. 陀螺仪积分 ----
    gx, gy = _gyro_pitch_roll()
    angle_pitch += math.radians(_deadzone(gx) * dt)
    angle_roll += math.radians(_deadzone(gy) * dt)

    # ---- 2. 加速度计缓慢回正 ----
    ar = _acc_pitch_roll()
    if ar is not None:
        ap, aro = ar
        angle_pitch = (1.0 - ALPHA_ACC) * angle_pitch + ALPHA_ACC * ap
        angle_roll = (1.0 - ALPHA_ACC) * angle_roll + ALPHA_ACC * aro

    draw_cube()


# ============================================================
# 4. 主循环
# ============================================================
gc.collect()

last_cube_ms = time.ticks_ms()
draw_cube()
lv.timer_create(update_cube, CUBE_UPDATE_MS, None)
print("就绪：陀螺仪立方体已启动")

while True:
    lv.timer_handler()
    time.sleep_ms(5)
