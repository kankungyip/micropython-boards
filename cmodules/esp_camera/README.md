# ESP-CAMERA

Fork: [https://github.com/lemariva/micropython-camera-driver](https://github.com/lemariva/micropython-camera-driver)

基于 [ESP32-CAMERA](https://github.com/espressif/esp32-camera) 的 MicroPython 版本摄像头驱动，适用于 ESP32、ESP32S2、ESP32S3 芯片，支持多款摄像头。

## 使用说明

### 基础函数

#### `camera.init(*)`

初始化摄像头，必须初始化后才能使用摄像头拍照图片。

- **关键词参数**
  - `format`, 图片数据格式

    | 格式               | 说明              |
    | ------------------ | ----------------- |
    | `camera.JPEG`      | JPEG 格式         |
    | `camera.YUV422`    | YUV422            |
    | `camera.RGB565`    | RGB565 （默认值） |
    | `camera.GRAYSCALE` | 灰度              |

  - `framesize`, 分辨率（帧尺寸）

    | 分辨率（帧尺寸）       | 说明              |
    | ---------------------- | ----------------- |
    | `camera.FRAME_96X96`   | 96×96             |
    | `camera.FRAME_QQVGA`   | 160×120           |
    | `camera.FRAME_QCIF`    | 176×144           |
    | `camera.FRAME_HQVGA`   | 240×160           |
    | `camera.FRAME_240X240` | 240×240           |
    | `camera.FRAME_QVGA`    | 320×240           |
    | `camera.FRAME_CIF`     | 352×288           |
    | `camera.FRAME_HVGA`    | 480×320           |
    | `camera.FRAME_VGA`     | 640×480（默认值） |
    | `camera.FRAME_SVGA`    | 800×600           |
    | `camera.FRAME_XGA`     | 1024×768          |
    | `camera.FRAME_HD`      | 1280×720          |
    | `camera.FRAME_SXGA`    | 1280×1024         |
    | `camera.FRAME_UXGA`    | 1900×1200         |
    | `camera.FRAME_FHD`     | 1920×1080         |
    | `camera.FRAME_P_HD`    | 2560×1440         |
    | `camera.FRAME_P_3MP`   | 2048×1536         |
    | `camera.FRAME_QXGA`    | 2048×1536         |
    | `camera.FRAME_QHD`     | 2560×1440         |
    | `camera.FRAME_WQXGA`   | 2560×1600         |
    | `camera.FRAME_P_FHD`   | 2560×1600         |
    | `camera.FRAME_QSXGA`   | 2560×2048         |

  - `quality`，JPEG 质量，默认值：12，可选范围：0~63
  - `d0`，D0 引脚
  - `d1`，D1 引脚
  - `d2`，D2 引脚
  - `d3`，D3 引脚
  - `d4`，D4 引脚
  - `d5`，D5 引脚
  - `d6`，D6 引脚
  - `d7`，D7 引脚
  - `vsync`，VSYNC 引脚
  - `href`，HREF 引脚
  - `pclk`，PCLK 引脚
  - `pwdn`，POWER 引脚
  - `reset`，RESET 引脚
  - `xclk`，XCLK 引脚
  - `sda`，SDA 引脚
  - `scl`，SCL 引脚
  - `xclk_freq`，XCLK 信号频率，默认值：`camera.XCLK_20MHz`，可选值：`camera.XCLK_10MHz`
  - `fb_size`，帧缓存数量，默认值：1
  - `fb_location`，帧缓存区，默认值：`camera.FB_PSRAM`，可选值：`camera.FB_DRAM`

#### `camera.deinit()`

卸载摄像头，释放使用的硬件资源。

#### `camera.snapshot()`

获取 1 帧图片数据。

#### `camera.jpeg(quality)`

拍摄 1 帧 JPEG 图片数据，可选设置 JPEG 质量，可选范围：0~63（低质量）。

### 画面设置函数

#### `camera.quality(quality)`

重新设置 JPEG 质量，可选范围：0~63（低质量）。

#### `camera.flip(enable)`

设置画面上下颠倒。

#### `camera.mirror(enable)`

设置画面左右镜像。

#### `camera.contrast(contrast)`

设置画面对比度，默认值：0，可选范围：-2~2（高对比度）。

#### `camera.saturation(saturation)`

设置画面饱和度，默认值：0，可选范围：-2（灰度）~2。

#### `camera.brightness(brightness)`

设置画面亮度，默认值：0，可选范围：-2~2（高亮度）。

#### `camera.speffect(effect)`

设置画面特效。

| 特效                       | 说明           |
| -------------------------- | -------------- |
| `camera.EFFECT_NONE`       | 原图（默认值） |
| `camera.EFFECT_NEGATIVE`   | 负片           |
| `camera.EFFECT_GRAYSCALE`  | 灰度           |
| `camera.EFFECT_SEPIA`      | 棕褐色         |
| `camera.EFFECT_RED`        | 红色           |
| `camera.EFFECT_GREEN`      | 绿色           |
| `camera.EFFECT_BLUE`       | 蓝色           |
| `camera.EFFECT_ANTIQUE`    | 复古           |
| `camera.EFFECT_SKETCH`     | 素描           |
| `camera.EFFECT_SOLARIZE`   | 日光           |
| `camera.EFFECT_POSTERIZE`  | 海报           |
| `camera.EFFECT_WHITEBOARD` | 白板           |
| `camera.EFFECT_BLACKBOARD` | 黑板           |
| `camera.EFFECT_AQUA`       | 水晶           |

#### `camera.whitebalance(whitebalance)`

设置画面白平衡。

| 白平衡             | 说明           |
| ------------------ | -------------- |
| `camera.WB_NONE`   | 原始（默认值） |
| `camera.WB_SUNNY`  | 晴天           |
| `camera.WB_CLOUDY` | 阴天           |
| `camera.WB_OFFICE` | 办公室         |
| `camera.WB_HOME`   | 家中           |
