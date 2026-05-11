# OpenMV 颜色追踪 API 快速参考

> 官方文档: https://book.openmv.cc/image/blob.html

## 初始化

```python
import sensor, image, time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)  # 160x120
sensor.skip_frames(10)

# 关键: 关闭自动调节, 确保颜色识别稳定
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
```

## find_blobs() API

```python
blobs = image.find_blobs(
    thresholds,          # 颜色阈值列表 [(L,A,B) ...]
    roi=Auto,            # 感兴趣区域 (x,y,w,h)
    x_stride=2,          # X方向跳步 (值越大越快但越不精确)
    y_stride=1,
    invert=False,        # 反转阈值
    area_threshold=10,   # 最小外框面积
    pixels_threshold=10, # 最小像素数
    merge=False,         # 是否合并邻近色块
    margin=0,            # 合并边距
)
```

## LAB 颜色阈值设定

```python
# 使用 IDE 工具: 工具 → Machine Vision → Threshold Editor
# 拖动滑块实时预览

# E题靶心红色阈值 (需根据实际靶面调整)
red_threshold = (30, 100, 15, 127, 15, 127)
# 格式: (minL, maxL, minA, maxA, minB, maxB)

# 多颜色同时识别
thresholds = [red_threshold, green_threshold, blue_threshold]
```

## Blob 对象属性

| 属性 | 含义 | 用途 |
|------|------|------|
| blob.cx() | 色块中心 X | 计算水平偏差 |
| blob.cy() | 色块中心 Y | 计算俯仰偏差 |
| blob.w() | 宽度 | 估算距离/大小 |
| blob.h() | 高度 | 估算距离/大小 |
| blob.pixels() | 像素数 | 滤波: 过滤小面积噪点 |
| blob.area() | 外框面积 w×h | 筛选最大目标 |
| blob.rect() | 矩形 (x,y,w,h) | 画出识别框 |
| blob.rotation() | 旋转角 (弧度) | 识别方向 |
| blob.code() | 16bit颜色码 | 区分不同颜色目标 |
| blob.density() | 填充密度 | 判断是否为实心圆 |

## E题靶心识别示例

```python
import sensor, image, time
from pyb import UART

# 靶心红色阈值
target_threshold = (25, 85, 20, 100, 10, 100)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.VGA)  # 640x480, 需要更高分辨率
sensor.skip_frames(20)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

uart = UART(3, 115200)

def find_target(img):
    blobs = img.find_blobs([target_threshold], pixels_threshold=5,
                           area_threshold=10, merge=True)
    if not blobs:
        return None
    # 取最大的色块 (假设靶心是最明显的红色)
    best = max(blobs, key=lambda b: b.pixels())
    return best

while True:
    img = sensor.snapshot()
    target = find_target(img)
    if target:
        img.draw_cross(target.cx(), target.cy(), color=(0,255,0), size=10)
        img.draw_rectangle(target.rect())

        # 发送坐标 (cx, cy) 到 MCU
        data = bytearray([0x2C, 4, target.cx() & 0xFF,
                          target.cy() & 0xFF,
                          target.cx() >> 8,
                          target.cy() >> 8, 0x5B])
        uart.write(data)
```

## 性能优化

| 方法 | 效果 |
|------|------|
| 降低分辨率 (QQVGA) | 帧率提升 3-4x |
| 使用 ROI 限定搜索区域 | 减少无效扫描 |
| 增大 x_stride/y_stride | 牺牲精度换速度 |
| 关闭 merge | 减少合并计算 |
| 设置合理的 area_threshold | 过滤噪点 |
