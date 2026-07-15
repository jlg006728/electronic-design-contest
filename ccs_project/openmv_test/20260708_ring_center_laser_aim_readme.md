# 20260708 靶心识别 + 激光瞄准说明

## 文件

- OpenMV 脚本：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260708_openmv_uart_stream_ring_center_v1.py`
- MSPM0 参数版：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260708_120000_v23_ring_center_laser_aim.c`

## 激光笔固定位置

激光笔应固定在二维云台的俯仰输出端，和 OpenMV 镜头同向、近距离并排，优先让激光轴尽量靠近镜头光轴。

不要把激光笔固定在底盘上，也不要只固定在水平舵机底座上。底盘固定只能指向车头方向，无法跟随靶心；只跟水平舵机则没有上下修正，打不到墙面靶心。

推荐结构：

- 下层舵机 PA14：水平/yaw。
- 上层舵机 PA17：俯仰/pitch。
- OpenMV 和激光笔都固定在上层俯仰架上。
- 激光笔尽量贴近 OpenMV 镜头一侧，两个轴线平行。
- 上电前手调云台 HOME，使 OpenMV 画面中心大致对准靶纸，激光点也尽量落在靶心附近。

## OpenMV 优化

旧脚本追踪的是红色 blob 中心，面对同心圆靶纸时容易追到某段圆环或顶部红胶带。

新脚本保持原 17 字节 UART 协议不变，但把 `cx/cy` 改为同心圆目标中心：

- 优先识别视线范围内最小的有效红色圆环，也就是靠近靶心的内圈。
- 找到内圈后直接用该圆环外接框中心作为靶心坐标。
- 内圈找不到时，再使用霍夫圆检测同心圆结构。
- 默认关闭红色 blob 回退，避免红色光影/胶带被误认为靶。
- 如果启用红色回退，会合并圆环区域并过滤小而密的红色块。
- 可选 blob 后的霍夫圆细化，默认关闭。

现场调试顺序：

1. 只运行 OpenMV 脚本，看内圈附近是否出现黄色圆和绿色十字。
2. 黄色圆应优先套住最小红色圆环，绿色十字应落在靶心小黑点附近。
3. 如果没有黄色圆，先调 `RED_THRESHOLDS` 的 `a_min`：例如把第二组 `(0, 100, 2, 80, -18, 95)` 的 `2` 改成 `0`。
4. 如果黄色圆套到了顶部红胶带或红色噪点，提高 `SMALL_RING_MIN_W/H`，或降低 `SMALL_RING_MAX_DENSITY_PERCENT`。
5. 如果内圈始终断裂，再调 `CIRCLE_PRIMARY_THRESHOLD`：识别不到就降低，误检多就升高。
6. 只有圆检测完全不稳定时，才临时打开 `ENABLE_RED_BLOB_FALLBACK = True`。

## MSPM0 优化

V23 只改瞄准参数，不改 UART 协议、循迹、B/C/D/A 状态机。

关键变化：

```c
#define LASER_CALIB_X_PIXELS        0
#define LASER_CALIB_Y_PIXELS        0
#define AIM_DEAD_X_PIXELS           8
#define AIM_DEAD_Y_PIXELS           6
#define AIM_X_DIVISOR               3
#define AIM_Y_DIVISOR               4
#define AIM_MAX_STEP_US             45
#define AIM_MIN_PIXELS              25U
#define AIM_FILTER_DIVISOR          3
#define AIM_TARGET_CONFIRM_FRAMES   3U
```

含义：

- 死区更小：更适合激光瞄准靶心。
- 步进更慢：减少激光点在靶心附近抖动。
- 目标确认帧更多：减少误识别红胶带/噪点后乱动。
- 滤波更强：圆环识别轻微跳动时云台更稳。
- 激光偏置可调：如果 OpenMV 十字已经对准靶心，但激光点固定偏离靶心，优先机械微调；还差一点时再改 `LASER_CALIB_X_PIXELS/Y_PIXELS`。

## 现场标定

1. 固定好激光笔后，先不要让车跑，只做静态瞄准。
2. 手动让靶心在 OpenMV 画面中心附近，观察激光点是否也在靶心附近。
3. 若画面中心对准靶心但激光固定偏左/右/上/下，先机械微调激光夹具，不要先改视觉代码。
4. 若偏差随距离明显变化，说明激光轴和镜头轴不平行，需要重新调整夹具角度。
5. 若方向反了，再改 `AIM_X_PULSE_SIGN` 或 `AIM_Y_PULSE_SIGN`。
