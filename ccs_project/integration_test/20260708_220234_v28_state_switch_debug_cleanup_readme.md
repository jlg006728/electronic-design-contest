# 20260708_220234 V28 状态切换调试版完善

## 基底

- 基于 V27：`20260708_215716_v27_state_switch_hold_2s_debug.c`。
- 当前 CCS 活动文件：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`。
- 本版只编译验证，未自动烧录。

## 本版完善内容

V28 不改变核心控制策略：

```text
A 点无线桥接 -> 任意红外看到黑线 -> 立即进入普通循迹
```

本版只做调试可读性和代码清理：

1. 删除 V25 遗留的 `PHASE_START_GAP_CAPTURE` 捕线状态及相关函数。
2. 删除未使用的捕线参数和变量，避免后续调试误读。
3. 状态切换强制停车 2 秒期间，PA15 LED 闪烁。
4. 新增启动瞬间红外快照 Watch 变量。

## 状态提示

```text
上电 1 声：程序启动
停 2 秒 + 2 声：进入 A 点无线桥接
停 2 秒 + 4 声：上电时程序直接认为已经有线
停 2 秒 + 3 声：桥接过程中任意红外看到黑线，切入普通循迹
长响：桥接超时仍未看到黑线
```

## 新增/保留 Watch 变量

启动瞬间红外快照：

```c
g_startup_line_valid_snapshot
g_startup_line_raw_mask_snapshot
g_startup_line_active_count_snapshot
g_startup_line_error_snapshot
```

状态切换停车：

```c
g_state_switch_reason
g_state_switch_hold_frames_left
g_state_switch_hold_event_count
g_state_switch_hold_frames_total
```

含义：

```text
g_state_switch_reason = 1：进入桥接
g_state_switch_reason = 2：桥接重捕线进入循迹
g_state_switch_reason = 3：上电直接进入循迹
```

## 首测判断

如果白底启动后是：

```text
停 2 秒 + 2 声
```

说明 A 点无线桥接入口正确。

如果白底启动后是：

```text
停 2 秒 + 4 声
```

说明启动瞬间至少一路红外被代码读成黑线。此时重点看：

```c
g_startup_line_raw_mask_snapshot
```

哪一位是 1，就查对应传感器 DO、接线、阈值和地面反光。

如果弯道入口出现：

```text
停 2 秒 + 3 声
```

说明重捕切换成功；如果之后仍冲出弯道，下一步调普通循迹速度/急弯参数。
