# 20260707_153950 不使用编码器的 5/7 距离缩放版 V9

## 文件

- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_153950_course_point_stop_buzzer_scaled_5over7_v9.c`
- 基于 V8：关键点停车 + OpenMV + 双舵机 + 5V 有源蜂鸣器。

## 目的

用户实测 A->B 距离比地图实际距离多约 `2/5`，因此这一版不接入编码器，先把所有按距离触发的点位阈值乘 `5/7`，用于快速现场验证。

## 关键参数

```c
#define COURSE_LAP_SAMPLES          914U
#define COURSE_BC_MID_SAMPLE        330U

#define COURSE_POINT_B_SAMPLE       203U
#define COURSE_POINT_C_SAMPLE       457U
#define COURSE_POINT_D_SAMPLE       660U
#define COURSE_POINT_A_SAMPLE       COURSE_LAP_SAMPLES
```

来源：

```text
1280 * 5 / 7 = 914
284  * 5 / 7 = 203
462  * 5 / 7 = 330
640  * 5 / 7 = 457
924  * 5 / 7 = 660
```

## 注意

这仍然不是精确距离控制，只是把当前时间估距误差按现场现象修正。电池电压、负重、速度变化后仍会漂。

## 验证

- CCS `gmake all` 编译通过。
- 未自动烧录。
