# V94 软件验证记录

验证日期：2026-07-23

## 标定输入

三次落地直推距离均为50.0 cm。生产代码使用有方向净计数`g_encoder_left_count/g_encoder_right_count`，总边沿`g_encoder_*_edges`仅用于诊断。

| 次数 | 左净计数 | 右净计数 | edges/cm |
|---:|---:|---:|---:|
| 1 | -33152 | 33334 | 664.86 |
| 2 | -33428 | 33334 | 667.62 |
| 3 | -33003 | 33093 | 660.96 |

平均值为`664.48 edges/cm`，固件最近整数取`ENCODER_RISING_EDGES_PER_CM=664U`。病房20～70 cm窗口对应`13280～46480 edges`，药房20～100 cm窗口对应`13280～66400 edges`。

## Host验证

- 完整路线：`medicine_route_host_test: 19/19 passed`
- 极简循迹：`simple_line_control_test: 9/9 passed`
- 路线模块总行覆盖率：`97.71%`
- 完整路线仿真直接使用生产标定`664 edges/cm`以及生产阈值：方向容差`1328`、轮差检查`3320`、端点确认缓冲`3320 edges`
- 路线表锁定9个端点动作索引、病房顺序和20/70/100 cm边界

## TI构建

- CCS：`20.5`
- TI Arm Clang：`4.0.4.LTS`
- MSPM0 SDK：`2.10.00.04`
- clean build：0 diagnostics
- linkInfo banner：`TI ARM Clang Linker PC v4.0.4.LTS`
- map已确认`GROUP1_IRQHandler`、`SysTick_Handler`和`g_firmware_version`

关键SHA256：

```text
C6A94F1B7C17610C789E2259BBE8C6428E6A5E45A401A4F20947EAE96540CE90  project/empty.c
8EA510DD71E0E96990F84E2625089FFB051D318F4FE909CB36BF2EABDFA2EBA3  project/encoder_calibration_profile.h
A335C6B1B7709B3133960B1252E08BD094828FC1A18A7065478523EC2315A3B4  project/Debug/mock_problem.out
1396A4DAF8EB592687AABC11913FF2677156E505A90C668939AA5A168E443DE5  project/Debug/mock_problem.map
B7552EF48216336DAA03E6C2A7DD1316A4B5B63F7A2229C608CDEF165BE90993  tests/medicine_route_host_test.c
```

## 边界

软件、构建和50 cm编码器比例已通过；MPU静止/方向、单路口90°、单病房2秒与180°、九个端点窗口、半程和完整路线仍需实车验收。未烧录，未操作CCS图形界面，不得据此宣布全车路线完成。

继续构建必须使用`C:\ti\ccs2050\ccs\theia\ccstudio.exe`。关闭CCS 20.2；它会把Debug生成文件重新写回TI Arm Clang 4.0.3。
