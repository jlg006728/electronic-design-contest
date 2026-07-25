# V93 软件稳定快照

- 固件：`g_firmware_version=93`
- 固定路线：`药房→1→2→3→4→7→5→6→8→药房`
- 病房动作：每处静止2000 ms，随后原地180°掉头
- 路口动作：10次90°陀螺仪闭环转向
- OpenMV：未启用
- 工具链：CCS 20.5、TI Arm Clang 4.0.4.LTS、MSPM0 SDK 2.10.00.04、SysConfig 1.26.2

## 已验证

- 完整路线Host测试：`18/18 passed`
- 路线模块行覆盖率：`97.67%`
- 极简循迹Host测试：`9/9 passed`
- 循迹模块行覆盖率：`97.83%`
- TI clean build：SysConfig、编译、链接全部通过，0 diagnostics
- map中存在`GROUP1_IRQHandler`、`SysTick_Handler`、编码器计数、路线故障、控制超时与停止原因符号
- 最终独立复审：0 CRITICAL、0 HIGH、0 MEDIUM

最终产物：

- `mock_problem.out` SHA256：`429E3419AB318EE55BB8FCF1DD577F0365A1DEF97F3BE6F24D03863E5ABE0CC5`
- `mock_problem.map` SHA256：`207843DAA34BB6E84BF659C9A005ED557939EF919B0223CCDE5E6A812C709B13`

## 尚未验证

- 未自动烧录，未执行目标板HIL或完整实车路线。
- `ENCODER_RISING_EDGES_PER_CM=736`仍是理论初值。
- 九个端点窗口仍为首轮采样宽窗口：病房20～70 cm，药房20～100 cm。
- 必须先完成50 cm编码器标定、九段各3次端点里程采样、MPU/单路口/单病房/半程测试，再进行全程验收。

软件通过不能替代实车完成。只有一次性访问全部八个病房并返回药房，且全部Watch验收值正确，才能宣布总目标完成。
