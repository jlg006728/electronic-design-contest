# V91 中央提前纠偏

## 现象与根因

- YB-MYX05-V1.0已实测白底原始输出为高：物理灯亮/`g_gray_x*=1`表示白底，不表示压到红线。
- 例如白底基准`0xFF`、原始mask`0xC3`时，X1/X2/X7/X8亮、X3-X6灭；归一化红线mask为`0x3C`，实际是红线位于中央X3-X6。
- V90单P参数为`error/36`、限幅70。X3/X6只有41 ticks，而X2/X7为69、X1/X8为70，中央修正偏弱且外侧几乎没有递增余量。

## V91改动

- 保持基础PWM左380/右383、10ms主循环、灰度扫描、白底XOR标定、丢线搜索和停车保护不变。
- 单P调整为`error/26`、限幅110，不增加桥接、弯道状态机、MPU、编码器或任务逻辑。
- 单点响应：X4/X5为19 ticks，X3/X6为57，X2/X7为96，X1/X8为110；最大修正时内轮仍为270/273 ticks，不反转、不原地转向。
- 新增`g_gray_line_x1...g_gray_line_x8`，其1才表示相对白底检测到红线；`g_gray_x1...g_gray_x8`继续保留原始高电平含义。
- 固件版本为91。

## 验证

- TDD RED：新增测试首先因缺少`simple_line_profile.h`按预期编译失败；代码审查补测时又因缺少mask到误差的共享函数按预期失败。
- GREEN：Host控制测试9/9通过；覆盖X1-X8物理位权、空mask、正负最大PWM边界与白底XOR；整体行覆盖率97.83%，控制头文件行覆盖率93.62%。
- SysConfig 1.26.2、TI Arm Clang 4.0.4.LTS三单元编译和链接通过，0 diagnostics；未烧录。

## 实车顺序

1. 上电前八路全部放在白底，确认`g_gray_white_baseline_mask=255`。
2. 架空烧录，手动把红线从X4/X5移向X6、X7、X8，确认`g_line_correction`依次约19、57、96、110且方向正确。
3. 落地低速直线，每次至少测试3次；观察`g_gray_line_x*`和`g_gray_line_mask`，不要用物理灯亮灭判断红线位置。
4. 若仍来回摆动，记录`g_gray_line_mask`、`g_gray_line_error`、`g_line_correction`及左右PWM的连续变化，再决定是否加入简单D阻尼；本版不预先增加D项。
