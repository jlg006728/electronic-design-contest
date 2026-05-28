# CCS 操作指南 — 从零开始烧录电机测试程序

> 适用于 CCS Theia 20.5.0 + MSPM0 SDK 2.10.00.04

---

## 一、打开 CCS

1. 双击桌面 `CCS 20.5.0` 快捷方式，或从开始菜单找到 Code Composer Studio 20.5
2. 选择 Workspace 路径: `C:\Users\1\Desktop\电赛\ccs_project` (可以勾选 "Use this as the default")
3. 点击 Launch

---

## 二、新建工程 (只需做一次)

### 2.1 创建项目

1. 菜单: **File → New → CCS Project**
2. Target 选择: **MSPM0G3507** (在下拉框搜索)
3. Connection 选择: **TI XDS110 USB Debug Probe** (LaunchPad 自带)
4. Project name: `motor_test`
5. Project template: **Empty Project (with main.c)** → 点击 Finish

### 2.2 替换 main.c

1. 在左侧 Project Explorer 中找到 `motor_test → main.c`
2. 右键 → Delete (删除默认的 main.c)
3. 把 `C:\Users\1\Desktop\电赛\ccs_project\motor_test\main.c` 复制到工程目录:
   - 方法1: 直接拖拽文件到 Project Explorer 中的 motor_test 文件夹
   - 方法2: 右键 motor_test → Import → File System → 选择 main.c

### 2.3 配置 SysConfig (简化版)

由于测试程序手动初始化 PWM，SysConfig 只需开启 GPIO 时钟:

1. 右键工程 → **New → SysConfig File**
2. 文件名保持默认，点 Finish
3. 在 SysConfig 图形界面中:
   - 左侧搜索 **GPIO**，拖入配置区
   - 勾选 **GPIOA** 和 **GPIOB** 的时钟使能
4. 保存 (Ctrl+S)

> 如果不想用 SysConfig，也可以在 main.c 开头手动加:
> `DL_GPIO_enablePower(GPIOA); DL_GPIO_enablePower(GPIOB);`

---

## 三、添加 SDK 路径 (如果工程找不到 DriverLib)

1. 右键工程 → **Properties**
2. **Build → Arm Compiler → Include Options**
3. 添加路径: `C:/ti/mspm0_sdk_2_10_00_04/source`
4. **Build → Arm Linker → File Search Path**
5. 添加库: `C:/ti/mspm0_sdk_2_10_00_04/source/ti/driverlib/lib/mspm0g3507/driverlib.lib`
6. 点击 Apply and Close

> 如果是用 CCS 的 MSPM0 工程模板创建的项目，SDK 路径通常会自动配置好。

---

## 四、编译

1. 点击工具栏的锤子图标 (Build)，或按 **Ctrl+B**
2. 底部 Console 窗口应显示:
   ```
   **** Build of configuration Debug for project motor_test ****
   ...
   Build Finished. 0 Errors, 0 Warnings.
   ```
3. 如果有错误:
   - **找不到头文件** → 检查 SDK 路径 (第三节)
   - **找不到 driverlib.lib** → 检查链接库路径
   - **语法错误** → 检查 main.c 是否完整复制

---

## 五、连接 LaunchPad 并烧录

### 5.1 硬件连接

1. 用 USB 线连接 LaunchPad 到电脑
2. 电脑应识别到 "XDS110 Debug Probe" (首次可能需要安装驱动)
3. 设备管理器中应出现: `Ports (COM & LPT) → XDS110 Class Application/User UART`

### 5.2 烧录 (Debug 模式)

1. 点击工具栏的虫子图标 (Debug)，或按 **F11**
2. CCS 会自动:
   - 编译 (如果没编译过)
   - 连接 LaunchPad
   - 下载程序到芯片
   - 停在 main() 入口
3. 点击工具栏的绿色三角 (Resume)，或按 **F8** → 程序开始运行
4. 观察电机动作:
   - LED 闪 3 次 → 开始测试
   - 左电机正转 2s → 停 0.5s → 反转 2s → 停
   - 右电机正转 2s → 停 0.5s → 反转 2s → 停
   - 双电机 30% → 50% → 70% → 停
   - LED 快闪 → 测试完成

### 5.3 重新烧录

如果修改了代码:
1. 先点红色方块 (Terminate) 停止调试
2. 修改代码 → Ctrl+B 编译
3. 再点虫子图标 (Debug) 重新烧录

### 5.4 不进入 Debug 直接运行

如果只想下载程序让它自动运行:
1. 菜单: **Run → Debug As → LaunchPad (XDS110)**
2. 或者: 右键工程 → **Debug As → Code Composer Studio Session**

---

## 六、常见问题

### Q: Debug 报错 "Could not connect to device"
- 检查 USB 线是否连接
- 检查 LaunchPad 电源开关是否打开
- 拔插 USB 重试
- 检查设备管理器是否有 XDS110 设备

### Q: 编译报错 "unresolved symbol DL_GPIO_setPins"
- SDK 路径没配好，检查第三节
- 确认 driverlib.lib 已添加到链接器

### Q: 电机不转
- 检查 STBY (PB4) 是否接了 HIGH (用万用表测 LaunchPad PB4 引脚应为 3.3V)
- 检查 TB6612 VIN+ 是否接了 7.4V
- 检查电机线是否接好
- 用万用表测 PWMA (PA12) 是否有 PWM 输出 (应有 ~1.6V 平均电压 @50% 占空比)

### Q: 电机转向反了
- 交换同一通道的两根电机线 (AO1↔AO2 或 BO1↔BO2)

### Q: LED 不闪
- 检查 PA15 引脚是否正确连接 LED
- 确认 LED 方向正确 (长脚接 PA15，短脚接 GND)

---

## 七、测试预期结果

| 阶段 | LED | 左电机 | 右电机 | 持续时间 |
|------|-----|--------|--------|----------|
| 启动 | 闪3次 | 停 | 停 | ~1s |
| 测试1 | 亮 | **正转** | 停 | 2s |
| 间隔 | 灭 | 停 | 停 | 0.5s |
| 测试2 | 亮 | **反转** | 停 | 2s |
| 间隔 | 灭 | 停 | 停 | 0.5s |
| 测试3 | 亮 | 停 | **正转** | 2s |
| 间隔 | 灭 | 停 | 停 | 0.5s |
| 测试4 | 亮 | 停 | **反转** | 2s |
| 间隔 | 灭 | 停 | 停 | 0.5s |
| 测试5 | 亮 | 30% | 30% | 1.5s |
| 测试5 | 亮 | 50% | 50% | 1.5s |
| 测试5 | 亮 | 70% | 70% | 1.5s |
| 完成 | 快闪 | 停 | 停 | 持续 |

> 如果某个电机不转但其他正常 → 检查该电机的接线
> 如果两个电机都不转 → 检查 STBY 和 VIN+ 电源
> 如果 PWM 引脚无输出 → 检查 PA12/PA13 接线和 TIMG0 配置
