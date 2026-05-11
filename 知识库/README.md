# E题 简易自行瞄准装置 — 知识库索引

## 赛题概要

> TI杯 2025年全国大学生电子设计竞赛 — E题 (本科组)

设计制作一个自动寻迹小车 + 二维云台瞄准装置，使用 MSPM0 MCU 控制，蓝紫激光笔瞄准靶面。

## 器件清单速查

| 类别 | 推荐器件 | 数量 | 知识库文档 |
|------|----------|------|-----------|
| 主控MCU | MSPM0G3507 | 1 | [数据手册](MCU_MSPM0/MSPM0G3507_数据手册摘要.md) / [SDK API](MCU_MSPM0/MSPM0_SDK_DriverLib_API速查.md) |
| MCU开发 | MSPM0 开发指南 PDF | — | [PDF目录索引](MCU_MSPM0/MSPM0开发指南_目录索引.md) |
| 开发板 | LP-MSPM0G3507 | 1 | [EVM用户指南索引](MCU_MSPM0/EVM用户指南_目录索引.md) |
| 巡迹传感器 | TCRT5000 | 7-8 | [数据手册](传感器_巡迹/TCRT5000_数据手册摘要.md) |
| 巡迹算法 | PID + 传感器阵列 | — | [PID控制理论](传感器_巡迹/PID控制理论与巡迹算法.md) |
| IMU | MPU6050 (可选) | 1 | [数据手册](传感器_巡迹/MPU6050_数据手册摘要.md) |
| 电机驱动 | TB6612FNG | 1 | [数据手册](电机驱动/TB6612FNG_数据手册摘要.md) |
| 减速电机 | TT直流 + 编码器 | 2 | [规格书](电机驱动/TT直流减速电机_编码器规格.md) |
| 运动学 | 差速转向模型 | — | [运动学模型](电机驱动/差速转向运动学模型.md) |
| 云台舵机 | MG90S | 2 | [规格摘要](舵机云台/MG90S_规格摘要.md) |
| 视觉模块 | OpenMV H7 | 1 | [选型对比](视觉模块/OpenMV_K230_选型对比.md) / [API文档](视觉模块/OpenMV颜色追踪API.md) |
| 通信协议 | UART 串口 | — | [通信协议](视觉模块/OpenMV_MSPM0_串口通信协议.md) |
| 激光笔 | 405nm <=10mW | 1 | [规格要求](激光模块/405nm激光模块_规格要求.md) |
| 靶纸 | A4 UV紫外感光纸 | 若干 | [见激光模块文档](激光模块/405nm激光模块_规格要求.md) |
| 电源 | 双路独立供电 | — | [供电方案](电源方案/供电方案设计.md) |
| 报告 | 设计总结报告 | 1 | [报告模板](电赛设计报告模板.md) |

## MCU 引脚资源估算 (MSPM0G3507)

| 功能 | GPIO 数量 | 说明 |
|------|-----------|------|
| 巡迹传感器 | 7-8 | 数字输入或 ADC 输入 |
| 电机方向控制 | 4 | AIN1/AIN2/BIN1/BIN2 |
| 电机 PWM | 2 | PWMA/PWMB |
| 电机编码器 | 4 | 可选, 用于速度闭环 |
| STBY 控制 | 1 | TB6612 使能 |
| 舵机 PWM | 2 | 云台二轴控制 |
| 视觉模块 UART | 2 | TX/RX |
| MPU6050 I2C | 2 | SDA/SCL (可选) |
| LED 指示 | 2 | 状态指示 |
| 按键/编码器 | 2-3 | 圈数设置等 |
| **合计** | **~28** | MSPM0G3507 可用~40+ GPIO |

## 关键技术挑战速查

| 挑战 | 参考文档 |
|------|----------|
| 靶心识别 (0.1cm @ 50cm) | [OpenMV颜色追踪API](视觉模块/OpenMV颜色追踪API.md) |
| 巡迹控制 (黑线1.8cm) | [PID控制理论](传感器_巡迹/PID控制理论与巡迹算法.md) |
| 光斑精度 (D1<=2cm) | [通信协议](视觉模块/OpenMV_MSPM0_串口通信协议.md) + [PID](传感器_巡迹/PID控制理论与巡迹算法.md) |
| 同步画圆 (D2<=2cm) | [运动学模型](电机驱动/差速转向运动学模型.md) |
| 小车速度 (20s/圈, >=20cm/s) | [电机规格](电机驱动/TT直流减速电机_编码器规格.md) |
| 航向保持 | [MPU6050](传感器_巡迹/MPU6050_数据手册摘要.md) |

## 关键链接

| 资源 | 链接 |
|------|------|
| TI MSPM0G3507 产品页 | https://www.ti.com/product/MSPM0G3507 |
| MSPM0 SDK 下载 | https://www.ti.com/tool/MSPM0-SDK |
| MSPM0 DriverLib API | {SDK}/docs/english/driverlib/ |
| Vishay TCRT5000 | https://www.vishay.com/docs/83760/tcrt5000.pdf |
| Pololu TB6612FNG | https://www.pololu.com/product/0713/resources |
| OpenMV 官方文档 | https://docs.openmv.io |
| OpenMV 颜色追踪 | https://book.openmv.cc/image/blob.html |
| 电赛培训网 | https://www.nuedc-training.com.cn |
| E题开源参考 | https://oshwhub.com/yun_7783/ |

## 下一步

- [ ] 选购器件, 搭建硬件测试平台
- [ ] MSPM0 开发环境搭建 (CCS IDE + MSP SDK)
- [ ] 巡迹算法开发与调试 (传感器阵列 + PID)
- [ ] 云台 + OpenMV 瞄准系统开发 (颜色追踪 + 串口通信)
- [ ] 系统集成与联调 (小车 + 云台 + 激光)
