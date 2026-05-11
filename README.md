# 电子设计竞赛知识库 — 2025年E题复刻

> **TI杯 2025年全国大学生电子设计竞赛 · E题 · 本科组**
>
> 简易自行瞄准装置 — 自动寻迹小车 + 二维云台瞄准系统

[![MCU](https://img.shields.io/badge/MCU-MSPM0G3507-blue)](https://www.ti.com/product/MSPM0G3507)
[![Language](https://img.shields.io/badge/lang-C%20%2F%20MicroPython-orange)]()
[![Status](https://img.shields.io/badge/status-knowledge%20base%20ready-brightgreen)]()

---

## 项目简介

本项目为备考 **2026年全国大学生电子设计竞赛** 而建立，首先复刻 2025 年 E 题"简易自行瞄准装置"。仓库包含完整的器件知识库、理论推导和参考资料，覆盖从硬件选型到算法设计的全流程。

### 赛题目标

设计制作一辆自动巡迹小车，搭载二维云台和 405nm 蓝紫激光笔，实现：
- 沿黑色轨迹线（1.8cm 宽）自动循迹行驶
- 视觉识别靶面红/绿色靶心（≤0.1cm @ 50cm）
- 激光光斑自动瞄准并跟随靶心画圆
- 双路独立供电，独立开关带 LED 指示

---

## 仓库结构

```
electronic-design-contest/
├── README.md                          # ← 本文件
├── 知识库/                             # 核心知识库
│   ├── README.md                      #   知识库索引 & 速查表
│   ├── MCU_MSPM0/                     #   主控芯片
│   │   ├── MSPM0G3507_数据手册摘要.md
│   │   ├── MSPM0_SDK_DriverLib_API速查.md
│   │   ├── MSPM0开发指南_目录索引.md
│   │   └── EVM用户指南_目录索引.md
│   ├── 传感器_巡迹/                    #   巡迹系统
│   │   ├── TCRT5000_数据手册摘要.md
│   │   ├── PID控制理论与巡迹算法.md
│   │   └── MPU6050_数据手册摘要.md
│   ├── 电机驱动/                       #   驱动 & 运动学
│   │   ├── TB6612FNG_数据手册摘要.md
│   │   ├── TT直流减速电机_编码器规格.md
│   │   └── 差速转向运动学模型.md
│   ├── 舵机云台/                       #   云台伺服
│   │   └── MG90S_规格摘要.md
│   ├── 视觉模块/                       #   视觉识别
│   │   ├── OpenMV_K230_选型对比.md
│   │   ├── OpenMV颜色追踪API.md
│   │   └── OpenMV_MSPM0_串口通信协议.md
│   ├── 激光模块/                       #   激光瞄准
│   │   └── 405nm激光模块_规格要求.md
│   ├── 电源方案/                       #   供电设计
│   │   └── 供电方案设计.md
│   └── 电赛设计报告模板.md             #   报告写作
├── 25年电赛E题/                        # 原题网页存档
├── MSPM0开发指南.pdf                   # TI 72页开发手册
└── EVM用户指南.pdf                     # LaunchPad 25页用户指南
```

---

## 知识库覆盖范围

| 类别 | 器件/主题 | 关键参数 |
|------|-----------|----------|
| **主控** | MSPM0G3507 | Cortex-M0+ @ 80MHz, 128KB Flash, 32KB SRAM, 22 PWM |
| **开发板** | LP-MSPM0G3507 | XDS110-ET 板载调试器, EnergyTrace, BoosterPack |
| **巡迹** | TCRT5000 × 7~8 | IR反射式, 0.2-15mm, 950nm |
| **航向** | MPU6050 (可选) | 6轴 IMU, I2C, DMP |
| **驱动** | TB6612FNG | 双H桥, 1.0A 连续, 3.2A 峰值 |
| **电机** | TT直流 + 编码器 | 1:48, 300RPM @ 6V, Hall 编码器 |
| **舵机** | MG90S × 2 | 金属齿轮, 2.2kg·cm @ 6V, 0.08s/60° |
| **视觉** | OpenMV Cam H7 | STM32F765VI, OV7725, MicroPython |
| **激光** | 405nm 蓝紫激光 | ≤10mW, 可调焦, UV 感光纸 |
| **电源** | 双路独立供电 | 锂电池方案, 独立开关 + LED |

---

## 控制架构

```
┌─────────────────────────────────────────────────┐
│                   系统框图                        │
│                                                  │
│  TCRT5000×7 ──→ MSPM0G3507 ──→ TB6612FNG ──→ 电机 │
│       ↑              ↑                │          │
│  巡迹传感器        UART              PWM         │
│                     │                            │
│  MPU6050 ───────────┤          MG90S ──→ 云台舵机 │
│  (航向角)           │                            │
│                     │                            │
│  OpenMV H7 ─────────┘         405nm激光 ──→ 瞄准  │
│  (颜色追踪)                                      │
└─────────────────────────────────────────────────┘
```

- **巡迹**: 位置式 PID（外环）→ 增量式 PID（内环速度）
- **瞄准**: OpenMV 颜色追踪 → 靶心坐标 → MSPM0 → 舵机 + 激光

---

## 快速开始

1. **阅读赛题**: 打开 `25年电赛E题/` 查看原题要求
2. **速查器件**: 打开 `知识库/README.md` 查看器件清单和引脚估算
3. **环境搭建**: 参考 `MSPM0开发指南.pdf` 安装 CCS IDE + MSPM0 SDK
4. **算法理解**: `知识库/传感器_巡迹/PID控制理论与巡迹算法.md`
5. **运动学**: `知识库/电机驱动/差速转向运动学模型.md`

---

## 相关资源

| 资源 | 链接 |
|------|------|
| TI MSPM0G3507 | [ti.com/product/MSPM0G3507](https://www.ti.com/product/MSPM0G3507) |
| MSPM0 SDK | [ti.com/tool/MSPM0-SDK](https://www.ti.com/tool/MSPM0-SDK) |
| OpenMV 文档 | [docs.openmv.io](https://docs.openmv.io) |
| 电赛培训网 | [nuedc-training.com.cn](https://www.nuedc-training.com.cn) |
| E题开源参考 | [oshwhub.com/yun_7783](https://oshwhub.com/yun_7783/) |

---

## 许可

本仓库为学习备考用途。器件资料版权归原厂商（TI, Vishay, Toshiba 等）所有。
