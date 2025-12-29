# Xindabei_v2 - 电磁循迹小车

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Platform: STM32F4](https://img.shields.io/badge/Platform-STM32F4-blue.svg)](https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html)
[![Framework: FreeRTOS](https://img.shields.io/badge/Framework-FreeRTOS-green.svg)](https://www.freertos.org/)

本项目是一个基于 STM32F401 的电磁循迹智能车软件部分，集成了先进的控制算法（LADRC、S-Curve、PID）、复杂的场景状态机以及完善的参数调试系统，适用于双T电感排布和差速两轮（或四轮鼠车）。为2025湖南科技大学信达杯电磁组参赛作品的软件部分，硬件和车模请参考[这个仓库](https://github.com/Zdyi1/Xindabei_PCB)

## 核心特性

### 1. 控制算法
- **LADRC (线性自抗扰控制)**: 设想用于角速度环或电机控制，在本人测试的时候发现当前硬件下表现不是很好，故没有使用。
- **S-Curve 速度规划**: 实现平滑的加减速过程，减少机械冲击，提升行驶稳定性。
- **EncPLL (编码器观测器)**: 改进的龙伯格观测器，高精度速度估算，有效滤除编码器原始数据的噪声。
- **多级 PID 控制**: 针对循迹偏差和姿态角进行精准控制。循迹环/角速环->角速度环->(电流环 未成功)->电机速度环

### 2. 智能场景识别 (C++ FSM)
- 基于 C++ 有限状态机，比c语言实现更能明确地管理状态：
  - **Normal Tracking**: 基础电磁循迹。
  - **Ring/Loop**: 环岛自动识别、预进入、环内行驶及退出逻辑。
  - **Obstacle Avoidance**: 基于 TOF 传感器的障碍物检测与避障（实际上因为tof不好用用了定时停车）
  - **Safety**: 集成 IMU 翻车检测，异常情况自动紧急制动。
  - **Stop**: 停车状态，本来应该用干簧管检测磁铁停车，后来发现不好用换成了定时停车

### 3. 完善的传感器集成
- **电磁采集**: 支持 ADS1220 (24位高精度 ADC) 和 STM32 内部 ADC 协同工作（后来因为stm32那一路不好用于是中路换成了左电感和右电感之和，也能勉强用）
- **姿态感知**: 集成 LSM6DS3 IMU，通过二阶巴特沃斯滤波器之后传入 Madgwick 实时解算小车姿态，并可获取姿态欧拉角，其偏航角能方便的在角度环使用。
- **测距避障**: VL53L0X TOF 传感器，支持长距离高可靠性测量

### 4. 调试与交互系统
- **参数服务器 (ParamServer)**: 统一管理系统参数，支持运行时动态修改。
- **多维交互**:
  - **OLED 菜单**: 实时显示系统状态、传感器数据，并支持通过按键调整参数。
  - **UART 命令行**: 类似 Shell 的交互界面，支持参数查询、修改及数据波形输出。
- **日志系统**: 分级日志输出 (INFO, WARN, ERROR)，方便定位问题。

## 硬件架构

- **MCU**: STM32F401RCT6
- **传感器**:
  - IMU: LSM6DS3 (SPI)
  - TOF: VL53L0X (I2C)
  - ADC: ADS1220 (SPI) + Internal ADC
- **执行器**:
  - 电机驱动: A4950
  - 编码器: 增量式编码器
  - 指示灯: WS2812B RGB LED (DMA 控制)
- **显示**: SSD1306 OLED (I2C)

## 项目结构

```text
Core/
├── Inc/Src      # 核心逻辑实现
│   ├── track.cpp      # 循迹状态机核心
│   ├── motor.c        # 电机控制与 PLL 测速
│   ├── ladrc.c        # 自抗扰控制器实现
│   ├── s_curve.c      # S 曲线规划
│   ├── imu.c          # IMU 数据处理与滤波
│   ├── param_server.c # 参数管理系统
│   └── ...
Drivers/         # 硬件驱动库 (HAL, CMSIS, VL53L0X)
Middlewares/     # 中间件 (FreeRTOS)
```

## 编译与烧录

项目使用 CMake 构建系统，并使用OpenOCD烧录

### 编译
```bash
mkdir build
cd build
cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Release
ninja
```

### 烧录 (OpenOCD,CMSIS_DAP)
```bash
openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg -c "program build/Xindabei_v2.bin 0x08000000 verify reset exit"
```

## 使用说明

1. **启动**: 上电后，OLED 将显示初始化信息。
2. **调试**:
   - 连接串口 (115200bps)，输入 `var` 查看所有可调参数。
   - 使用 `var [name] [value]` 实时修改参数（如 `var velTracking 1.5`）。
   - 通过 OLED 菜单切换显示页面，观察电感原始值、PID 输出等。
   - 通过取消注释default_task中的print_handler来实时打印数据给vofa使用
3. **运行**: 触发启动信号（按键），小车进入循迹状态，两个按钮对应两个发车状态，一个保底版，一个激进版

## 后言

这是第一次做这么大体量的复杂的嵌入式系统，后面在比较赶的时间还是把车成功的调了出来。在写代码的时候考虑了很多东西，但是在实践的时候有些没成功，有些有更好的方法，最后也是修修补补，看着哪个好用就用哪个。在写项目的时候还是尽力了保持结构清晰，使用cpp是因为状态机部分因为状态多，使用面向对象能显著提高代码可读性。这个项目也算是一个学习过程吧，喜欢可以给大家一点点有限的参考，有想法欢迎讨论😉

## 开源协议

本项目采用 [MIT License](LICENSE) 开源。

