# drone-gesture 详细教程

> 基于 MediaPipe Hands 的 ROS2 手势识别无人机控制系统

---

## 目录

1. [项目概述](#1-项目概述)
2. [硬件环境与依赖](#2-硬件环境与依赖)
3. [系统架构](#3-系统架构)
4. [核心算法详解](#4-核心算法详解)
   - 4.1 [MediaPipe Hands 21 关键点](#41-mediapipe-hands-21-关键点)
   - 4.2 [手势分类算法](#42-手势分类算法)
   - 4.3 [连续速度控制](#43-连续速度控制)
   - 4.4 [安全监控机制](#44-安全监控机制)
5. [模块逐一讲解](#5-模块逐一讲解)
6. [ROS2 通信拓扑](#6-ros2-通信拓扑)
7. [运行与测试](#7-运行与测试)
8. [参数调优指南](#8-参数调优指南)
9. [常见问题](#9-常见问题)

---

## 1. 项目概述

drone-gesture 是一个 ROS2 Python 包，实现通过手势控制无人机的起飞、降落、前进和紧急悬停。系统有两条并行控制通路：

- **离散命令模式** (`gesture_commander`)：识别到特定手势后发送一次性命令（起飞/降落/前进/悬停）
- **连续速度控制模式** (`gesture_velocity_controller`)：张开手掌后，手部的移动实时映射为无人机的 3D 速度命令

两条通路共享同一个手势识别节点 (`gesture_recognizer`)，通过 ROS2 topic `/gesture` 分发。

---

## 2. 硬件环境与依赖

### 硬件平台

| 组件 | 型号 | 说明 |
|------|------|------|
| 机载计算机 | Jetson Nano 4GB | ARM64, Ubuntu 20.04 |
| 飞控 | 雷迅 V5+ | ArduCopter 固件 |
| 电调/电机 | 思翼 A8Mini | 一体化动力模块 |
| 摄像头 | RTSP 网络摄像头 | `rtsp://192.168.1.10:554/stream` |
| 串口连接 | `/dev/ttyTHS1:57600` | MAVROS 与飞控通信 |

### 软件依赖

```bash
# Python 包
pip install mediapipe opencv-python numpy psutil

# ROS2 Humble + MAVROS
sudo apt install ros-humble-mavros ros-humble-cv-bridge ros-humble-image-transport

# 飞控端参数 (ArduCopter)
# SERIAL1_PROTOCOL = 2 (MAVLink2)
# SERIAL1_BAUD = 57600
```

---

## 3. 系统架构

```
摄像头/RTSP
     │
     ▼
┌─────────────────────┐
│ gesture_recognizer   │  30fps 视频流处理
│ MediaPipe Hands      │  21 关键点检测
│ 手势分类 + 防抖      │
└──────┬──────┬────────┘
       │      │
  /gesture  /gesture/image (标注视频)
       │      │
       ▼      ▼
┌──────────────┐  ┌──────────────────────────┐
│gesture_      │  │gesture_velocity_         │
│commander     │  │controller                │
│              │  │                          │
│离散命令模式:  │  │连续速度控制模式:           │
│起飞/降落/前进 │  │手部移动 → 3D速度          │
│紧急悬停      │  │低通滤波+死区+sigmoid      │
└──────┬───────┘  └──────────┬───────────────┘
       │                     │
       ▼                     ▼
┌──────────────────────────────────┐
│  MAVROS → ArduCopter 飞控       │
│  /mavros/setpoint_velocity      │
│  /mavros/setpoint_position      │
│  /mavros/cmd/arming             │
│  /mavros/set_mode               │
└──────────────────────────────────┘

独立监控节点:
┌─────────────────┐  ┌─────────────────┐
│safety_monitor   │  │diagnostics      │
│电量/连接/心跳    │  │CPU/内存/FPS     │
│自动紧急降落      │  │系统健康聚合      │
└─────────────────┘  └─────────────────┘
```

---

## 4. 核心算法详解

### 4.1 MediaPipe Hands 21 关键点

MediaPipe Hands 检测手部 21 个关键点，每个点包含 `(x, y, z)` 归一化坐标：

```
        8(TIP)        ← 食指尖
        7(DIP)
        6(PIP)
   4    5(MCP)
   3       13  14  15  16
   2       12  11  10  9
   1       17  18  19  20
   0(WRIST)
  拇指        中指  无名指  小指
```

坐标系说明：
- `x`: 0→1 从左到右
- `y`: 0→1 从上到下（图像坐标系，y 轴向下）
- `z`: 越远离摄像头值越大

### 4.2 手势分类算法

`gesture_definitions.py` 中的 `classify_gesture()` 使用**双重检测投票**判断每根手指是否伸直：

**方法 1：y 坐标比较（快速）**
```python
y_extended = tip.y < pip.y  # 指尖在指间关节上方 → 伸直
```

**方法 2：向量夹角（鲁棒）**
```python
# MCP→PIP 和 PIP→TIP 两个向量的夹角
v1 = (pip.x - mcp.x, pip.y - mcp.y)
v2 = (tip.x - pip.x, tip.y - pip.y)
cos_angle = dot(v1, v2) / (|v1| * |v2|)
angle_extended = cos_angle > 0.5  # 约 60° 以内算伸直
```

**投票规则**：两种方法都说伸直才算伸直（AND 逻辑），降低误检。

**拇指特殊处理**：因为拇指横向运动，用拇指尖到手腕的距离与拇指 IP 关节到手腕的距离比较：
```python
tip_dist > ip_dist * 1.1  →  拇指伸直
```

**手势分类规则**：

| 手势 | 条件 | 映射命令 |
|------|------|---------|
| OPEN_PALM | 所有 5 指伸直 | 起飞 / 启用速度控制 |
| FIST | 所有 5 指弯曲 | 降落 / 停止控制 |
| THUMBS_UP | 拇指伸直 + 其余 4 指弯曲 | 前进 |
| OK_SIGN | 拇指-食指 pinch < 0.08 + 其余 3 指至少 2 个伸直 | 功能键 / 紧急悬停 |

### 4.3 连续速度控制

`gesture_velocity_controller.py` 实现了从手部运动到无人机速度的完整信号处理链：

```
手掌中心坐标序列
      │
      ▼
┌─────────────────┐
│ HandVelocity    │  EWMA 速度估计
│ Estimator       │  v[n] = β·Δpos/Δt + (1-β)·v[n-1]
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ LowPassFilter   │  一阶低通滤波
│ α=0.3           │  y[n] = α·x[n] + (1-α)·y[n-1]
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ DeadZoneFilter  │  死区消除微小抖动
│ threshold=0.005 │  |input| < threshold → 0
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ tanh 非线性映射  │  小动作低灵敏度, 大动作不超速
│ vx = max_vel · tanh(sensitivity · (-fy))
└────────┬────────┘
         │
         ▼
   /gesture/velocity_cmd
         │
         ▼
  gesture_commander (转发到 MAVROS)
         │
         ▼
   /mavros/setpoint_velocity/cmd_vel
```

**坐标映射关系**（摄像头正对操作者）：

| 手部动作 | MediaPipe 变化 | 无人机动作 |
|---------|---------------|-----------|
| 手向右移 | x 增大 | 右平移 (vy+) |
| 手向上抬 | y 减小 | 前进 (vx+) |
| 手远离摄像头 | z 减小 | 上升 (vz+) |

### 4.4 安全监控机制

`safety_monitor.py` 以 2Hz 频率检查三项指标：

| 检查项 | WARNING 条件 | CRITICAL 条件 |
|--------|-------------|--------------|
| 飞控连接 | - | 未连接 / 超时 5s |
| 电池电量 | ≤ 30% | ≤ 15% |
| 手势心跳 | 超时 10s | - |

当安全等级为 CRITICAL 且飞控已解锁时，自动触发紧急降落（切换到 LAND 模式）。使用 `_emergency_triggered` 标志确保只触发一次。

---

## 5. 模块逐一讲解

### 5.1 gesture_recognizer.py — 手势识别节点

**职责**：读取视频流 → MediaPipe 检测 → 分类手势 → 发布结果

**关键流程**：
1. 以 30fps 定时器驱动 `process_frame()`
2. BGR→RGB 转换后送入 MediaPipe Hands
3. 调用 `classify_gesture(landmarks)` 分类
4. **防抖**：连续 2 帧相同手势才发布（防止闪烁）
5. 发布 JSON 到 `/gesture`，包含手势 ID、置信度、21 个关键点坐标

**ROS2 参数**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `video_source` | 0 | 摄像头索引或 RTSP URL |
| `model_complexity` | 1 | MediaPipe 模型复杂度 (0/1) |
| `min_detection_confidence` | 0.7 | 最低检测置信度 |
| `min_tracking_confidence` | 0.5 | 最低跟踪置信度 |
| `publish_annotated_video` | True | 是否发布标注图像 |

### 5.2 gesture_commander.py — 离散命令节点

**职责**：监听手势 → 状态机转换 → 发送飞控命令

**状态机**：

```
IDLE ──(张开手掌)──→ ARMING ──(解锁成功)──→ TAKING_OFF ──(超时8s)──→ HOVERING
                                                                        │
                                              ┌──(竖拇指)──→ MOVING ──┘
                                              │                │
                                              │           (超时/OK手势)
                                              │                │
                                              └────────────────┘
HOVERING ──(握拳)──→ LANDING ──(落地)──→ IDLE
```

**防抖机制**：同一手势在 `gesture_debounce_time`（默认 1.5s）内不重复响应。

### 5.3 gesture_velocity_controller.py — 连续速度控制节点

**职责**：手部运动 → 速度估计 → 滤波 → 无人机速度命令

**三个子模块**：
- `HandVelocityEstimator`：从手掌中心坐标序列估计速度（EWMA 平滑）
- `LowPassFilter`：一阶 IIR 低通滤波，消除高频抖动
- `DeadZoneFilter`：死区滤波，消除静止时的微小漂移

**控制模式**：
- `idle`：握拳或无手 → 不发送速度
- `velocity`：张开手 → 用手部运动控制
- `discrete`：OK 手势 → 预留离散命令

### 5.4 safety_monitor.py — 安全监控节点

**职责**：监控飞控状态、电量、手势心跳，必要时自动紧急降落

### 5.5 diagnostics.py — 诊断节点

**职责**：聚合系统健康状态（CPU、内存、手势 FPS、飞控状态、安全等级），以 0.5Hz 发布并支持服务查询。

---

## 6. ROS2 通信拓扑

### Topics

| Topic | 类型 | 发布者 | 订阅者 |
|-------|------|--------|--------|
| `/gesture` | String (JSON) | gesture_recognizer | commander, velocity_controller, safety_monitor, diagnostics |
| `/gesture/image` | Image | gesture_recognizer | (可视化工具) |
| `/gesture/velocity_status` | String (JSON) | velocity_controller | (监控) |
| `/drone/safety_status` | String (JSON) | safety_monitor | diagnostics |
| `/drone/diagnostics` | String (JSON) | diagnostics | (监控) |
| `/gesture/velocity_cmd` | TwistStamped | velocity_controller | commander |
| `/mavros/setpoint_velocity/cmd_vel` | TwistStamped | commander | MAVROS |
| `/mavros/setpoint_position/local` | PoseStamped | commander | MAVROS |
| `/mavros/state` | State | MAVROS | commander, safety_monitor |
| `/mavros/battery` | BatteryState | MAVROS | safety_monitor |

### Services

| 服务 | 类型 | 客户端 |
|------|------|--------|
| `/mavros/cmd/arming` | CommandBool | commander |
| `/mavros/set_mode` | SetMode | commander, safety_monitor |
| `/drone/get_diagnostics` | Trigger | (外部查询) |

---

## 7. 运行与测试

### 7.1 编译

```bash
cd ~/ros2_ws
colcon build --packages-select drone_gesture
source install/setup.bash
```

### 7.2 单独运行各节点（测试模式）

```bash
# 手势识别 (用摄像头 0)
ros2 run drone_gesture gesture_recognizer --ros-args -p video_source:=0

# 离散命令 (测试模式, 不连飞控)
ros2 run drone_gesture gesture_commander --ros-args -p test_mode:=true

# 连续速度控制 (测试模式)
ros2 run drone_gesture gesture_velocity_controller --ros-args -p test_mode:=true
```

### 7.3 独立测试脚本（不需要 ROS2）

```bash
# 用摄像头
python3 drone_gesture/test_gesture_standalone.py

# 用 RTSP 流
python3 drone_gesture/test_gesture_standalone.py rtsp://192.168.1.10:554/stream

# 用视频文件
python3 drone_gesture/test_gesture_standalone.py test_video.mp4
```

### 7.4 单元测试

```bash
# 运行全部测试
python3 -m unittest discover -s test -p "test_*.py" -v

# 单独运行
python3 -m unittest test.test_gesture_definitions -v
python3 -m unittest test.test_velocity_controller -v
python3 -m unittest test.test_safety_monitor -v
```

### 7.5 全系统启动

```bash
# 测试模式 (用摄像头, 不连飞控)
ros2 launch drone_gesture gesture_full.launch.py test_mode:=true

# 真机模式
ros2 launch drone_gesture gesture_full.launch.py
```

---

## 8. 参数调优指南

### 手势识别灵敏度

```bash
# 提高检测灵敏度 (可能增加误检)
ros2 run drone_gesture gesture_recognizer --ros-args \
  -p min_detection_confidence:=0.5 \
  -p min_tracking_confidence:=0.3

# 降低误检 (可能漏检)
ros2 run drone_gesture gesture_recognizer --ros-args \
  -p min_detection_confidence:=0.8 \
  -p min_tracking_confidence:=0.6
```

### 速度控制平滑度

```bash
ros2 run drone_gesture gesture_velocity_controller --ros-args \
  -p smoothing_alpha:=0.2 \      # 更平滑 (延迟增大)
  -p dead_zone:=0.01 \           # 更大的死区 (减少漂移)
  -p velocity_sensitivity:=1.5 \ # 降低灵敏度
  -p max_velocity:=0.8           # 降低最大速度
```

### 安全阈值

```bash
ros2 run drone_gesture safety_monitor --ros-args \
  -p battery_warning_pct:=35.0 \     # 提前提醒
  -p battery_critical_pct:=20.0 \    # 提前紧急降落
  -p heartbeat_timeout_sec:=5.0 \    # 更严格的超时
  -p auto_land_on_critical:=true
```

---

## 9. 常见问题

### Q: MediaPipe 检测不到手？

- 确保光线充足，手部在画面中清晰可见
- 降低 `min_detection_confidence` 到 0.5
- 检查摄像头是否正常打开（`cv2.VideoCapture` 返回 `True`）

### Q: 手势识别抖动严重？

- 增加防抖帧数（当前为 2 帧，可在代码中改为 3-4 帧）
- 使用连续速度控制模式替代离散命令模式

### Q: 无人机速度命令不生效？

- 确认 MAVROS 已连接飞控（`/mavros/state` 的 `connected` 为 `True`）
- 确认飞控已解锁（`armed` 为 `True`）
- 确认飞控在 GUIDED 模式

### Q: Windows 上无法运行？

- `test_gesture_standalone.py` 可在 Windows 直接运行（只需 opencv + mediapipe）
- ROS2 节点需要在 WSL2 或 Linux 环境中运行
- 单元测试使用 `unittest` 框架，可在 Windows 运行；需要 rclpy 的测试会自动跳过
