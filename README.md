# drone-gesture

基于 MediaPipe Hands 的 ROS2 手势识别无人机控制系统。

## 功能

- 实时手势识别 (MediaPipe Hands 21 关键点)
- 离散命令模式: 张开手掌=起飞, 握拳=降落, 竖拇指=前进, OK=功能键
- 连续速度控制: 手部移动映射到无人机速度 (低通滤波+死区+sigmoid)
- 安全监控: 电量/连接/心跳/自动紧急降落
- 系统诊断: CPU/内存/FPS 聚合

## 架构

```
摄像头/RTSP -> gesture_recognizer -> gesture_commander -> MAVROS -> 飞控
                                   -> gesture_velocity_controller -> MAVROS
              safety_monitor (独立监控)
              diagnostics (系统状态)
```

## Nodes

| Node | 功能 |
|------|------|
| gesture_recognizer | MediaPipe 视频流手势识别 |
| gesture_commander | 手势->飞控命令 (离散模式) |
| gesture_velocity_controller | 手势->连续速度控制 |
| safety_monitor | 电量/连接/心跳监控 |
| diagnostics | 系统诊断聚合 |

## 快速开始

```bash
# 安装依赖
pip install mediapipe opencv-python numpy psutil
sudo apt install ros-humble-mavros ros-humble-cv-bridge

# 编译
cd ros2_ws && colcon build --packages-select drone_gesture
source install/setup.bash

# 测试模式 (用摄像头, 不连飞控)
ros2 launch drone_gesture gesture_full.launch.py test_mode:=true
```

## 手势方案

| 手势 | 离散模式 | 连续模式 |
|------|---------|---------|
| 张开手掌 | 起飞 | 启用速度控制 |
| 握拳 | 降落 | 停止控制 |
| 竖拇指 | 前进 | - |
| OK手势 | 功能键 | - |

## 硬件

- Jetson Nano + 雷迅 V5+ (ArduCopter) + 思翼 A8Mini
- 或任何支持 ROS2 + MAVROS 的飞控

## License

MIT
