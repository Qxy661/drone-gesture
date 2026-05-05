"""
手势连续速度控制器
Continuous Gesture Velocity Controller

核心创新: 不是离散命令(起飞/降落), 而是用手的运动来连续控制无人机

原理:
1. MediaPipe 检测手部 21 个关键点
2. 计算手掌中心(Palm Center)在连续帧之间的位移
3. 位移方向和速度映射为无人机的 3D 速度命令
4. 手势模式切换: 张开手=控制模式, 握拳=停止

控制映射:
  手向前移动 -> 无人机前进 (body frame)
  手向后移动 -> 无人机后退
  手向左/右移动 -> 无人机左/右平移
  手向上/下移动 -> 无人机上升/下降
  手移动速度 -> 无人机速度 (非线性映射, 越快越敏感)

优势:
- 直觉控制: 像用手推着无人机飞
- 连续控制: 不需要反复做手势
- 速度可调: 手动快=飞得快, 动慢=飞得慢
- 抗抖动: 低通滤波 + 死区消除手部自然抖动
"""
import json
import math
import time
from collections import deque
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped

from drone_gesture.gesture_definitions import GestureID


class LowPassFilter:
    """一阶低通滤波器
    y[n] = alpha * x[n] + (1-alpha) * y[n-1]
    alpha 越小越平滑, 但延迟越大
    用于消除手部抖动
    """
    def __init__(self, alpha: float = 0.3, dim: int = 3):
        self.alpha = alpha
        self.state = [0.0] * dim

    def update(self, x: list) -> list:
        self.state = [
            self.alpha * xi + (1 - self.alpha) * si
            for xi, si in zip(x, self.state)
        ]
        return list(self.state)

    def reset(self):
        self.state = [0.0] * len(self.state)


class DeadZoneFilter:
    """死区滤波器
    |input| < threshold -> output = 0
    用于消除手部微小抖动导致的无人机漂移
    """
    def __init__(self, threshold: float = 0.01):
        self.threshold = threshold

    def apply(self, value: float) -> float:
        if abs(value) < self.threshold:
            return 0.0
        # 恢复连续性: 死区外按比例缩放
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.threshold)


class HandVelocityEstimator:
    """手部速度估计器
    从 MediaPipe 关键点序列中估计手掌的运动速度

    使用指数加权移动平均 (EWMA) 估计速度:
    v[n] = beta * (pos[n] - pos[n-1]) / dt + (1-beta) * v[n-1]
    """
    def __init__(self, smoothing: float = 0.4):
        self.beta = smoothing
        self.prev_pos = None
        self.prev_time = None
        self.velocity = [0.0, 0.0, 0.0]

    def update(self, palm_x: float, palm_y: float, palm_z: float,
               timestamp: float) -> Tuple[float, float, float]:
        """输入归一化手掌坐标, 输出归一化速度"""
        if self.prev_pos is None or self.prev_time is None:
            self.prev_pos = (palm_x, palm_y, palm_z)
            self.prev_time = timestamp
            return (0.0, 0.0, 0.0)

        dt = timestamp - self.prev_time
        if dt < 0.001 or dt > 1.0:  # 防止异常dt
            self.prev_pos = (palm_x, palm_y, palm_z)
            self.prev_time = timestamp
            return tuple(self.velocity)

        # 原始速度
        raw_vx = (palm_x - self.prev_pos[0]) / dt
        raw_vy = (palm_y - self.prev_pos[1]) / dt
        raw_vz = (palm_z - self.prev_pos[2]) / dt

        # EWMA 平滑
        self.velocity[0] = self.beta * raw_vx + (1 - self.beta) * self.velocity[0]
        self.velocity[1] = self.beta * raw_vy + (1 - self.beta) * self.velocity[1]
        self.velocity[2] = self.beta * raw_vz + (1 - self.beta) * self.velocity[2]

        self.prev_pos = (palm_x, palm_y, palm_z)
        self.prev_time = timestamp

        return tuple(self.velocity)

    def reset(self):
        self.prev_pos = None
        self.prev_time = None
        self.velocity = [0.0, 0.0, 0.0]


class GestureVelocityControllerNode(Node):
    """手势连续速度控制节点

    订阅 /gesture_raw (原始手势关键点) 和 /gesture (离散手势)
    发布 /mavros/setpoint_velocity/cmd_vel (连续速度命令)

    控制模式:
    - IDLE: 手势为握拳或无手 -> 不发送速度
    - VELOCITY_CONTROL: 张开手 -> 用手部运动控制无人机速度
    - DISCRETE: OK手势 -> 触发离散命令(拍照等)
    """
    def __init__(self):
        super().__init__('gesture_velocity_controller')

        # 参数
        self.declare_parameter('max_velocity', 1.0)       # m/s
        self.declare_parameter('velocity_sensitivity', 2.0)  # 速度放大系数
        self.declare_parameter('smoothing_alpha', 0.3)     # 低通滤波系数
        self.declare_parameter('dead_zone', 0.005)         # 死区阈值
        self.declare_parameter('test_mode', True)

        self.max_vel = self.get_parameter('max_velocity').value
        self.sensitivity = self.get_parameter('velocity_sensitivity').value
        alpha = self.get_parameter('smoothing_alpha').value
        dz = self.get_parameter('dead_zone').value
        self.test_mode = self.get_parameter('test_mode').value

        # 子模块
        self.vel_filter = LowPassFilter(alpha=alpha, dim=3)
        self.deadzone = DeadZoneFilter(threshold=dz)
        self.velocity_estimator = HandVelocityEstimator(smoothing=0.4)

        # 状态
        self.control_mode = "idle"  # idle / velocity / discrete
        self.current_gesture = GestureID.NONE
        self.palm_center = (0.0, 0.0, 0.0)
        self.enabled = False  # 张开手才启用连续控制

        # 订阅
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(String, '/gesture', self._gesture_cb, qos)

        # 发布
        if not self.test_mode:
            self.vel_pub = self.create_publisher(
                TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        self.status_pub = self.create_publisher(
            String, '/gesture/velocity_status', 10)

        # 控制循环 20Hz
        self.timer = self.create_timer(0.05, self._control_loop)

        mode = "TEST" if self.test_mode else "REAL"
        self.get_logger().info(
            f'GestureVelocityController started ({mode})')

    def _gesture_cb(self, msg):
        try:
            data = json.loads(msg.data)
            gesture_id = GestureID(data['gesture_id'])
            self.current_gesture = gesture_id

            # 从 MediaPipe landmarks 提取手掌中心
            # landmarks[9] = 中指 MCP, 近似手掌中心
            if 'landmarks' in data and len(data['landmarks']) > 9:
                lm = data['landmarks'][9]  # 中指根部
                self.palm_center = (lm['x'], lm['y'], lm['z'])
        except (json.JSONDecodeError, KeyError, ValueError):
            pass

    def _control_loop(self):
        now = time.time()

        # 模式切换
        if self.current_gesture == GestureID.OPEN_PALM:
            if not self.enabled:
                self.enabled = True
                self.velocity_estimator.reset()
                self.vel_filter.reset()
                self.get_logger().info('>>> Velocity control ENABLED')
            self.control_mode = "velocity"
        elif self.current_gesture == GestureID.FIST:
            self.enabled = False
            self.control_mode = "idle"
        elif self.current_gesture == GestureID.OK_SIGN:
            self.control_mode = "discrete"

        vx, vy, vz, vyaw = 0.0, 0.0, 0.0, 0.0

        if self.control_mode == "velocity" and self.enabled:
            # 估计手部速度
            raw_vel = self.velocity_estimator.update(
                self.palm_center[0], self.palm_center[1],
                self.palm_center[2], now)

            # 低通滤波去抖动
            filtered = self.vel_filter.update(list(raw_vel))

            # 死区处理
            fx = self.deadzone.apply(filtered[0])
            fy = self.deadzone.apply(filtered[1])
            fz = self.deadzone.apply(filtered[2])

            # 非线性映射: 小动作->小速度, 大动作->大速度
            # 使用 sigmoid-like 映射: v = max_vel * tanh(sensitivity * input)
            vx = self.max_vel * math.tanh(self.sensitivity * (-fy))  # 手向前 -> 无人机前进
            vy = self.max_vel * math.tanh(self.sensitivity * fx)     # 手向右 -> 无人机右移
            vz = self.max_vel * math.tanh(self.sensitivity * (-fz)) * 0.5  # 垂直减速

        # 发布
        if not self.test_mode and self.control_mode == "velocity":
            vel_msg = TwistStamped()
            vel_msg.header.stamp = self.get_clock().now().to_msg()
            vel_msg.twist.linear.x = vx
            vel_msg.twist.linear.y = vy
            vel_msg.twist.linear.z = vz
            vel_msg.twist.angular.z = vyaw
            self.vel_pub.publish(vel_msg)
        elif self.test_mode and self.control_mode == "velocity":
            self.get_logger().info(
                f'[VEL] mode={self.control_mode} '
                f'vel=({vx:.3f}, {vy:.3f}, {vz:.3f})',
                throttle_duration_sec=0.5)

        # 发布状态
        status = {
            'mode': self.control_mode,
            'enabled': self.enabled,
            'gesture': self.current_gesture.value,
            'palm_pos': list(self.palm_center),
            'velocity_cmd': [vx, vy, vz],
        }
        out = String()
        out.data = json.dumps(status)
        self.status_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GestureVelocityControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
