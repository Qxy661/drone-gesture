"""
手势命令映射 ROS2 节点
将识别到的手势转换为无人机控制命令
通过 MAVROS 发送给飞控 (ArduCopter)
"""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

from enum import Enum

from drone_gesture.gesture_definitions import GestureID


class DroneState(str, Enum):
    """无人机状态枚举"""
    IDLE = 'idle'
    ARMING = 'arming'
    TAKING_OFF = 'taking_off'
    HOVERING = 'hovering'
    MOVING = 'moving'
    LANDING = 'landing'
    DISARMING = 'disarming'


class GestureCommanderNode(Node):
    def __init__(self):
        super().__init__('gesture_commander')

        self.declare_parameter('takeoff_altitude', 1.0)
        self.declare_parameter('forward_velocity', 0.5)
        self.declare_parameter('forward_duration', 2.0)
        self.declare_parameter('gesture_debounce_time', 1.5)
        self.declare_parameter('test_mode', False)

        self.takeoff_alt = self.get_parameter('takeoff_altitude').value
        self.forward_vel = self.get_parameter('forward_velocity').value
        self.forward_dur = self.get_parameter('forward_duration').value
        self.debounce_time = self.get_parameter('gesture_debounce_time').value
        self.test_mode = self.get_parameter('test_mode').value

        self.drone_state = DroneState.IDLE
        self.fcu_connected = False
        self.armed = False
        self.current_mode = ''
        self.last_gesture_time = 0.0
        self.last_gesture_id = GestureID.NONE
        self.move_start_time = 0.0
        self.takeoff_start_time = 0.0
        self.land_start_time = 0.0
        self.arming_start_time = 0.0
        self.arm_retry_count = 0
        self.mode_retry_count = 0
        self.max_arm_retries = 5
        self.arm_timeout = 10.0  # 解锁超时 (秒)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.gesture_sub = self.create_subscription(
            String, '/gesture', self.gesture_callback, qos)

        # 订阅速度控制器的输出，在 HOVERING/MOVING 状态下转发到 MAVROS
        self.vel_cmd_sub = self.create_subscription(
            TwistStamped, '/gesture/velocity_cmd', self._velocity_cmd_cb, 10)

        if not self.test_mode:
            self.state_sub = self.create_subscription(
                State, '/mavros/state', self.state_callback, 10)
            self.vel_pub = self.create_publisher(
                TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
            self.pos_pub = self.create_publisher(
                PoseStamped, '/mavros/setpoint_position/local', 10)
            self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
            self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
            self.get_logger().info('手势命令节点已启动 (真机模式)')
        else:
            self.get_logger().info('手势命令节点已启动 (测试模式)')

        self.timer = self.create_timer(0.1, self.update_state)

    def state_callback(self, msg):
        self.fcu_connected = msg.connected
        self.armed = msg.armed
        self.current_mode = msg.mode

    def _velocity_cmd_cb(self, msg):
        """转发速度控制器的命令到 MAVROS (仅在悬停/手动控制状态)"""
        if self.drone_state == DroneState.HOVERING and not self.test_mode:
            self.vel_pub.publish(msg)

    def gesture_callback(self, msg):
        try:
            data = json.loads(msg.data)
            gesture_id = GestureID(data['gesture_id'])
        except (json.JSONDecodeError, KeyError, ValueError):
            return

        now = time.time()
        if gesture_id == self.last_gesture_id and now - self.last_gesture_time < self.debounce_time:
            return

        self.last_gesture_id = gesture_id
        self.last_gesture_time = now
        self.get_logger().info(
            f'收到手势: {data.get("gesture", "unknown")} 当前状态: {self.drone_state}')
        self.execute_gesture(gesture_id)

    def execute_gesture(self, gesture_id):
        if gesture_id == GestureID.OPEN_PALM:
            self._handle_takeoff()
        elif gesture_id == GestureID.FIST:
            self._handle_land()
        elif gesture_id == GestureID.THUMBS_UP:
            self._handle_move_forward()
        elif gesture_id == GestureID.OK_SIGN:
            self._handle_toggle_mode()

    def _handle_takeoff(self):
        if self.drone_state == DroneState.IDLE:
            self.get_logger().info('>>> 起飞指令')
            if self.test_mode:
                self.drone_state = DroneState.TAKING_OFF
                self.takeoff_start_time = time.time()
                self.get_logger().info('[测试] 模拟起飞')
            else:
                self._set_mode('GUIDED')
                self._arm(True)
                self.drone_state = DroneState.ARMING
                self.arming_start_time = time.time()
                self.arm_retry_count = 0

    def _handle_land(self):
        if self.drone_state in (DroneState.HOVERING, DroneState.MOVING):
            self.get_logger().info('>>> 降落指令')
            self.land_start_time = time.time()
            if self.test_mode:
                self.drone_state = DroneState.LANDING
                self.get_logger().info('[测试] 模拟降落')
            else:
                self._set_mode('LAND')
                self.drone_state = DroneState.LANDING

    def _handle_move_forward(self):
        if self.drone_state == DroneState.HOVERING:
            self.get_logger().info(f'>>> 前进指令 ({self.forward_vel} m/s)')
            self.drone_state = DroneState.MOVING
            self.move_start_time = time.time()
            if not self.test_mode:
                self._send_velocity(self.forward_vel, 0.0, 0.0)

    def _handle_toggle_mode(self):
        """OK手势: 紧急停止 — 立即悬停"""
        self.get_logger().info('>>> OK手势 - 紧急悬停')
        if self.drone_state == DroneState.MOVING:
            self.drone_state = DroneState.HOVERING
            if not self.test_mode:
                self._send_velocity(0.0, 0.0, 0.0)

    def _send_velocity(self, vx, vy, vz):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        self.vel_pub.publish(msg)

    def _set_mode(self, mode):
        if not self.set_mode_client.service_is_ready():
            self.get_logger().warn('set_mode 服务未就绪，跳过')
            return
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_client.call_async(req)
        future.add_done_callback(lambda f: self._on_set_mode_done(f, mode))

    def _on_set_mode_done(self, future, mode):
        try:
            result = future.result()
            if result.mode_sent:
                self.get_logger().info(f'模式切换 {mode}: 成功')
            else:
                self.get_logger().warn(f'模式切换 {mode}: 失败')
                if self.mode_retry_count < self.max_arm_retries:
                    self.mode_retry_count += 1
                    self._set_mode(mode)
        except Exception as e:
            self.get_logger().error(f'模式切换异常: {e}')

    def _arm(self, arm):
        if not self.arming_client.service_is_ready():
            self.get_logger().warn('arming 服务未就绪，跳过')
            return
        req = CommandBool.Request()
        req.value = arm
        future = self.arming_client.call_async(req)
        future.add_done_callback(lambda f: self._on_arm_done(f, arm))

    def _on_arm_done(self, future, arm):
        try:
            result = future.result()
            action = "解锁" if arm else "上锁"
            if result.success:
                self.get_logger().info(f'{action}: 成功')
            else:
                self.get_logger().warn(f'{action}: 失败')
                if self.arm_retry_count < self.max_arm_retries:
                    self.arm_retry_count += 1
                    self._arm(arm)
        except Exception as e:
            self.get_logger().error(f'解锁异常: {e}')

    def update_state(self):
        now = time.time()
        if self.drone_state == DroneState.ARMING:
            # 解锁超时检查
            if now - self.arming_start_time > self.arm_timeout:
                self.get_logger().error('解锁超时, 返回 IDLE')
                self.drone_state = DroneState.IDLE
                return
            # 等待飞控解锁成功, 然后发送起飞目标点
            if self.armed:
                if not self.test_mode:
                    msg = PoseStamped()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.pose.position.z = self.takeoff_alt
                    self.pos_pub.publish(msg)
                self.drone_state = DroneState.TAKING_OFF
                self.takeoff_start_time = time.time()  # 从此刻开始计时
                self.get_logger().info('已解锁, 开始起飞')
        elif self.drone_state == DroneState.TAKING_OFF:
            # 持续发送起飞目标点 (MAVROS 需要持续 setpoint)
            if not self.test_mode:
                msg = PoseStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.pose.position.z = self.takeoff_alt
                self.pos_pub.publish(msg)
            # 超时或测试模式下切换到悬停
            if self.test_mode:
                if now - self.takeoff_start_time > 3.0:
                    self.drone_state = DroneState.HOVERING
                    self.get_logger().info('悬停 (测试)')
            elif now - self.takeoff_start_time > 8.0:
                self.drone_state = DroneState.HOVERING
                self.get_logger().info('悬停')
        elif self.drone_state == DroneState.MOVING:
            if now - self.move_start_time > self.forward_dur:
                self.drone_state = DroneState.HOVERING
                if not self.test_mode:
                    self._send_velocity(0.0, 0.0, 0.0)
                self.get_logger().info('前进完成')
            elif not self.test_mode:
                self._send_velocity(self.forward_vel, 0.0, 0.0)
        elif self.drone_state == DroneState.LANDING:
            if self.test_mode:
                if self.land_start_time > 0 and now - self.land_start_time > 3.0:
                    self.drone_state = DroneState.IDLE
                    self.get_logger().info('已降落 (测试)')
            elif not self.armed:
                self.drone_state = DroneState.IDLE
                self.get_logger().info('已降落')


def main(args=None):
    rclpy.init(args=args)
    node = GestureCommanderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
