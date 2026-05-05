import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode


class SafetyLevel:
    OK = "ok"
    WARNING = "warning"
    CRITICAL = "critical"


class SafetyMonitorNode(Node):
    """安全监控节点 - 监控飞控连接、电量、手势心跳
    Safety monitor - checks FCU connection, battery, gesture heartbeat
    """
    def __init__(self):
        super().__init__("safety_monitor")

        # 参数 / Parameters
        self.declare_parameter("battery_warning_pct", 30.0)
        self.declare_parameter("battery_critical_pct", 15.0)
        self.declare_parameter("heartbeat_timeout_sec", 10.0)
        self.declare_parameter("connection_timeout_sec", 5.0)
        self.declare_parameter("auto_land_on_critical", True)

        self.battery_warn = self.get_parameter("battery_warning_pct").value
        self.battery_crit = self.get_parameter("battery_critical_pct").value
        self.heartbeat_timeout = self.get_parameter("heartbeat_timeout_sec").value
        self.conn_timeout = self.get_parameter("connection_timeout_sec").value
        self.auto_land = self.get_parameter("auto_land_on_critical").value

        # 状态 / State
        self.fcu_connected = False
        self.fcu_armed = False
        self.fcu_mode = ""
        self.battery_pct = 100.0
        self.last_heartbeat = 0.0
        self.last_state_time = 0.0
        self.safety_level = SafetyLevel.OK
        self.warnings = []

        # QoS: BEST_EFFORT for sensor data
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # 订阅 / Subscriptions
        self.state_sub = self.create_subscription(
            State, "/mavros/state", self._state_cb, 10)
        self.battery_sub = self.create_subscription(
            BatteryState, "/mavros/battery", self._battery_cb, qos)
        self.gesture_sub = self.create_subscription(
            String, "/gesture", self._gesture_cb, qos)

        # 发布 / Publishers
        self.status_pub = self.create_publisher(
            String, "/drone/safety_status", 10)

        # 服务客户端 - 紧急模式切换
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")

        # 定时检查 2Hz
        self.timer = self.create_timer(0.5, self._check_safety)
        self.get_logger().info("Safety monitor started")

    def _state_cb(self, msg):
        self.fcu_connected = msg.connected
        self.fcu_armed = msg.armed
        self.fcu_mode = msg.mode
        self.last_state_time = time.time()

    def _battery_cb(self, msg):
        # BatteryState.percentage: 0.0~1.0
        if msg.percentage >= 0:
            self.battery_pct = msg.percentage * 100.0

    def _gesture_cb(self, msg):
        self.last_heartbeat = time.time()

    def _check_safety(self):
        now = time.time()
        self.warnings = []

        # 1) 飞控连接检查
        if not self.fcu_connected:
            self.warnings.append("FCU_NOT_CONNECTED")
        elif now - self.last_state_time > self.conn_timeout:
            self.warnings.append("FCU_CONNECTION_LOST")

        # 2) 电量检查
        if self.battery_pct <= self.battery_crit:
            self.warnings.append(f"BATTERY_CRITICAL({self.battery_pct:.0f}%)")
        elif self.battery_pct <= self.battery_warn:
            self.warnings.append(f"BATTERY_LOW({self.battery_pct:.0f}%)")

        # 3) 手势节点心跳
        if self.last_heartbeat > 0 and now - self.last_heartbeat > self.heartbeat_timeout:
            self.warnings.append("GESTURE_HEARTBEAT_LOST")

        # 判定安全等级
        crit_prefixes = ("FCU_NOT_CONNECTED", "FCU_CONNECTION_LOST", "BATTERY_CRITICAL")
        has_critical = any(w.startswith(crit_prefixes) for w in self.warnings)

        if has_critical:
            self.safety_level = SafetyLevel.CRITICAL
            if self.auto_land and self.fcu_armed:
                self._emergency_land()
        elif self.warnings:
            self.safety_level = SafetyLevel.WARNING
        else:
            self.safety_level = SafetyLevel.OK

        # 发布安全状态 JSON
        status = {
            "level": self.safety_level,
            "battery_pct": round(self.battery_pct, 1),
            "fcu_connected": self.fcu_connected,
            "fcu_armed": self.fcu_armed,
            "fcu_mode": self.fcu_mode,
            "warnings": self.warnings,
            "timestamp": now,
        }
        out = String()
        out.data = json.dumps(status)
        self.status_pub.publish(out)

        if self.warnings:
            self.get_logger().warn(f"Safety: {self.warnings}")

    def _emergency_land(self):
        """紧急降落 - 切换到 LAND 模式"""
        if not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("set_mode unavailable")
            return
        req = SetMode.Request()
        req.custom_mode = "LAND"
        self.set_mode_client.call_async(req)
        self.get_logger().warn(">>> Emergency LAND triggered")


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
