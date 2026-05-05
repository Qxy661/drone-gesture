import json
import time
import psutil
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from mavros_msgs.msg import State


class DiagnosticsNode(Node):
    """系统诊断节点 - 聚合各节点健康状态，提供查询服务
    Diagnostics - aggregates health from all nodes, provides query service
    """
    def __init__(self):
        super().__init__("diagnostics")

        self.safety_status = {}
        self.fcu_state = {}
        self.gesture_count = 0
        self.last_gesture_time = 0.0
        self.gesture_fps = 0.0
        self._gesture_window = []
        self.start_time = time.time()

        # 订阅
        self.create_subscription(String, "/drone/safety_status", self._safety_cb, 10)
        self.create_subscription(State, "/mavros/state", self._state_cb, 10)
        self.create_subscription(String, "/gesture", self._gesture_cb, 10)

        # 发布
        self.diag_pub = self.create_publisher(String, "/drone/diagnostics", 10)

        # 服务: 手动查询诊断
        self.srv = self.create_service(
            Trigger, "/drone/get_diagnostics", self._handle_query)

        # 定时发布 0.5Hz
        self.timer = self.create_timer(2.0, self._publish_diagnostics)
        self.get_logger().info("Diagnostics node started")

    def _safety_cb(self, msg):
        try:
            self.safety_status = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _state_cb(self, msg):
        self.fcu_state = {
            "connected": msg.connected,
            "armed": msg.armed,
            "mode": msg.mode,
        }

    def _gesture_cb(self, msg):
        self.gesture_count += 1
        now = time.time()
        self._gesture_window.append(now)
        # 保留最近10秒的帧用于计算FPS
        self._gesture_window = [t for t in self._gesture_window if now - t < 10.0]
        if len(self._gesture_window) > 1:
            dt = self._gesture_window[-1] - self._gesture_window[0]
            self.gesture_fps = len(self._gesture_window) / dt if dt > 0 else 0
        self.last_gesture_time = now

    def _collect(self):
        now = time.time()
        cpu = psutil.cpu_percent(interval=None)
        mem = psutil.virtual_memory()
        return {
            "timestamp": now,
            "uptime_sec": round(now - self.start_time, 1),
            "system": {
                "cpu_percent": cpu,
                "memory_percent": mem.percent,
                "memory_used_mb": round(mem.used / 1024 / 1024),
            },
            "gesture": {
                "total_frames": self.gesture_count,
                "fps": round(self.gesture_fps, 1),
                "last_gesture_age": round(now - self.last_gesture_time, 1)
                    if self.last_gesture_time > 0 else -1,
            },
            "fcu": self.fcu_state,
            "safety": self.safety_status,
        }

    def _publish_diagnostics(self):
        msg = String()
        msg.data = json.dumps(self._collect(), indent=2)
        self.diag_pub.publish(msg)

    def _handle_query(self, request, response):
        response.success = True
        response.message = json.dumps(self._collect(), indent=2)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
