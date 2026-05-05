"""
手势识别 ROS2 节点
使用 MediaPipe Hands 检测手部关键点并分类手势
"""

import json
import cv2
import numpy as np
import mediapipe as mp

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from drone_gesture.gesture_definitions import (
    classify_gesture, GESTURE_NAMES, GestureID
)


class GestureRecognizerNode(Node):
    def __init__(self):
        super().__init__('gesture_recognizer')

        # 声明参数
        self.declare_parameter('video_source', 0)
        self.declare_parameter('model_complexity', 1)
        self.declare_parameter('min_detection_confidence', 0.7)
        self.declare_parameter('min_tracking_confidence', 0.5)
        self.declare_parameter('publish_annotated_video', True)
        self.declare_parameter('max_num_hands', 1)

        # 获取参数
        self.video_source = self.get_parameter('video_source').value
        model_complexity = self.get_parameter('model_complexity').value
        min_detection = self.get_parameter('min_detection_confidence').value
        min_tracking = self.get_parameter('min_tracking_confidence').value
        self.publish_annotated = self.get_parameter('publish_annotated_video').value
        max_hands = self.get_parameter('max_num_hands').value

        # MediaPipe 初始化
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=max_hands,
            model_complexity=model_complexity,
            min_detection_confidence=min_detection,
            min_tracking_confidence=min_tracking,
        )

        # OpenCV 视频源
        if isinstance(self.video_source, str) and self.video_source.startswith(('rtsp://', 'http://')):
            self.cap = cv2.VideoCapture(self.video_source)
        else:
            self.cap = cv2.VideoCapture(int(self.video_source))

        if not self.cap.isOpened():
            self.get_logger().error(f'无法打开视频源: {self.video_source}')
            return

        self.bridge = CvBridge()

        # QoS 设置
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # 发布者
        self.gesture_pub = self.create_publisher(String, '/gesture', qos)
        if self.publish_annotated:
            self.image_pub = self.create_publisher(Image, '/gesture/image', qos)

        self.current_gesture = GestureID.NONE
        self.gesture_count = 0
        self.frame_count = 0

        # 定时器: 30fps
        self.timer = self.create_timer(1.0 / 30.0, self.process_frame)

        self.get_logger().info(f'手势识别节点已启动, 视频源: {self.video_source}')

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('无法读取视频帧')
            return

        self.frame_count += 1

        # 转换颜色空间 BGR → RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)

        gesture_id = GestureID.NONE
        landmarks_data = None

        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]
            landmarks = hand_landmarks.landmark

            # 分类手势
            gesture_id = classify_gesture(landmarks)

            # 保存关键点数据
            landmarks_data = [
                {'x': lm.x, 'y': lm.y, 'z': lm.z}
                for lm in landmarks
            ]

            # 绘制手部关键点 (用于标注视频)
            if self.publish_annotated:
                self.mp_drawing.draw_landmarks(
                    frame,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style(),
                )

        # 防抖: 连续相同手势才更新
        if gesture_id == self.current_gesture:
            self.gesture_count += 1
        else:
            self.current_gesture = gesture_id
            self.gesture_count = 1

        # 发布手势 (至少连续2帧相同)
        if self.gesture_count >= 2 and gesture_id != GestureID.NONE:
            msg = String()
            payload = {
                'gesture': GESTURE_NAMES[gesture_id],
                'gesture_id': int(gesture_id),
                'confidence': 1.0,
                'frame': self.frame_count,
            }
            if landmarks_data:
                payload['landmarks'] = landmarks_data
            msg.data = json.dumps(payload)
            self.gesture_pub.publish(msg)

            self.get_logger().info(
                f'手势: {GESTURE_NAMES[gesture_id]} (id={gesture_id})',
                throttle_duration_sec=1.0,
            )

        # 发布标注视频
        if self.publish_annotated:
            # 在画面上显示手势名称
            if gesture_id != GestureID.NONE:
                cv2.putText(
                    frame,
                    f'Gesture: {GESTURE_NAMES[gesture_id]}',
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 255, 0),
                    2,
                )

            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                img_msg.header.stamp = self.get_clock().now().to_msg()
                self.image_pub.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f'图像转换失败: {e}')

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        self.hands.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GestureRecognizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
