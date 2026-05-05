"""
独立手势识别测试脚本 (不需要 ROS2)
用于快速验证 MediaPipe 手势识别效果

用法: python3 test_gesture_standalone.py [视频源]
  python3 test_gesture_standalone.py          # 用摄像头
  python3 test_gesture_standalone.py 0        # 用摄像头 (显式指定)
  python3 test_gesture_standalone.py test.mp4 # 用视频文件
  python3 test_gesture_standalone.py rtsp://192.168.1.10:554/stream  # RTSP流
"""

import sys
import cv2
import mediapipe as mp
from gesture_definitions import classify_gesture, GESTURE_NAMES, GestureID


def main():
    # 视频源
    source = sys.argv[1] if len(sys.argv) > 1 else '0'
    if source.isdigit():
        source = int(source)

    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        print(f'错误: 无法打开视频源 {source}')
        return

    print(f'视频源: {source}')
    print('按 q 退出, 按 s 截图')
    print()

    mp_hands = mp.solutions.hands
    mp_drawing = mp.solutions.drawing_utils
    mp_styles = mp.solutions.drawing_styles

    hands = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=1,
        model_complexity=1,
        min_detection_confidence=0.7,
        min_tracking_confidence=0.5,
    )

    current_gesture = GestureID.NONE
    gesture_count = 0
    frame_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print('视频结束或读取失败')
            break

        frame_count += 1
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(rgb)

        gesture_id = GestureID.NONE

        if results.multi_hand_landmarks:
            hand_lm = results.multi_hand_landmarks[0]
            gesture_id = classify_gesture(hand_lm.landmark)

            # 绘制手部关键点
            mp_drawing.draw_landmarks(
                frame, hand_lm, mp_hands.HAND_CONNECTIONS,
                mp_styles.get_default_hand_landmarks_style(),
                mp_styles.get_default_hand_connections_style(),
            )

        # 防抖
        if gesture_id == current_gesture:
            gesture_count += 1
        else:
            current_gesture = gesture_id
            gesture_count = 1

        # 显示手势名称
        if gesture_count >= 3 and gesture_id != GestureID.NONE:
            name = GESTURE_NAMES[gesture_id]
            cv2.putText(frame, f'Gesture: {name}', (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
            if gesture_count == 3:
                print(f'[Frame {frame_count}] 手势: {name}')
        else:
            cv2.putText(frame, 'No gesture', (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 2)

        # 帧计数
        cv2.putText(frame, f'Frame: {frame_count}', (10, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        cv2.imshow('Gesture Test (q=quit, s=screenshot)', frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('s'):
            filename = f'gesture_capture_{frame_count}.jpg'
            cv2.imwrite(filename, frame)
            print(f'截图已保存: {filename}')

    cap.release()
    cv2.destroyAllWindows()
    hands.close()
    print('结束')


if __name__ == '__main__':
    main()
