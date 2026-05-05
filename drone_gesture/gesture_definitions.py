"""
手势定义和常量
基于 MediaPipe Hands 21个关键点的手势分类
"""

from enum import IntEnum


class GestureID(IntEnum):
    """手势 ID 枚举"""
    NONE = 0          # 未识别
    OPEN_PALM = 1     # 张开手掌 → 起飞
    FIST = 2          # 握拳 → 降落
    THUMBS_UP = 3     # 竖拇指 → 前进
    OK_SIGN = 4       # OK手势 → 功能键


GESTURE_NAMES = {
    GestureID.NONE: 'none',
    GestureID.OPEN_PALM: 'open_palm',
    GestureID.FIST: 'fist',
    GestureID.THUMBS_UP: 'thumbs_up',
    GestureID.OK_SIGN: 'ok_sign',
}

# MediaPipe Hands 关键点索引
# https://google.github.io/mediapipe/solutions/hands.html
WRIST = 0
THUMB_CMC = 1
THUMB_MCP = 2
THUMB_IP = 3
THUMB_TIP = 4
INDEX_MCP = 5
INDEX_PIP = 6
INDEX_DIP = 7
INDEX_TIP = 8
MIDDLE_MCP = 9
MIDDLE_PIP = 10
MIDDLE_DIP = 11
MIDDLE_TIP = 12
RING_MCP = 13
RING_PIP = 14
RING_DIP = 15
RING_TIP = 16
PINKY_MCP = 17
PINKY_PIP = 18
PINKY_DIP = 19
PINKY_TIP = 20


def is_finger_extended(landmarks, finger_tip, finger_pip, finger_mcp):
    """
    判断手指是否伸直

    使用两种检测方式的组合:
    1. 指尖(TIP) vs 指间关节(PIP) 的 y 坐标比较
    2. TIP-PIP-MCP 三点形成的夹角 (更鲁棒)

    在图像坐标系中, y轴向下为正, 所以伸直时 tip_y < pip_y
    """
    tip = landmarks[finger_tip]
    pip = landmarks[finger_pip]
    mcp = landmarks[finger_mcp]

    # 方法1: y坐标比较 (快速)
    y_extended = tip.y < pip.y

    # 方法2: 向量夹角 (鲁棒)
    # 向量 MCP->PIP 和 PIP->TIP 的夹角
    v1x, v1y = pip.x - mcp.x, pip.y - mcp.y
    v2x, v2y = tip.x - pip.x, tip.y - pip.y
    dot = v1x * v2x + v1y * v2y
    len1 = (v1x**2 + v1y**2)**0.5
    len2 = (v2x**2 + v2y**2)**0.5
    if len1 < 1e-6 or len2 < 1e-6:
        return y_extended
    cos_angle = max(-1.0, min(1.0, dot / (len1 * len2)))
    angle_deg = abs(cos_angle)  # 接近1表示伸直(共线)
    angle_extended = angle_deg > 0.5  # 约60度以内算伸直

    # 两种方法投票: 都说伸直才算伸直, 降低误检
    return y_extended and angle_extended


def is_thumb_extended(landmarks):
    """
    判断拇指是否伸直
    拇指的判断方式不同, 因为它横向运动
    通过拇指尖和拇指IP关节的x距离判断
    """
    # 使用拇指尖和手腕的距离 vs 拇指IP和手腕的距离
    thumb_tip = landmarks[THUMB_TIP]
    thumb_ip = landmarks[THUMB_IP]
    wrist = landmarks[WRIST]

    # 计算到手腕的距离
    tip_dist = ((thumb_tip.x - wrist.x)**2 + (thumb_tip.y - wrist.y)**2)**0.5
    ip_dist = ((thumb_ip.x - wrist.x)**2 + (thumb_ip.y - wrist.y)**2)**0.5

    # 如果拇指尖比IP关节离手腕更远, 说明伸直
    return tip_dist > ip_dist * 1.1


def classify_gesture(landmarks):
    """
    根据手部关键点分类手势

    参数:
        landmarks: MediaPipe Hands 检测到的21个关键点列表

    返回:
        GestureID: 识别到的手势ID
    """
    if landmarks is None or len(landmarks) < 21:
        return GestureID.NONE

    # 检测每根手指是否伸直
    thumb_ext = is_thumb_extended(landmarks)
    index_ext = is_finger_extended(landmarks, INDEX_TIP, INDEX_PIP, INDEX_MCP)
    middle_ext = is_finger_extended(landmarks, MIDDLE_TIP, MIDDLE_PIP, MIDDLE_MCP)
    ring_ext = is_finger_extended(landmarks, RING_TIP, RING_PIP, RING_MCP)
    pinky_ext = is_finger_extended(landmarks, PINKY_TIP, PINKY_PIP, PINKY_MCP)

    all_fingers = [thumb_ext, index_ext, middle_ext, ring_ext, pinky_ext]
    four_fingers = [index_ext, middle_ext, ring_ext, pinky_ext]

    # 张开手掌: 所有手指伸直
    if all(all_fingers):
        return GestureID.OPEN_PALM

    # 握拳: 所有手指弯曲
    if not any(all_fingers):
        return GestureID.FIST

    # 竖拇指: 拇指伸直, 其余弯曲
    if thumb_ext and not any(four_fingers):
        return GestureID.THUMBS_UP

    # OK手势: 拇指和食指形成圆圈, 其余伸直
    # 放宽条件: pinch距离 < 0.08, 其余3指至少2个伸直
    thumb_tip = landmarks[THUMB_TIP]
    index_tip = landmarks[INDEX_TIP]
    pinch_dist = ((thumb_tip.x - index_tip.x)**2 + (thumb_tip.y - index_tip.y)**2)**0.5

    three_fingers = [middle_ext, ring_ext, pinky_ext]
    if pinch_dist < 0.08 and sum(three_fingers) >= 2:
        return GestureID.OK_SIGN

    return GestureID.NONE
