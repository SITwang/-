import cv2
import mediapipe as mp
import math
import time
# 初始化 MediaPipe Pose 模型
mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils


# 角度计算函数
def calculate_angle(a, b, c):
    # 计算三点之间的角度
    a = [a.x, a.y]
    b = [b.x, b.y]
    c = [c.x, c.y]
    radians = math.atan2(c[1] - b[1], c[0] - b[0]) - math.atan2(a[1] - b[1], a[0] - b[0])
    angle = abs(radians) * 180.0 / math.pi
    return angle


# 检测引体向上
def detect_pullup(angle):
    if angle > 160:  # 当肘部几乎完全伸直时
        return 'up'
    elif angle < 90:  # 当肘部弯曲到一定程度时
        return 'down'
    return None


# 初始化视频捕捉
cap = cv2.VideoCapture(0)

# 初始化 MediaPipe Pose 模型
with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
    pullup_count = 0
    previous_state = None  # 用来记录动作状态
    start_time = time.time()

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # 转换为 RGB 格式
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = pose.process(frame_rgb)

        # 如果检测到姿势
        if results.pose_landmarks:
            # 提取肩膀、肘部和手腕的关键点
            landmarks = results.pose_landmarks.landmark
            shoulder = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER]
            elbow = landmarks[mp_pose.PoseLandmark.LEFT_ELBOW]
            wrist = landmarks[mp_pose.PoseLandmark.LEFT_WRIST]

            # 计算肘部的角度
            angle = calculate_angle(shoulder, elbow, wrist)

            # 检测引体向上的动作
            state = detect_pullup(angle)

            # 计数逻辑
            if state == 'down' and previous_state != 'down':
                pullup_count += 1  # 增加引体向上计数
                print(f'Pull-up count: {pullup_count}')
            previous_state = state

            # 绘制关键点和连接线
            mp_drawing.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

        # 显示视频帧
        cv2.imshow("Pull-up Detection", frame)

        # 按 'q' 键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
