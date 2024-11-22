import cv2
import mediapipe as mp
import time

# MediaPipe 面部网格模型
mp_face_mesh = mp.solutions.face_mesh
mp_drawing = mp.solutions.drawing_utils

# 眼睛纵横比 EAR 计算函数
def eye_aspect_ratio(eye_points, landmarks):
    # 垂直距离
    A = dist(landmarks[1], landmarks[5])
    B = dist(landmarks[2], landmarks[4])
    # 水平距离
    C = dist(landmarks[0], landmarks[3])
    ear = (A + B) / (2.0 * C)
    return ear

# 计算两点之间的欧几里得距离
def dist(p1, p2):
    return ((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2) ** 0.5

# 左右眼的关键点索引
LEFT_EYE = [33, 159, 158, 133, 145, 153]  # MediaPipe 中的左眼关键点索引
RIGHT_EYE = [362, 385, 386, 263, 380, 374]  # MediaPipe 中的右眼关键点索引

# 设定EAR阈值来判断眨眼
EAR_THRESHOLD = 0.30

# 初始化视频捕捉
cap = cv2.VideoCapture(0)

# 初始化 MediaPipe 人脸网格模型
with mp_face_mesh.FaceMesh(min_detection_confidence=0.5, min_tracking_confidence=0.5) as face_mesh:
    blink_count = 0  # 记录眨眼次数
    previous_blink_state = False  # 上一个眨眼状态
    start_time = time.time()  # 记录开始时间

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # 转换为RGB颜色空间，MediaPipe要求RGB格式
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = face_mesh.process(frame_rgb)

        # 如果检测到人脸
        if results.multi_face_landmarks:
            for face_landmarks in results.multi_face_landmarks:
                # 提取左眼和右眼的关键点坐标
                left_eye_points = [face_landmarks.landmark[i] for i in LEFT_EYE]
                right_eye_points = [face_landmarks.landmark[i] for i in RIGHT_EYE]

                left_eye_ear = eye_aspect_ratio(LEFT_EYE, left_eye_points)
                right_eye_ear = eye_aspect_ratio(RIGHT_EYE, right_eye_points)

                # 判断是否眨眼
                if left_eye_ear < EAR_THRESHOLD or right_eye_ear < EAR_THRESHOLD:
                    if not previous_blink_state:
                        blink_count += 1  # 眨眼次数增加
                        previous_blink_state = True
                else:
                    previous_blink_state = False

                # 可视化面部关键点
               #q mp_drawing.draw_landmarks(frame, face_landmarks, mp_face_mesh.FACEMESH_CONTOURS)

        # 显示眨眼次数和时间
        elapsed_time = time.time() - start_time
        if elapsed_time >= 60:  # 1分钟
            print(f"眨眼次数: {blink_count}")
            blink_count = 0  # 重置计数器
            start_time = time.time()  # 重置开始时间

        # 在屏幕上显示眨眼次数
        cv2.putText(frame, f'Blinks: {blink_count}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # 显示结果
        cv2.imshow("Eye Blink Detection", frame)

        # 按 'q' 键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
