import mediapipe as mp
import cv2

# 创建视频捕捉对象
cap = cv2.VideoCapture(6)

# 初始化mediapipe姿态解决方案
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(static_image_mode=False, min_detection_confidence=0.5, min_tracking_confidence=0.5)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("Ignoring empty camera frame.")
        continue

    # 将图像从BGR转换到RGB
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    image.flags.writeable = False

    # 处理图像，检测姿态
    results = pose.process(image)

    # 在图像上绘制姿态注释
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    mp.solutions.drawing_utils.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

    # 显示结果
    cv2.imshow('Mediapipe Pose', image)
    if cv2.waitKey(5) & 0xFF == 27:
        break

pose.close()
cap.release()