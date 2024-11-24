import cv2
import numpy as np
import mediapipe as mp

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_holistic = mp.solutions.holistic

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from ocs2_msgs.msg import mpc_target_trajectories
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray
from kuavo_msgs.srv import changeArmCtrlMode
import numpy as np
import argparse
import sys
import select
import termios
import tty

from motion_capture_ik.msg import twoArmHandPose

class RobotControl:
    def __init__(self):
        self.robot_height = 0.00
        self.height_speed = 0.04

        rospy.init_node("test")
        self.joint_pub = rospy.Publisher("/kuavo_arm_traj", JointState, queue_size=10)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.step_pub = rospy.Publisher("/humanoid_mpc_stop_step_num", Int32, queue_size=10)

        rospy.Subscriber("/kuavo_arm_traj", JointState, self.arm_traj_callback)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        rospy.Subscriber("/humanoid_mpc_stop_step_num", Int32, self.stop_step_callback)

    def arm_pose_publish(self, left_arm_traj, right_arm_traj):
        msg = JointState()
        msg.name = ["arm_joint_" + str(i) for i in range(1, 15)]
        msg.header.stamp = rospy.Time.now()
        msg.position = np.array(left_arm_traj + right_arm_traj)
        self.joint_pub.publish(msg)
        print("======= Published to /kuavo_arm_traj")

    def publish_robot_vel(self, velocities):
        robot_vel = Twist()
        robot_vel.linear.x = velocities[0]
        robot_vel.linear.y = velocities[1]
        robot_vel.linear.z = velocities[3]
        robot_vel.angular.z = velocities[2]
        self.vel_pub.publish(robot_vel)
        print("======= Published to /cmd_vel")
    
    def arm_traj_callback(self, msg):
        a=1
        #print(f"======= Received from /kuavo_arm_traj:\n{msg}")

    def cmd_vel_callback(self, msg):
        a=1
        #print(f"======= Received from /cmd_vel\n{msg}")

    def stop_step_callback(self, msg):
        a=1
        #print(f"======= Received from /humanoid_mpc_stop_step_num\n{msg}")

    def mpc_target_callback(self, msg):
        a=1
        #print(f"======= Received from /humanoid_mpc_target_pose\n{msg}")
rc=RobotControl()

#rospy.init_node("sim_ik_cmd", anonymous=True)
#pub = rospy.Publisher('/ik/two_arm_hand_pose_cmd', twoArmHandPose, queue_size=10)  # 发布自定义的消息类
#record_data = np.load("/home/kuavo/David/data/data_pose.npy", allow_pickle=True)
    
#print(f"data size: {len(record_data)}")

#rate = rospy.Rate(100)  # 1/5=0.2s maximum value
#idx = 0
#forward_direction = True

# 创建视频捕捉对象

wrist_positions = []

cap = cv2.VideoCapture(4)

try:
    with mp_holistic.Holistic(
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5) as holistic:
        while cap.isOpened():
            success, image = cap.read()
            if not success:
                print("Ignoring empty camera frame.")
            # If loading a video, use 'break' instead of 'continue'.
                continue

            # To improve performance, optionally mark the image as not writeable to
            # pass by reference.
            image.flags.writeable = False
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = holistic.process(image)

            # Draw landmark annotation on the image.
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            mp_drawing.draw_landmarks(
                image,
                results.face_landmarks,
                mp_holistic.FACEMESH_CONTOURS,
                landmark_drawing_spec=None,
                connection_drawing_spec=mp_drawing_styles
                .get_default_face_mesh_contours_style())
            mp_drawing.draw_landmarks(
                image,
                results.pose_landmarks,
                mp_holistic.POSE_CONNECTIONS,
                landmark_drawing_spec=mp_drawing_styles
                .get_default_pose_landmarks_style())
            # Flip the image horizontally for a selfie-view display.

            if results.pose_landmarks:
                for landmark in results.pose_landmarks.landmark:
                # ??landmark?x, y, z???????visibility??????
                    #print(dir(results.pose_landmarks))
                    #print(results.pose_landmarks)
                    pass
                
                nose_position=results.pose_landmarks.landmark[0]
                left_feet_position=results.pose_landmarks.landmark[28]
                right_feet_position=results.pose_landmarks.landmark[27]
                middle_feet_position = np.array([
            (left_feet_position.x + right_feet_position.x) / 2,
            (left_feet_position.y + right_feet_position.y) / 2,
            (left_feet_position.z + right_feet_position.z) / 2
        ])
                # 计算鼻子和脚中间之间的欧氏距离
                height = np.linalg.norm(np.array([nose_position.x, nose_position.y, nose_position.z]) - middle_feet_position)

                print("身高（鼻子到中间脚的距离）:", height)

                #身高为两点之间的欧氏距离
                #height=nose_position-middle_feet_position
                #print(height)

                left_hip_position=results.pose_landmarks.landmark[23]
                right_hip_position=results.pose_landmarks.landmark[24]
                middle_hip_position = np.array([
            (left_hip_position.x + right_hip_position.x) / 2,
            (left_hip_position.y + right_hip_position.y) / 2,
            (left_hip_position.z + right_hip_position.z) / 2
        ])
                # 计算臀中间和脚中间之间的欧氏距离
                hip_height = np.linalg.norm(middle_hip_position - middle_feet_position)

                print("臀部高度（臀中间到脚中间的距离）:", hip_height)

                #计算臀部高度占身高的比例
                hip_scale = hip_height / height
                print("臀部高度占身高的比例:", hip_scale)

                height=hip_height-0.4

                height = max(-0.2, min(height, 0))
                rc.publish_robot_vel([0, 0, 0, height])

                #比例范围范围：0.2~0.4
                #上下的范围：  -0.2~0.00

                left_shoulder_position=results.pose_landmarks.landmark[11]
                right_shoulder_position=results.pose_landmarks.landmark[12]
                #原点在肩膀中间
                middle_shoulder_position = {
            'x': (left_shoulder_position.x + right_shoulder_position.x) / 2,
            'y': (left_shoulder_position.y + right_shoulder_position.y) / 2,
            'z': (left_shoulder_position.z + right_shoulder_position.z) / 2
        }
                # 假设 origin 已经定义为中间肩部位置
                origin = middle_shoulder_position

                # 获取左手腕的位置
                left_wrist_position = results.pose_landmarks.landmark[15]

                # 计算左手腕的真实位置
                left_wrist_real_position = {
            'x': left_wrist_position.x - middle_shoulder_position['x'],
            'y': left_wrist_position.y - middle_shoulder_position['y'],
            'z': left_wrist_position.z - middle_shoulder_position['z']
        }

                right_wrist_position = results.pose_landmarks.landmark[16]
                # 计算右手腕的真实位置
                right_wrist_real_position = {
            'x': right_wrist_position.x - middle_shoulder_position['x'],
            'y': right_wrist_position.y - middle_shoulder_position['y'],
            'z': right_wrist_position.z - middle_shoulder_position['z']
        }

                print("左手腕的真实位置:", left_wrist_real_position)
                print("右手腕的真实位置:", right_wrist_real_position)

                wrist_positions.append({
                    'left_wrist': left_wrist_real_position,
                    'right_wrist': right_wrist_real_position
                })

    #        eef_pose_msg = twoArmHandPose()

            # 填充左臂姿态数据
            #eef_pose_msg.left_pose.pos_xyz = record_data[idx]['left_pose']['pos_xyz']
            #eef_pose_msg.left_pose.quat_xyzw = record_data[idx]['left_pose']['rot_xyzw']
            #eef_pose_msg.left_pose.elbow_pos_xyz = record_data[idx]['left_pose']['elbow']
            #eef_pose_msg.left_pose.joint_angles = record_data[idx]['left_pose']['joint_angles']

            # 填充右臂姿态数据
            #eef_pose_msg.right_pose.pos_xyz = record_data[idx]['right_pose']['pos_xyz']
            #eef_pose_msg.right_pose.quat_xyzw = record_data[idx]['right_pose']['rot_xyzw']
            #eef_pose_msg.right_pose.elbow_pos_xyz = record_data[idx]['right_pose']['elbow']
            #eef_pose_msg.right_pose.joint_angles = record_data[idx]['right_pose']['joint_angles']

            # 打印消息
            #print(f"eef_pose_msg[{idx}]:\n {eef_pose_msg}")

            # 发布消息
            #pub.publish(eef_pose_msg)

            #rate.sleep()
            #idx = idx + 1 if forward_direction else idx - 1
            #if idx == len(record_data) - 1:
            #    forward_direction = False
            #elif idx == 0:
            #    forward_direction = True
            
            #fps = cap.get(cv2.CAP_PROP_FPS)

            #np.save('/home/kuavo/David/data/wrist_positions.npy', wrist_positions)

            #print(f"Frames per second: {fps}")



            # 显示结果
            cv2.imshow('Mediapipe Pose', image)
            if cv2.waitKey(5) & 0xFF == 27:
                break

#pose.close()

except KeyboardInterrupt:
    print("\n程序被手动终止。")

finally:
    np.save('/home/kuavo/David/data/wrist_positions.npy', wrist_positions)

    cap.release()
