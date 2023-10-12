#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import time

# List of joint controller topics for arm and hand movements
arm_hand_joint_topics = [
    "/darwin/j_high_arm_l_position_controller/command",
    "/darwin/j_high_arm_r_position_controller/command",
    "/darwin/j_low_arm_l_position_controller/command",
    "/darwin/j_low_arm_r_position_controller/command",
    "/darwin/j_shoulder_l_position_controller/command",
    "/darwin/j_shoulder_r_position_controller/command",
    "/darwin/j_wrist_l_position_controller/command",
    "/darwin/j_wrist_r_position_controller/command",
    "/darwin/j_gripper_l_position_controller/command",
    "/darwin/j_gripper_r_position_controller/command",
]

squat_joint_topics = [
    "/darwin/j_thigh1_l_position_controller/command",
    "/darwin/j_thigh1_r_position_controller/command",
    "/darwin/j_thigh2_l_position_controller/command",
    "/darwin/j_thigh2_r_position_controller/command",
    "/darwin/j_tibia_l_position_controller/command",
    "/darwin/j_tibia_r_position_controller/command",
    "/darwin/j_pelvis_l_position_controller/command",
    "/darwin/j_pelvis_r_position_controller/command"
]

other_joint_topics = [
    "/darwin/j_high_arm_l_position_controller/command",
    "/darwin/j_high_arm_r_position_controller/command",
    "/darwin/j_low_arm_l_position_controller/command",
    "/darwin/j_low_arm_r_position_controller/command",
    "/darwin/j_shoulder_l_position_controller/command",
    "/darwin/j_shoulder_r_position_controller/command",
    "/darwin/j_wrist_l_position_controller/command",
    "/darwin/j_wrist_r_position_controller/command",
    "/darwin/j_gripper_l_position_controller/command",
    "/darwin/j_gripper_r_position_controller/command",
]

def move_joint(joint_topic, position, duration=1.0):
    pub = rospy.Publisher(joint_topic, Float64, queue_size=1)
    start_time = time.time()
    while time.time() - start_time < duration:
        pub.publish(position)

def macarena_dance():
    rospy.init_node('darwin_macarena_dance', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        # Macarena dance moves
        # Left arm up
        move_joint(arm_hand_joint_topics[0], 1.0)
        move_joint(arm_hand_joint_topics[2], -0.5)
        move_joint(arm_hand_joint_topics[4], -1.0)
        move_joint(arm_hand_joint_topics[6], 0.5)
        time.sleep(2.0)  # Hold this pose for 2 seconds

        # Right arm up
        move_joint(arm_hand_joint_topics[1], 1.0)
        move_joint(arm_hand_joint_topics[3], -0.5)
        move_joint(arm_hand_joint_topics[5], 1.0)
        move_joint(arm_hand_joint_topics[7], -0.5)
        time.sleep(2.0)  # Hold this pose for 2 seconds

        # Reset arms to neutral position
        for topic in arm_hand_joint_topics:
            move_joint(topic, 0.0)

        rate.sleep()

if __name__ == '__main__':
    try:
        macarena_dance()
    except rospy.ROSInterruptException:
        pass
