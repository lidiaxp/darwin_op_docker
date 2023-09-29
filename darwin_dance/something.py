#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import threading
import time

# List of joint controller topics for squatting
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

# List of joint controller topics for other actions
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

def squat_callback():
    while not rospy.is_shutdown():
        # Perform squatting movement
        for topic in squat_joint_topics:
            move_joint(topic, 0.5)  # Adjust joint positions for squat
        time.sleep(2.0)  # Hold squat pose for 2 seconds

        # Reset squatting pose
        for topic in squat_joint_topics:
            move_joint(topic, 0.0)

def other_actions_callback():
    while not rospy.is_shutdown():
        # Perform other actions
        for topic in other_joint_topics:
            move_joint(topic, 1.0)  # Adjust joint positions for other actions
        time.sleep(3.0)  # Hold other actions pose for 3 seconds

        # Reset to neutral position
        for topic in other_joint_topics:
            move_joint(topic, 0.0)

def main():
    rospy.init_node('darwin_callbacks', anonymous=True)

    # Create threads for squatting and other actions
    squat_thread = threading.Thread(target=squat_callback)
    other_actions_thread = threading.Thread(target=other_actions_callback)

    squat_thread.start()
    other_actions_thread.start()

    squat_thread.join()
    other_actions_thread.join()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
