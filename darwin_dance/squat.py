# #!/usr/bin/env python
import rospy
from darwin_gazebo.darwin import Darwin
import time
from std_msgs.msg import Float64


def move_joint(joint_topic, position, duration=1.5):
    pub = rospy.Publisher(joint_topic, Float64, queue_size=1)
    start_time = time.time()
    while time.time() - start_time < duration:
        pub.publish(position)
 

if __name__=="__main__":
    rospy.init_node("walker_demo")
    
    darwin = Darwin()
    rospy.loginfo("Instantiating Darwin Client")

    rospy.sleep(1)
    rospy.loginfo("Darwin Walker Demo Starting")

    darwin.set_walk_velocity(0.2,0,0)
    rospy.sleep(3)

    darwin.set_walk_velocity(0,0,0)

    move_joint('/darwin/j_gripper_l_position_controller/command', -1.0)
    move_joint('/darwin/j_gripper_r_position_controller/command', -1.0)

    # Balancar na frente
    move_joint('/darwin/j_shoulder_l_position_controller/command', -0.2)
    move_joint('/darwin/j_shoulder_r_position_controller/command', 0.2)
    move_joint('/darwin/j_high_arm_l_position_controller/command', 1.5)
    move_joint('/darwin/j_high_arm_r_position_controller/command', 1.5)
    move_joint('/darwin/j_low_arm_l_position_controller/command', -1.3)
    move_joint('/darwin/j_low_arm_r_position_controller/command', 1.3)
    move_joint('/darwin/j_wrist_l_position_controller/command', -1.0)
    move_joint('/darwin/j_wrist_r_position_controller/command', -1.0)

    rospy.sleep(1)
    move_joint('/darwin/j_high_arm_l_position_controller/command', 0.5)
    move_joint('/darwin/j_high_arm_r_position_controller/command', 1.5)

    move_joint('/darwin/j_high_arm_l_position_controller/command', 1.5)
    move_joint('/darwin/j_high_arm_r_position_controller/command', 0.5)

    move_joint('/darwin/j_high_arm_l_position_controller/command', 0.5)
    move_joint('/darwin/j_high_arm_r_position_controller/command', 1.5)

    move_joint('/darwin/j_high_arm_l_position_controller/command', 1.5)
    move_joint('/darwin/j_high_arm_r_position_controller/command', 0.5)









    # Balancar vai e vem
    move_joint('/darwin/j_shoulder_l_position_controller/command', -1.7)
    move_joint('/darwin/j_shoulder_r_position_controller/command', 1.7)
    move_joint('/darwin/j_high_arm_l_position_controller/command', 1.5)
    move_joint('/darwin/j_high_arm_r_position_controller/command', 1.5)
    move_joint('/darwin/j_low_arm_l_position_controller/command', 0)
    move_joint('/darwin/j_low_arm_r_position_controller/command', 0)
    move_joint('/darwin/j_wrist_l_position_controller/command', -1.0)
    move_joint('/darwin/j_wrist_r_position_controller/command', -1.0)

    rospy.sleep(1)
    move_joint('/darwin/j_wrist_l_position_controller/command', 0.0)
    move_joint('/darwin/j_wrist_r_position_controller/command', 0.0)

    move_joint('/darwin/j_wrist_l_position_controller/command', -1.0)
    move_joint('/darwin/j_wrist_r_position_controller/command', -1.0)

    move_joint('/darwin/j_wrist_l_position_controller/command', 0.0)
    move_joint('/darwin/j_wrist_r_position_controller/command', 0.0)









    # Balancar pra frente
    move_joint('/darwin/j_shoulder_l_position_controller/command', -0.9)
    move_joint('/darwin/j_shoulder_r_position_controller/command', 0.9)
    move_joint('/darwin/j_high_arm_l_position_controller/command', 1.6)
    move_joint('/darwin/j_high_arm_r_position_controller/command', 1.6)
    move_joint('/darwin/j_low_arm_l_position_controller/command', 0.0)
    move_joint('/darwin/j_low_arm_r_position_controller/command', 0.0)
    move_joint('/darwin/j_wrist_l_position_controller/command', 0.0)
    move_joint('/darwin/j_wrist_r_position_controller/command', 0.0)


    rospy.sleep(1)
    move_joint("/darwin/j_shoulder_l_position_controller/command", -0.5)
    move_joint("/darwin/j_shoulder_r_position_controller/command", 2.0)

    move_joint("/darwin/j_shoulder_l_position_controller/command", -2.0)
    move_joint("/darwin/j_shoulder_r_position_controller/command", 0.5)












    # Bater no bumbum
    move_joint('/darwin/j_shoulder_l_position_controller/command', 1.1)
    move_joint('/darwin/j_shoulder_r_position_controller/command', -1.1)
    move_joint('/darwin/j_high_arm_l_position_controller/command', 1.3)
    move_joint('/darwin/j_high_arm_r_position_controller/command', 1.3)
    move_joint('/darwin/j_low_arm_l_position_controller/command', -0.6)
    move_joint('/darwin/j_low_arm_r_position_controller/command', 0.6)
    move_joint('/darwin/j_wrist_l_position_controller/command', -1.3)
    move_joint('/darwin/j_wrist_r_position_controller/command', -1.3)
    move_joint('/darwin/j_gripper_l_position_controller/command', -1.0)
    move_joint('/darwin/j_gripper_r_position_controller/command', -1.0)

    rospy.sleep(1)
    move_joint('/darwin/j_shoulder_l_position_controller/command', 0.8, duration=1.5)
    move_joint('/darwin/j_shoulder_r_position_controller/command', -0.8, duration=1.5)
    move_joint('/darwin/j_gripper_l_position_controller/command', 0.0, duration=1.5)
    move_joint('/darwin/j_gripper_r_position_controller/command', 0.0, duration=1.5)
    move_joint('/darwin/j_gripper_l_position_controller/command', -1.0, duration=1.5)
    move_joint('/darwin/j_gripper_r_position_controller/command', -1.0, duration=1.5)
    move_joint('/darwin/j_gripper_l_position_controller/command', 0.0, duration=1.5)
    move_joint('/darwin/j_gripper_r_position_controller/command', 0.0, duration=1.5)

    move_joint('/darwin/j_shoulder_l_position_controller/command', 1.1, duration=1.5)
    move_joint('/darwin/j_shoulder_r_position_controller/command', -1.1, duration=1.5)
    move_joint('/darwin/j_gripper_l_position_controller/command', 0.0, duration=1.5)
    move_joint('/darwin/j_gripper_r_position_controller/command', 0.0, duration=1.5)
    move_joint('/darwin/j_gripper_l_position_controller/command', -1.0, duration=1.5)
    move_joint('/darwin/j_gripper_r_position_controller/command', -1.0, duration=1.5)
    move_joint('/darwin/j_gripper_l_position_controller/command', 0.0, duration=1.5)
    move_joint('/darwin/j_gripper_r_position_controller/command', 0.0, duration=1.5)

    
    
    rospy.loginfo("Darwin Walker Demo Finished")