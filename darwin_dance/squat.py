# #!/usr/bin/env python

# import rospy
# from std_msgs.msg import Float64
# import time

# # List of joint controller topics for leg and pelvis movements
# leg_pelvis_joint_topics = [
#     "/darwin/j_thigh1_l_position_controller/command",
#     "/darwin/j_thigh1_r_position_controller/command",
#     "/darwin/j_thigh2_l_position_controller/command",
#     "/darwin/j_thigh2_r_position_controller/command",
#     "/darwin/j_tibia_l_position_controller/command",
#     "/darwin/j_tibia_r_position_controller/command",
#     "/darwin/j_pelvis_l_position_controller/command",
#     "/darwin/j_pelvis_r_position_controller/command"
# ]

# def move_joint(joint_topic, position, duration=1.0):
#     pub = rospy.Publisher(joint_topic, Float64, queue_size=1)
#     start_time = time.time()
#     while time.time() - start_time < duration:
#         pub.publish(position)

# def squat():
#     rospy.init_node('darwin_squat', anonymous=True)
#     rate = rospy.Rate(1)  # 1 Hz

#     while not rospy.is_shutdown():
#         # Move the leg joints and pelvis to perform a squat
#         for topic in leg_pelvis_joint_topics:
#             move_joint(topic, 0.5)  # Adjust joint positions for squat (you may need to adjust this value)

#         # Hold the squat position for a few seconds
#         time.sleep(2.0)

#         # Reset the leg joints and pelvis to their neutral positions
#         for topic in leg_pelvis_joint_topics:
#             move_joint(topic, 0.0)

#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         squat()
#     except rospy.ROSInterruptException:
#         pass

#!/usr/bin/env python

import rospy
from darwin_gazebo.darwin import Darwin


if __name__=="__main__":
    rospy.init_node("walker_demo")
    
    rospy.loginfo("Instantiating Darwin Client")
    darwin=Darwin()
    rospy.sleep(1)
 
    rospy.loginfo("Darwin Walker Demo Starting")


    darwin.set_walk_velocity(0.2,0,0)
    rospy.sleep(3)
    # darwin.set_walk_velocity(1,0,0)
    # rospy.sleep(3)
    # darwin.set_walk_velocity(0,1,0)
    # rospy.sleep(3)
    # darwin.set_walk_velocity(0,-1,0)
    # rospy.sleep(3)
    # darwin.set_walk_velocity(-1,0,0)
    # rospy.sleep(3)
    # darwin.set_walk_velocity(1,1,0)
    # rospy.sleep(5)
    darwin.set_walk_velocity(0,0,0)
    
    rospy.loginfo("Darwin Walker Demo Finished")