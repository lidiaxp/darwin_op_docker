# #!/usr/bin/env python
import rospy
from darwin_gazebo.darwin import Darwin
import time
from std_msgs.msg import Float64


class DarwinMotion:
    def __init__(self):
        rospy.init_node('darwin_motion', anonymous=True)
        self.darwin = Darwin()
        self.start_time = time.time()

        self.steps = {
            0: 'darwing_walking',
            1: 'setting',
            2: 'start_position',
            
            3: 'prepare_first_swing',
            4: 'shake_1_first_swing',
            5: 'shake_2_first_swing',
            6: 'shake_3_first_swing',
            7: 'shake_4_first_swing',
            8: 'shake_5_first_swing',
            9: 'shake_6_first_swing',
            10: 'shake_7_first_swing',
            11: 'shake_8_first_swing',

            12: 'prepare_second_swing',
            13: 'shake_1_second_swing',
            14: 'shake_2_second_swing',
            15: 'shake_3_second_swing',
            16: 'shake_4_second_swing',
            17: 'shake_5_second_swing',
            18: 'shake_6_second_swing',
            19: 'shake_7_second_swing',
            20: 'shake_8_second_swing',

            21: 'prepare_third_swing',
            22: 'shake_1_third_swing',
            23: 'shake_2_third_swing',
            24: 'shake_3_third_swing',
            25: 'shake_4_third_swing',
            26: 'shake_5_third_swing',
            27: 'shake_6_third_swing',
            28: 'shake_7_third_swing',
            29: 'shake_8_third_swing',

            30: 'prepare_fourth_swing',
            31: 'shake_1_fourth_swing',
            32: 'shake_2_fourth_swing',
            33: 'shake_3_fourth_swing',
            34: 'shake_4_fourth_swing',
            35: 'shake_5_fourth_swing',
            36: 'shake_6_fourth_swing',
            37: 'shake_7_fourth_swing',
            38: 'shake_8_fourth_swing',
            39: 'end'

            # 19: 'shake_1_1_fourth_swing',
            # # 20: 'shake_1_2_fourth_swing',
            # # 21: 'shake_1_3_fourth_swing',
            # 22: 'shake_2_1_fourth_swing',
            # # 23: 'shake_2_2_fourth_swing',
            # # 24: 'shake_2_3_fourth_swing',
            # 25: 'shake_3_1_fourth_swing',
            # # 26: 'shake_3_2_fourth_swing',
            # # 27: 'shake_3_3_fourth_swing',
            # 28: 'shake_4_1_fourth_swing',
            # # 29: 'shake_4_2_fourth_swing',
            # # 30: 'shake_4_3_fourth_swing',
            # 31: 'end'
        }

        callback_time = 1.5
        rospy.Timer(rospy.Duration(callback_time), self.tick)

        rospy.Timer(rospy.Duration(callback_time), self.shoulder_r)
        rospy.Timer(rospy.Duration(callback_time), self.high_arm_r)
        rospy.Timer(rospy.Duration(callback_time), self.low_arm_r)
        rospy.Timer(rospy.Duration(callback_time), self.wrist_arm_r)
        rospy.Timer(rospy.Duration(callback_time), self.gripper_arm_r)

        rospy.Timer(rospy.Duration(callback_time), self.shoulder_l)
        rospy.Timer(rospy.Duration(callback_time), self.high_arm_l)
        rospy.Timer(rospy.Duration(callback_time), self.low_arm_l)
        rospy.Timer(rospy.Duration(callback_time), self.wrist_arm_l)
        rospy.Timer(rospy.Duration(callback_time), self.gripper_arm_l)

        self.cur_step = 0
        rospy.sleep(1)
        self.darwin.set_walk_velocity(0.2,0,0)
        rospy.sleep(5)
        self.darwin.set_walk_velocity(0,0,0)

        self.cur_step = 1


    def move_joint(self, joint_topic, position, duration=1.5):
        pub = rospy.Publisher(joint_topic, Float64, queue_size=1)
        start_time = time.time()
        while time.time() - start_time < duration:
            pub.publish(position)
        

    def tick(self, data):
        if self.cur_step > 0:
            rospy.sleep(0.5)
            self.cur_step += 1

        if self.steps[self.cur_step] == 'end':
            exit()


    ########## RIGHT SIDE ##########
    def shoulder_r(self, data):
        _topic = '/darwin/j_shoulder_r_position_controller/command'
        if self.steps[self.cur_step] == 'prepare_first_swing':
            self.move_joint(_topic, 0.2)

        if self.steps[self.cur_step] == 'prepare_second_swing':
            self.move_joint(_topic, 1.7)

        if self.steps[self.cur_step] == 'prepare_third_swing':
            self.move_joint(_topic, 0.9)

        if self.steps[self.cur_step] == 'shake_1_third_swing':
            self.move_joint(_topic, 2.0)

        if self.steps[self.cur_step] == 'shake_2_third_swing':
            self.move_joint(_topic, 0.5)

        if self.steps[self.cur_step] == 'shake_3_third_swing':
            self.move_joint(_topic, 2.0)

        if self.steps[self.cur_step] == 'shake_4_third_swing':
            self.move_joint(_topic, 0.5)

        if self.steps[self.cur_step] == 'shake_5_third_swing':
            self.move_joint(_topic, 2.0)

        if self.steps[self.cur_step] == 'shake_6_third_swing':
            self.move_joint(_topic, 0.5)

        if self.steps[self.cur_step] == 'shake_7_third_swing':
            self.move_joint(_topic, 2.0)

        if self.steps[self.cur_step] == 'shake_8_third_swing':
            self.move_joint(_topic, 0.5)

        if self.steps[self.cur_step] == 'prepare_fourth_swing':
            self.move_joint(_topic, -1.1)

        if self.steps[self.cur_step] == 'prepare_fourth_swing':
            self.move_joint(_topic, -1.1)

        if self.steps[self.cur_step] == 'shake_1_fourth_swing':
            self.move_joint(_topic, -0.8)

        if self.steps[self.cur_step] == 'shake_2_fourth_swing':
            self.move_joint(_topic, -1.1)

        if self.steps[self.cur_step] == 'shake_3_fourth_swing':
            self.move_joint(_topic, -0.8)

        if self.steps[self.cur_step] == 'shake_4_fourth_swing':
            self.move_joint(_topic, -1.1)

        if self.steps[self.cur_step] == 'shake_5_fourth_swing':
            self.move_joint(_topic, -0.8)

        if self.steps[self.cur_step] == 'shake_6_fourth_swing':
            self.move_joint(_topic, -1.1)

        if self.steps[self.cur_step] == 'shake_7_fourth_swing':
            self.move_joint(_topic, -0.8)

        if self.steps[self.cur_step] == 'shake_8_fourth_swing':
            self.move_joint(_topic, -1.1)


    def high_arm_r(self, data):
        _topic = '/darwin/j_high_arm_r_position_controller/command'
        if self.steps[self.cur_step] == 'prepare_first_swing':
            self.move_joint(_topic, 1.5)

        if self.steps[self.cur_step] == 'shake_1_first_swing':
            self.move_joint(_topic, 1.5)

        if self.steps[self.cur_step] == 'shake_2_first_swing':
            self.move_joint(_topic, 0.5)

        if self.steps[self.cur_step] == 'shake_3_first_swing':
            self.move_joint(_topic, 1.5)

        if self.steps[self.cur_step] == 'shake_4_first_swing':
            self.move_joint(_topic, 0.5)

        if self.steps[self.cur_step] == 'shake_5_first_swing':
            self.move_joint(_topic, 1.5)

        if self.steps[self.cur_step] == 'shake_6_first_swing':
            self.move_joint(_topic, 0.5)

        if self.steps[self.cur_step] == 'shake_7_first_swing':
            self.move_joint(_topic, 1.5)

        if self.steps[self.cur_step] == 'shake_8_first_swing':
            self.move_joint(_topic, 0.5)

        if self.steps[self.cur_step] == 'prepare_second_swing':
            self.move_joint(_topic, 1.5)

        if self.steps[self.cur_step] == 'prepare_third_swing':
            self.move_joint(_topic, 1.6)
            

    def low_arm_r(self, data):
        _topic = '/darwin/j_low_arm_r_position_controller/command'
        if self.steps[self.cur_step] == 'prepare_first_swing':
            self.move_joint(_topic, 1.3)

        if self.steps[self.cur_step] == 'prepare_second_swing':
            self.move_joint(_topic, 0.0)

        if self.steps[self.cur_step] == 'prepare_third_swing':
            self.move_joint(_topic, 0.0)

        if self.steps[self.cur_step] == 'prepare_fourth_swing':
            self.move_joint(_topic, 0.6)


    def wrist_arm_r(self, data):
        _topic = '/darwin/j_wrist_r_position_controller/command'
        if self.steps[self.cur_step] == 'prepare_first_swing':
            self.move_joint(_topic, -1.0)

        if self.steps[self.cur_step] == 'prepare_second_swing':
            self.move_joint(_topic, -1.0)

        if self.steps[self.cur_step] == 'shake_1_second_swing':
            self.move_joint(_topic, -1.0)

        if self.steps[self.cur_step] == 'shake_2_second_swing':
            self.move_joint(_topic, 0.0)

        if self.steps[self.cur_step] == 'shake_3_second_swing':
            self.move_joint(_topic, -1.0)

        if self.steps[self.cur_step] == 'shake_4_second_swing':
            self.move_joint(_topic, 0.0)

        if self.steps[self.cur_step] == 'shake_5_second_swing':
            self.move_joint(_topic, -1.0)

        if self.steps[self.cur_step] == 'shake_6_second_swing':
            self.move_joint(_topic, 0.0)

        if self.steps[self.cur_step] == 'shake_7_second_swing':
            self.move_joint(_topic, -1.0)

        if self.steps[self.cur_step] == 'shake_8_second_swing':
            self.move_joint(_topic, 0.0)

        if self.steps[self.cur_step] == 'prepare_third_swing':
            self.move_joint(_topic, 0.0)

        if self.steps[self.cur_step] == 'prepare_fourth_swing':
            self.move_joint(_topic, -1.3)


    def gripper_arm_r(self, data):
        _topic = '/darwin/j_gripper_l_position_controller/command'
        if self.steps[self.cur_step] == 'start_position':
            self.move_joint(_topic, -1.0)

        if self.steps[self.cur_step] == 'prepare_fourth_swing':
            self.move_joint(_topic, -1.0)

        if self.steps[self.cur_step] in [
            'shake_1_fourth_swing', 'shake_2_fourth_swing', 'shake_3_fourth_swing', 'shake_4_fourth_swing'
            'shake_5_fourth_swing', 'shake_6_fourth_swing', 'shake_7_fourth_swing', 'shake_8_fourth_swing'
            ]:
            self.move_joint(_topic, -1.0, duration=0.5)
            self.move_joint(_topic, 0.0, duration=0.5)
            self.move_joint(_topic, -1.0, duration=0.5)


    ########## LEFT SIDE ##########
    def shoulder_l(self, data):
        _topic = '/darwin/j_shoulder_l_position_controller/command'
        if self.steps[self.cur_step] == 'prepare_first_swing':
            self.move_joint(_topic, -0.2)

        if self.steps[self.cur_step] == 'prepare_second_swing':
            self.move_joint(_topic, -1.7)

        if self.steps[self.cur_step] == 'prepare_third_swing':
            self.move_joint(_topic, -0.9)

        if self.steps[self.cur_step] == 'shake_1_third_swing':
            self.move_joint(_topic, -0.5)

        if self.steps[self.cur_step] == 'shake_2_third_swing':
            self.move_joint(_topic, -2.0)

        if self.steps[self.cur_step] == 'shake_3_third_swing':
            self.move_joint(_topic, -0.5)

        if self.steps[self.cur_step] == 'shake_4_third_swing':
            self.move_joint(_topic, -2.0)

        if self.steps[self.cur_step] == 'shake_5_third_swing':
            self.move_joint(_topic, -0.5)

        if self.steps[self.cur_step] == 'shake_6_third_swing':
            self.move_joint(_topic, -2.0)

        if self.steps[self.cur_step] == 'shake_7_third_swing':
            self.move_joint(_topic, -0.5)

        if self.steps[self.cur_step] == 'shake_8_third_swing':
            self.move_joint(_topic, -2.0)

        if self.steps[self.cur_step] == 'prepare_fourth_swing':
            self.move_joint(_topic, 1.1)

        if self.steps[self.cur_step] == 'shake_1_fourth_swing':
            self.move_joint(_topic, 0.8)

        if self.steps[self.cur_step] == 'shake_2_fourth_swing':
            self.move_joint(_topic, 1.1)

        if self.steps[self.cur_step] == 'shake_3_fourth_swing':
            self.move_joint(_topic, 0.8)

        if self.steps[self.cur_step] == 'shake_4_fourth_swing':
            self.move_joint(_topic, 1.1)

        if self.steps[self.cur_step] == 'shake_5_fourth_swing':
            self.move_joint(_topic, 0.8)

        if self.steps[self.cur_step] == 'shake_6_fourth_swing':
            self.move_joint(_topic, 1.1)

        if self.steps[self.cur_step] == 'shake_7_fourth_swing':
            self.move_joint(_topic, 0.8)

        if self.steps[self.cur_step] == 'shake_8_fourth_swing':
            self.move_joint(_topic, 1.1)


    def high_arm_l(self, data):
        _topic = '/darwin/j_high_arm_l_position_controller/command'
        if self.steps[self.cur_step] == 'prepare_first_swing':
            self.move_joint(_topic, 1.5)

        if self.steps[self.cur_step] == 'shake_1_first_swing':
            self.move_joint(_topic, 0.5)

        if self.steps[self.cur_step] == 'shake_2_first_swing':
            self.move_joint(_topic, 1.5)

        if self.steps[self.cur_step] == 'shake_3_first_swing':
            self.move_joint(_topic, 0.5)

        if self.steps[self.cur_step] == 'shake_4_first_swing':
            self.move_joint(_topic, 1.5)

        if self.steps[self.cur_step] == 'shake_5_first_swing':
            self.move_joint(_topic, 0.5)

        if self.steps[self.cur_step] == 'shake_6_first_swing':
            self.move_joint(_topic, 1.5)

        if self.steps[self.cur_step] == 'shake_7_first_swing':
            self.move_joint(_topic, 0.5)

        if self.steps[self.cur_step] == 'shake_8_first_swing':
            self.move_joint(_topic, 1.5)

        if self.steps[self.cur_step] == 'prepare_second_swing':
            self.move_joint(_topic, 1.5)

        if self.steps[self.cur_step] == 'prepare_third_swing':
            self.move_joint(_topic, 1.6)

        if self.steps[self.cur_step] == 'prepare_fourth_swing':
            self.move_joint(_topic, 1.3)


    def low_arm_l(self, data):
        _topic = '/darwin/j_low_arm_l_position_controller/command'
        if self.steps[self.cur_step] == 'prepare_first_swing':
            self.move_joint(_topic, -1.3)

        if self.steps[self.cur_step] == 'prepare_second_swing':
            self.move_joint(_topic, 0.0)

        if self.steps[self.cur_step] == 'prepare_third_swing':
            self.move_joint(_topic, 0.0)

        if self.steps[self.cur_step] == 'prepare_fourth_swing':
            self.move_joint(_topic, -0.6)


    def wrist_arm_l(self, data):
        _topic = '/darwin/j_wrist_l_position_controller/command'
        if self.steps[self.cur_step] == 'prepare_first_swing':
            self.move_joint(_topic, -1.0)

        if self.steps[self.cur_step] == 'prepare_second_swing':
            self.move_joint(_topic, -1.0)

        if self.steps[self.cur_step] == 'shake_1_second_swing':
            self.move_joint(_topic, 0.0)

        if self.steps[self.cur_step] == 'shake_2_second_swing':
            self.move_joint(_topic, -1.0)

        if self.steps[self.cur_step] == 'shake_3_second_swing':
            self.move_joint(_topic, 0.0)

        if self.steps[self.cur_step] == 'shake_4_second_swing':
            self.move_joint(_topic, -1.0)

        if self.steps[self.cur_step] == 'shake_5_second_swing':
            self.move_joint(_topic, 0.0)

        if self.steps[self.cur_step] == 'shake_6_second_swing':
            self.move_joint(_topic, -1.0)

        if self.steps[self.cur_step] == 'shake_7_second_swing':
            self.move_joint(_topic, 0.0)

        if self.steps[self.cur_step] == 'shake_8_second_swing':
            self.move_joint(_topic, -1.0)

        if self.steps[self.cur_step] == 'prepare_third_swing':
            self.move_joint(_topic, 0.0)

        if self.steps[self.cur_step] == 'prepare_fourth_swing':
            self.move_joint(_topic, -1.3)


    def gripper_arm_l(self, data):
        _topic = '/darwin/j_gripper_r_position_controller/command'
        if self.steps[self.cur_step] == 'start_position':
            self.move_joint(_topic, -1.0)

        if self.steps[self.cur_step] == 'prepare_fourth_swing':
            self.move_joint(_topic, -1.0)

        if self.steps[self.cur_step] in [
            'shake_1_fourth_swing', 'shake_2_fourth_swing', 'shake_3_fourth_swing', 'shake_4_fourth_swing'
            'shake_5_fourth_swing', 'shake_6_fourth_swing', 'shake_7_fourth_swing', 'shake_8_fourth_swing'
            ]:
            self.move_joint(_topic, -1.0, duration=0.5)
            self.move_joint(_topic, 0.0, duration=0.5)
            self.move_joint(_topic, -1.0, duration=0.5)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        darwinzin = DarwinMotion()
        darwinzin.run()
    except rospy.ROSInterruptException:
        pass
