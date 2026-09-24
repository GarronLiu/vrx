#!/usr/bin/env python

import rospy
from mavros_msgs.msg import OverrideRCIn, RCOut, State
from nav_msgs.msg import Odometry
import time
import math


class MotorControlNode:
    def __init__(self):
        rospy.init_node("motor_control_node", anonymous=True)
        self.pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=10)
        self.rate = rospy.Rate(100)  # 10 Hz

        self.pwm_min = 1000
        self.end_value_left = None
        self.duration = 2  # 信号切换过渡时间
        self.angle_threshold = 30  # Z型实验首向角偏转切换角度阈值

        self.initial_yaw = None  # 初始化为0
        self.current_yaw = None  # 初始化为0
        self.left_pwm = None  # 确保1通道是产生正向旋转的力
        self.right_pwm = None  # 确保3通道是产生反向旋转的力
        self.right_pwm_max = None
        self.left_pwm_max = None
        self.increasing_yaw = True  # 标志位，控制yaw角的增加和减少
        self.change_flag = False
        self.step_left = None
        self.step_right = None
        self.current_state = State()
        self.z_count = 0
        self.accumulate_yaw = 0
        self.spiral_change_flag = False
        self.left_spiral_pwm = None
        self.right_spiral_pwm = None
        self.last_yaw = 0

        rospy.Subscriber("/wamv/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/mavros/state", State, self.state_callback)

    def state_callback(self, msg):
        self.current_state = msg
        rospy.loginfo("Armed State: %s", self.current_state.armed)

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = self.euler_from_quaternion(
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        )

        if self.initial_yaw is None:
            self.initial_yaw = yaw
            self.last_yaw = yaw
        self.current_yaw = yaw

        #for z manuver
        dyaw = None
        if self.increasing_yaw:
            dyaw = self.current_yaw - self.initial_yaw
            if dyaw > math.pi:
                dyaw -= 2 * math.pi
            elif dyaw < -math.pi:
                dyaw += 2 * math.pi

            if dyaw >= math.radians(self.angle_threshold - 0.01):
                self.z_count += 1
                self.increasing_yaw = False
                self.change_flag = True
        else:
            dyaw = self.initial_yaw - self.current_yaw
            if dyaw > math.pi:
                dyaw -= 2 * math.pi
            elif dyaw < -math.pi:
                dyaw += 2 * math.pi

            if dyaw >= math.radians(self.angle_threshold - 0.01):
                self.increasing_yaw = True
                self.change_flag = True
        #for spiral manuver
        self.accumulate_yaw += self.current_yaw - self.last_yaw
        self.last_yaw = self.current_yaw
        if math.fabs(self.accumulate_yaw) >= math.pi - 0.01:
            self.accumulate_yaw = 0
            self.spiral_change_flag = True

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def smooth_channels(self, start, end, step):
        if start < end:
            return min(start + step, end)
        else:
            return max(start - step, end)

    def control_motor(self):
        start_time = time.time()
        self.left_pwm = 1500
        self.right_pwm = 1500
        self.right_pwm_max = 2000
        self.left_pwm_max = 2000
        self.step_left = (self.left_pwm_max - self.pwm_min) / self.duration / 100
        self.step_right = (self.right_pwm_max - self.pwm_min) / self.duration / 100

        stage = 0
        
        while not rospy.is_shutdown():
            # 如果飞控状态不是ARMED，不发布override消息
            if not self.current_state.armed:
                msg = OverrideRCIn()
                msg.channels = [0] * 18  # 将所有通道设置为0
                self.pub.publish(msg)
                stage = 0
                continue
            # 加速阶段
            if stage == 0:
                if time.time() - start_time < 10:
                    msg = OverrideRCIn()
                    self.left_pwm = self.smooth_channels(self.left_pwm, 2000, self.step_left)
                    self.right_pwm = self.smooth_channels(self.right_pwm, 2000, self.step_right)
                    msg.channels = [int(self.left_pwm), 0, int(self.right_pwm)] + [0] * 15  # 将所有通道设置为相同的值
                    self.pub.publish(msg)
                elif time.time() - start_time < 20:
                    msg = OverrideRCIn()
                    self.left_pwm = self.smooth_channels(self.left_pwm, 1100, self.step_left)
                    self.right_pwm = self.smooth_channels(self.right_pwm, 1100, self.step_right)
                    msg.channels = [int(self.left_pwm), 0, int(self.right_pwm)] + [0] * 15  # 将所有通道设置为相同的值
                    self.pub.publish(msg)
                elif time.time() - start_time < 30:
                    msg = OverrideRCIn()
                    self.left_pwm = self.smooth_channels(self.left_pwm, 2000, self.step_left)
                    self.right_pwm = self.smooth_channels(self.right_pwm, 2000, self.step_right)
                    msg.channels = [int(self.left_pwm), 0, int(self.right_pwm)] + [0] * 15  # 将所有通道设置为相同的值
                    self.pub.publish(msg)
                else:
                    stage = 1
                    start_time = time.time()
                    print("Start Z Manuver")
            # Z型实验阶段
            elif stage == 1:
                if self.z_count < 3:
                    if self.increasing_yaw:
                        msg = OverrideRCIn()
                        self.left_pwm = self.smooth_channels(self.left_pwm, 1250, self.step_left)
                        self.right_pwm = self.smooth_channels(self.right_pwm, 2000, self.step_right)
                        msg.channels = [int(self.left_pwm), 0, int(self.right_pwm)] + [
                            0
                        ] * 15  # 将所有通道设置为相同的值
                        self.pub.publish(msg)
                    else:
                        msg = OverrideRCIn()
                        self.left_pwm = self.smooth_channels(self.left_pwm, 2000, self.step_left)
                        self.right_pwm = self.smooth_channels(self.right_pwm, 1250, self.step_right)
                        msg.channels = [int(self.left_pwm), 0, int(self.right_pwm)] + [
                            0
                        ] * 15  # 将所有通道设置为相同的值
                        self.pub.publish(msg)
                else:
                    stage = 2
                    start_time = time.time()
                    print("Start left turn")
            #左回转阶段
            elif stage == 2:
                if time.time() - start_time < 30:
                    msg = OverrideRCIn()
                    self.left_pwm = self.smooth_channels(self.left_pwm, 1800, self.step_left)
                    self.right_pwm = self.smooth_channels(self.right_pwm, 2000, self.step_right)
                    msg.channels = [int(self.left_pwm), 0, int(self.right_pwm)] + [
                        0
                    ] * 15  # 将所有通道设置为相同的值
                    self.pub.publish(msg)
                else:
                    stage = 3
                    start_time = time.time()
                    print("Start right turn")
            #右回转阶段
            elif stage == 3:
                if time.time() - start_time < 30:
                    msg = OverrideRCIn()
                    self.left_pwm = self.smooth_channels(self.left_pwm, 2000, self.step_left)
                    self.right_pwm = self.smooth_channels(self.right_pwm, 1800, self.step_right)
                    msg.channels = [int(self.left_pwm), 0, int(self.right_pwm)] + [
                        0
                    ] * 15  # 将所有通道设置为相同的值
                    self.pub.publish(msg)
                else:
                    stage = 4
                    start_time = time.time()
                    self.accumulate_yaw = 0
                    self.spiral_change_flag = False
                    self.left_spiral_pwm = 2000
                    self.right_spiral_pwm = 1900
                    print("Start right spiral")
            #右回旋螺线实验
            elif stage == 4:
                if self.spiral_change_flag:
                    self.spiral_change_flag = False
                    self.accumulate_yaw = 0
                    if self.right_spiral_pwm > 1000:
                        self.right_spiral_pwm -= 100
                    else:
                        stage = 5
                        print("Start left spiral")
                        start_time = time.time()
                        self.accumulate_yaw = 0
                        self.spiral_change_flag = False
                        self.left_spiral_pwm = 1900
                        self.right_spiral_pwm = 2000
                        continue
                msg = OverrideRCIn()
                self.left_pwm = self.smooth_channels(self.left_pwm, self.left_spiral_pwm, self.step_left)
                self.right_pwm = self.smooth_channels(self.right_pwm, self.right_spiral_pwm, self.step_right)
                msg.channels = [int(self.left_pwm), 0, int(self.right_pwm)] + [
                        0
                    ] * 15  # 将所有通道设置为相同的值
                self.pub.publish(msg)

            #左回旋螺线实验
            elif stage == 5:
                if self.spiral_change_flag:
                    self.spiral_change_flag = False
                    self.accumulate_yaw = 0
                    if self.left_spiral_pwm > 1000:
                        self.left_spiral_pwm -= 100
                    else:
                        self.left_pwm = 1500
                        self.right_pwm = 1500
                        stage = 6
                        print("Finished!")
                        break
                msg = OverrideRCIn()
                self.left_pwm = self.smooth_channels(self.left_pwm, self.left_spiral_pwm, self.step_left)
                self.right_pwm = self.smooth_channels(self.right_pwm, self.right_spiral_pwm, self.step_right)
                msg.channels = [int(self.left_pwm), 0, int(self.right_pwm)] + [
                        0
                    ] * 15
                self.pub.publish(msg)

            self.rate.sleep()

            

        # 停止发布消息
        msg = OverrideRCIn()
        msg.channels = [0] * 18  # 将所有通道设置为0
        self.pub.publish(msg)

    def run(self):
        rospy.sleep(2)  # 等待订阅者初始化
        self.control_motor()
        rospy.spin()


if __name__ == "__main__":
    try:
        node = MotorControlNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
