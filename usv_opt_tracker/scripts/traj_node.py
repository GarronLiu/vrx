#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import math
import csv


class TrajectoryNode:
    def __init__(self):
        rospy.init_node("trajectory_node", anonymous=True)
        self.path_pub = rospy.Publisher(
            "/manager/trajectory/tracking", Path, queue_size=10
        )
        self.odom_sub = rospy.Subscriber(
            "/wamv/odom", Odometry, self.odom_callback
        )

        self.path = Path()
        self.path.header.frame_id = "odom"
        self.initial_pose = None
        self.current_pose = None
        self.trajectory_type = "eight"  # 可选值: "circle", "eight", "custom"
        self.model_type_str = "wamv"
        self.radius = 10.0  # 圆形轨迹的半径
        self.eight_length = 30.0  # 八字形轨迹的长度
        self.custom_points = []  # 任意形状轨迹的点列表

    def odom_callback(self, msg):
        if self.initial_pose is None:
            self.initial_pose = msg.pose.pose
            self.initial_pose.position.y += 2.0
            rospy.loginfo("Initial pose set: %s", self.initial_pose)
            self.generate_trajectory()

        self.current_pose = msg.pose.pose
        if self.is_back_to_start():
            # rospy.loginfo("Back to start, publishing trajectory")
            self.path.header.stamp = rospy.Time.now()
            self.path_pub.publish(self.path)

    def is_back_to_start(self):
        if self.initial_pose is None or self.current_pose is None:
            return False

        distance = math.sqrt(
            (self.current_pose.position.x - self.initial_pose.position.x) ** 2
            + (self.current_pose.position.y - self.initial_pose.position.y) ** 2
        )
        return distance < 2.0  # 距离阈值，可根据需要调整

    def generate_trajectory(self):
        if self.trajectory_type == "circle":
            self.generate_circle_trajectory()
        elif self.trajectory_type == "eight":
            self.generate_eight_trajectory()
        elif self.trajectory_type == "custom":
            self.generate_custom_trajectory()
        self.save_trajectory_to_csv()

    def save_trajectory_to_csv(self):
        with open(f'{self.trajectory_type}_trajectory_{self.model_type_str}.csv', mode='w') as file:
            writer = csv.writer(file)
            writer.writerow(['x', 'y'])
            for pose in self.path.poses:
                writer.writerow([pose.pose.position.x, pose.pose.position.y])

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

    def generate_circle_trajectory(self):
        num_points = 100
        _, _, initial_angle = self.euler_from_quaternion(
            self.initial_pose.orientation.x,
            self.initial_pose.orientation.y,
            self.initial_pose.orientation.z,
            self.initial_pose.orientation.w,
        )
        initial_angle = initial_angle + math.pi / 2
        for i in range(num_points):
            angle = initial_angle - 2 * math.pi * i / num_points
            x = (
                self.initial_pose.position.x
                + self.radius * math.cos(angle)
                + self.radius * math.sin(initial_angle - math.pi / 2)
            )
            y = (
                self.initial_pose.position.y
                + self.radius * math.sin(angle)
                - self.radius * math.cos(initial_angle - math.pi / 2)
            )
            z = self.initial_pose.position.z

            pose = PoseStamped()
            pose.header.frame_id = "odom"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation = self.initial_pose.orientation

            self.path.poses.append(pose)

    def generate_eight_trajectory(self):
        num_points = 100
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = self.initial_pose.position.x + self.eight_length * math.sin(angle)
            y = (
                self.initial_pose.position.y
                + self.eight_length * math.sin(2 * angle) / 2
            )
            z = self.initial_pose.position.z

            pose = PoseStamped()
            pose.header.frame_id = "odom"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation = self.initial_pose.orientation

            self.path.poses.append(pose)
        pose_end = self.path.poses[0]
        self.path.poses.append(pose_end)


    def generate_custom_trajectory(self):
        self.custom_points.append((0.0, 0.0, 0.0))
        self.custom_points.append((15.0, 0.0, 0.0))
        num_points = 40
        radius_ = 10.0
        for i in range(num_points):
            angle = math.pi/2 -  math.pi * i / (num_points - 1)
            x = 15.0 + radius_ * math.cos(angle)
            y = -radius_ + radius_ * math.sin(angle)
            self.custom_points.append((x, y, 0.0))

        last_point = self.custom_points[-1]
        radius_ = 7.5
        num_points = 30
        for i in range(num_points):
            angle = math.pi/2 +  math.pi * i / (num_points - 1)
            x = last_point[0] + radius_ * math.cos(angle)
            y = last_point[1]-radius_ + radius_ * math.sin(angle)
            self.custom_points.append((x, y, 0.0))
        
        last_point = self.custom_points[-1]
        num_points = 20
        radius_ = 5
        for i in range(num_points):
            angle = math.pi/2 -  math.pi * i / (num_points - 1)
            x = last_point[0] + radius_ * math.cos(angle)
            y = last_point[1]-radius_ + radius_ * math.sin(angle)
            self.custom_points.append((x, y, 0.0))
        
        last_point = self.custom_points[-1]
        self.custom_points.append((last_point[0]-10, last_point[1], 0.0))

        last_point = self.custom_points[-1]
        num_points = 20
        radius_ = 5
        for i in range(num_points):
            angle = math.pi*3/2 -  math.pi/2 * i / (num_points - 1)
            x = last_point[0] + radius_ * math.cos(angle)
            y = last_point[1] + radius_ + radius_ * math.sin(angle)
            self.custom_points.append((x, y, 0.0))

        last_point = self.custom_points[-1]
        num_points = 30
        radius_ = 7.5
        for i in range(num_points):
            angle = math.pi * i / (num_points - 1)
            x = last_point[0] -radius_ + radius_ * math.cos(angle)
            y = last_point[1] + radius_ * math.sin(angle)
            self.custom_points.append((x, y, 0.0))

        last_point = self.custom_points[-1]
        num_points = 20
        radius_ = 5
        for i in range(num_points):
            angle = -  math.pi/2 * i / (num_points - 1)
            x = last_point[0] - radius_ + radius_ * math.cos(angle)
            y = last_point[1] + radius_ * math.sin(angle)
            self.custom_points.append((x, y, 0.0))

        last_point = self.custom_points[-1]
        num_points = 60
        radius_ = 15
        for i in range(num_points):
            angle = math.pi*3/2 -  (math.pi - math.atan2(2,3))* i / (num_points - 1)
            x = last_point[0] + radius_ * math.cos(angle)
            y = last_point[1] + radius_ + radius_ * math.sin(angle)
            self.custom_points.append((x, y, 0.0))
        self.custom_points.append((0.0, 0.0, 0.0))
        last_point = self.custom_points[-1]
        print(last_point)

        scale = 0.5
        for point in self.custom_points:
            pose = PoseStamped()
            pose.header.frame_id = "odom"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = self.initial_pose.position.x + point[0]*scale
            pose.pose.position.y = self.initial_pose.position.y + point[1]*scale
            pose.pose.position.z = self.initial_pose.position.z + point[2]*scale
            pose.pose.orientation = self.initial_pose.orientation

            self.path.poses.append(pose)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = TrajectoryNode()
        node.run()
    except rospy.ROSInterruptException:
        pass