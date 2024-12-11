#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class HuskyHighlevelController:
    def __init__(self):
        rospy.init_node('husky_highlevel_controller', anonymous=True)
        self.p_ang = 1.0
        self.fixed_velocity = 0.3

        self.subscriber = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.msg = Twist()

        # 目标位置和距离
        self.target_distance = None
        self.target_angle = None

    def set_vel(self, vel, dof):
        if dof == "forward":
            self.msg.linear.x = vel
        elif dof == "ang":
            self.msg.angular.z = vel

    def publish_velocity(self):
        self.vel_pub.publish(self.msg)

    def laser_callback(self, msg):
        # 去除所有等于零的点并保留对应的索引
        valid_ranges = [(r, i) for i, r in enumerate(msg.ranges) if r > 0]
        if not valid_ranges:
            rospy.logwarn("No valid points detected in laser scan!")
            return

        filtered_ranges = [r for r, _ in valid_ranges]
        filtered_indices = [i for _, i in valid_ranges]

        # 检查是否有物体距离小于18cm
        if any(r < 0.18 for r in filtered_ranges):
            rospy.loginfo("Detected an object within 18cm. Stopping the robot.")
            self.stop_robot()
            return

        # 找到突变点
        mutation_points = self.find_mutation_points(filtered_ranges)

        if mutation_points:
            # 筛选最近突变点
            target_angle, target_distance = self.find_nearest_mutation_point(
                mutation_points, filtered_indices, msg
            )
            self.target_angle = target_angle
            self.target_distance = target_distance

            self.set_vel(-target_angle, "ang")
            self.set_vel(self.fixed_velocity, "forward")
            self.publish_velocity()

    def find_mutation_points(self, ranges):
        mutation_points = []
        for i in range(1, len(ranges) - 1):
            if abs(ranges[i] - ranges[i - 1]) > 0.17:  # 突变点阈值
                mutation_points.append(i)
        return mutation_points

    def find_nearest_mutation_point(self, mutation_points, valid_indices, msg):
        # 筛选最近突变点
        nearest_distance = float('inf')
        nearest_angle = 0.0

        for idx in mutation_points:
            distance = msg.ranges[valid_indices[idx]]
            angle = msg.angle_min + valid_indices[idx] * msg.angle_increment

            if distance < nearest_distance:
                nearest_distance = distance
                nearest_angle = angle

        return nearest_angle, nearest_distance

    def stop_robot(self):
        """停止机器人运动。"""
        self.set_vel(0.0, "forward")
        self.set_vel(0.0, "ang")
        self.publish_velocity()


if __name__ == '__main__':
    controller = HuskyHighlevelController()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()
