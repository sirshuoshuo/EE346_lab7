#!/usr/bin/env python3
import rospy
import actionlib
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from enum import Enum, auto

class State(Enum):
    MOVE_TO_GOAL = auto()
    SEARCH_COLUMN = auto()
    WAIT = auto()

class HuskyHighlevelController:
    def __init__(self):
        self.p_ang = 1.0
        self.fixed_velocity = 0.3

        self.subscriber = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.msg = Twist()
        self.target_distance = None
        self.target_angle = None

        self.column_found = False
        self.is_searching_column = False

    def set_vel(self, vel, dof):
        if dof == "forward":
            self.msg.linear.x = vel
        elif dof == "ang":
            self.msg.angular.z = vel

    def publish_velocity(self):
        self.vel_pub.publish(self.msg)

    def laser_callback(self, msg):
        if not self.is_searching_column:
            return

        valid_ranges = [(r, i) for i, r in enumerate(msg.ranges) if r > 0]
        if not valid_ranges:
            rospy.logwarn("No valid points detected in laser scan!")
            return

        filtered_ranges = [r for r, _ in valid_ranges]

        if any(r < 0.18 for r in filtered_ranges):
            rospy.loginfo("Detected a column within 18cm. Stopping the robot.")
            self.stop_robot()
            self.column_found = True
            return

        mutation_points = self.find_mutation_points(filtered_ranges)

        if mutation_points:
            target_angle, target_distance = self.find_nearest_mutation_point(
                mutation_points, valid_ranges, msg
            )
            self.set_vel(-target_angle, "ang")
            self.set_vel(self.fixed_velocity, "forward")
            self.publish_velocity()

    def find_mutation_points(self, ranges):
        mutation_points = []
        for i in range(1, len(ranges) - 1):
            if abs(ranges[i] - ranges[i - 1]) > 0.17:
                mutation_points.append(i)
        return mutation_points

    def find_nearest_mutation_point(self, mutation_points, valid_ranges, msg):
        nearest_distance = float('inf')
        nearest_angle = 0.0

        for idx in mutation_points:
            distance = valid_ranges[idx][0]
            angle = msg.angle_min + valid_ranges[idx][1] * msg.angle_increment
            if distance < nearest_distance:
                nearest_distance = distance
                nearest_angle = angle

        return nearest_angle, nearest_distance

    def stop_robot(self):
        self.set_vel(0.0, "forward")
        self.set_vel(0.0, "ang")
        self.publish_velocity()

class StateMachine:
    def __init__(self):
        self.state = State.MOVE_TO_GOAL
        self.controller = HuskyHighlevelController()
        # self.goals = [
        #     (2.12, 1.58, 1.0),
        #     (-2.04, 3.05, 1.0),
        #     (-1.08, 0.48, 1.0),
        #     (-1.57, -0.51, 1.0),
        # ]
        self.goals = [
            # (-1.078, 0.478, 1.0),  # midle_stop_point
            # (1.034892201423645, 0.16465091705322266,1.0),   # p2
            (2.12,1.58,1.0),
            (2.01, 3.53, 1.0),  # p3
            # (2.03,4.28,1.0),
            # (-2.04, 3.05, 1.0), # p4
            (-1.40,3.82,1.0),
            # (-1.08, 0.48, 1.0),  # midle_stop_point
            # (-1.57, -0.51, 1.0) # p1
            (-1.42,-0.11,1.0)
        ]
        
        self.current_goal_index = 0

    def move_to_goal(self, x, y, w):
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = w

        rospy.loginfo(f"Sending goal: x={x}, y={y}, w={w}")
        client.send_goal(goal)

        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return False
        else:
            rospy.loginfo("Goal reached!")
            return True

    def stop_and_wait(self):
        self.controller.stop_robot()
        rospy.loginfo("Stopping for 2 seconds...")
        rospy.sleep(2)

    def run(self):
        while not rospy.is_shutdown():
            if self.state == State.MOVE_TO_GOAL:
                x, y, w = self.goals[self.current_goal_index]
                if self.move_to_goal(x, y, w):
                    if (x, y, w) == (-0.62, 0.53, 1.0):
                        rospy.loginfo("Skipping column search at this goal.")
                        self.state = State.WAIT
                    else:
                        rospy.loginfo("Starting column detection.")
                        self.controller.is_searching_column = True
                        self.controller.column_found = False
                        self.state = State.SEARCH_COLUMN

            elif self.state == State.SEARCH_COLUMN:
                if self.controller.column_found:
                    self.controller.is_searching_column = False
                    self.state = State.WAIT
                rospy.sleep(0.1)

            elif self.state == State.WAIT:
                self.stop_and_wait()
                self.current_goal_index += 1
                if self.current_goal_index >= len(self.goals):
                    rospy.loginfo("All goals reached. Stopping robot.")
                    break
                self.state = State.MOVE_TO_GOAL

if __name__ == '__main__':
    try:
        rospy.init_node('state_machine_navigation', anonymous=True)
        state_machine = StateMachine()
        state_machine.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Process interrupted.")
