import rospy
import numpy as np
import threading
import tf
import cv2
import math
import time
import matplotlib.pyplot as plt
import tf2_ros
import tf2_geometry_msgs

from scipy.stats import norm
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from geometry_msgs.msg import Twist, Pose, PointStamped, Point
from tf2_geometry_msgs import PoseStamped
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

class Explorer:
    def __init__(self):
        self.map = None
        self.map_metadata = None
        rospy.Subscriber("/map_metadata", MapMetaData, self.update_meta_data )
        rospy.Subscriber("/map", OccupancyGrid, self.update_map)
        rospy.Subscriber("/odom", Odometry, self.update_orientation )
        self._timer = rospy.Timer(rospy.Duration(0.5), self.calculate_new_vector)
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
        self.new_vector = None
        self.current_position = None
        self.current_orientation = None
        self.direction = None
        self.obstacles = None
        self.targets = None
        self.origin_top_left = None
        self.tf_listener = tf.TransformListener()
        self.frontiers = None

    def calc_wheel_position(self):
        position = self.tf_listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))[0]
        left_wheel = position[1] + 0.3
        right_wheel = position[1] - 0.3
        h = Header()
        p = PointStamped(h, Point(position[0], 0, position[2]))
        p.header.frame_id = "base_footprint"
        p.point.y = left_wheel
        left_in_map = self.tf_listener.transformPoint("map", p)
        p.point.y = right_wheel
        right_in_map = self.tf_listener.transformPoint("map", p)

    def update_orientation(self, msg):
        q = msg.pose.pose.orientation
        self.current_orientation = np.array([q.x, q.y, q.z, q.w])
        #print(tf.transformations.euler_from_quaternion(self.current_orientation))
        p = msg.pose.pose.position
        self.current_position = np.array([p.x, p.y])
        #self.direction = self.current_position - self.last_position

    def update_map(self, msg):
        if self.map_metadata:
            self.map = np.reshape(msg.data, (self.map_metadata.height, self.map_metadata.width))
            self.targets = np.array([10., 10.])
            self.frontiers = cv2.Canny(self.map.astype("uint8"), 25, 1)
            plt.imshow(self.frontiers)
            plt.show()
            lines = cv2.HoughLines(self.frontiers, 1, np.pi/180, 1)
            #cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
            if lines is not None:
                for i in range(0, len(lines)):
                    rho = lines[i][0][0]
                    theta = lines[i][0][1]
                    a = math.cos(theta)
                    b = math.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
                    pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
                    cv2.line(self.frontiers, pt1, pt2, (0, 0, 255), 3, cv2.LINE_AA)
                    print(f"Point1: {pt1}, Point2: {pt2}")
            plt.imshow(self.frontiers)
            plt.show()
            exit()




    def update_meta_data(self, msg):
        self.map_metadata = msg
        self.origin_top_left = np.array([msg.origin.position.x, msg.origin.position.y]) * -1

    def calculate_new_vector(self, a):
        if self.current_position is None or not self.map_metadata or self.map is None:
            return
        self.obstacles = np.transpose((self.map > 0).nonzero())
        self.obstacles = self._convert_index_to_meter(self.obstacles)
        target = np.array([[10., 0.]])
        obs = np.sum(self.obstacle_function(np.linalg.norm(self.obstacles - self.current_position, axis=1, keepdims=True)) * (self.obstacles - self.current_position), axis=0)
        tar = np.sum(self.target_function(np.linalg.norm(target - self.current_position, axis=1)) * (target - self.current_position), axis=0)
        print(f"Sum Obstacles: {obs}")
        print(f"Sum Targets: {tar}")
        self.new_vector = tar - obs
        print(self.new_vector)
        # Stuff specific to the test simulation to convert between coordinate systems
        #self.new_vector[0], self.new_vector[1] = self.new_vector[1], self.new_vector[0]
        #self.new_vector *= np.array([1, -1])
        self._publish_marker()
        self._publish_twist_msg()

    def target_function(self, dist):
        func = 1 / len(self.targets) * 0.001
        return 1

    def obstacle_function(self, dist):
        active_obs = dist < 1
        dist[~active_obs] = 100
        #dist[active_obs] = dist[active_obs] - 0.3
        num_active = sum(active_obs)
        func = (12 / dist) * (1 / np.maximum(num_active, 1))
        func[~active_obs] = 0
        return func

    def _convert_index_to_meter(self, ind):
        ind = ind.astype(float)
        ind *= self.map_metadata.resolution
        ind -= self.origin_top_left
        ind = np.flip(ind, 1)
        return ind

    def _publish_twist_omni(self):
        msg = Twist()
        norm_vector = (self.new_vector / np.linalg.norm(self.new_vector)) / 2
        msg.linear.x = norm_vector[0]
        msg.linear.y = norm_vector[1]
        self.twist_pub.publish(msg)

    def _publish_twist_msg(self):
        msg = Twist()

        angle = self._calculate_angle2()
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(self.current_orientation)
        #print(f"Angle3: {self._calc_angle3()}")
        #diff = (angle +np.pi) - (yaw +np.pi)
        diff = angle - yaw
        msg.angular.z = diff
        msg.linear.x = 0.1
        self.twist_pub.publish(msg)

    def _calculate_angle2(self):
        norm_vector = self.new_vector / np.linalg.norm(self.new_vector)
        goal = self.current_position + norm_vector
        #goal = self.current_position + np.array([1, 0])
        #angle_to_goal = np.arctan2(self.current_position[1] - goal[1], self.current_position[0] - goal[0])
        angle_to_goal = np.arctan2(goal[1] - self.current_position[1], goal[0] - self.current_position[0])
        angle_as_quanternion = list(tf.transformations.quaternion_from_euler(0, 0, angle_to_goal, axes="sxyz"))
        return angle_to_goal

    def _publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time()
        marker.id = 0
        marker.type = 0  # Type = ARROW
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1

        marker.scale.x = 0.05
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        base_point = Point()
        base_point.x = self.current_position[0]
        base_point.y = self.current_position[1]
        base_point.z = 0
        marker.points.append(base_point)

        norm_vector = self.new_vector / np.linalg.norm(self.new_vector)
        tip_position = self.current_position + norm_vector
        tip = Point()
        tip.x = tip_position[0]
        tip.y = tip_position[1]
        tip.z = 0

        marker.points.append(tip)
        marker.color.r = 1
        marker.color.a = 1

        marker.lifetime = rospy.Duration(1)

        self.marker_pub.publish(marker)

    def _gaussian_window(self, mean, std):
        """
        This method creates a window of values with a gaussian distribution of
        size "mean" and standart deviation "std".
        Code from: https://github.com/scipy/scipy/blob/v0.14.0/scipy/signal/windows.py#L976
        """
        n = np.arange(0, mean) - (mean - 1.0) / 2.0
        sig2 = 2 * std * std
        w = np.exp(-n ** 2 / sig2)
        return w


if __name__ == "__main__":
    rospy.init_node("pf_exploration")
    ex = Explorer()
    rospy.spin()
