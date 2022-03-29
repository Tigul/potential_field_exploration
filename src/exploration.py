import rospy
import numpy as np
import threading
import tf
import time
import tf2_ros
import tf2_geometry_msgs

from scipy.stats import norm
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from geometry_msgs.msg import Twist, Pose
from tf2_geometry_msgs import PoseStamped

class Explorer:
    def __init__(self):
        self.map = None
        self.map_metadata = None
        self.potential_field = None
        rospy.Subscriber("/map_metadata", MapMetaData, self.update_meta_data )
        rospy.Subscriber("/map", OccupancyGrid, self.update_map)
        rospy.Subscriber("/odom", Odometry, self.update_orientation )
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        #self.gauss = np.outer(self._gaussian_window, self._gaussian_window)
        self.vector = None
        self.new_vector = None
        self.current_position = None
        self.current_orientation = None

        self.tf_listener = tf.TransformListener()

    def update_orientation(self, msg):
        q = msg.pose.pose.orientation
        self.current_orientation = np.array([q.x, q.y, q.z, q.w])


    def update_map(self, msg):
        if self.map_metadata:
            self.map = np.reshape(msg.data, (self.map_metadata.height, self.map_metadata.width))
            self.obstacles = np.transpose((self.map>0).nonzero())
            self.current_position = np.array(self.tf_listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))[0])
            self.current_position = np.array([self.current_position[0], self.current_position[1]])
            self.vector = self.calculate_new_vector()
            self._publish_twist_msg()

    def update_meta_data(self, msg):
        self.map_metadata = msg

    def calculate_new_vector(self):
        cp = self.current_position / self.map_metadata.resolution
        origin = np.array([self.map_metadata.origin.position.x, self.map_metadata.origin.position.y]) * -1 / self.map_metadata.resolution
        cp = origin + cp
        target = np.array([[100., 100.]])

        obs = np.sum(self.obstacle_function(np.abs(self.obstacles - self.current_position)) * (self.obstacles - self.current_position), axis=0)
        tar = np.sum(self.target_function(np.abs(target - self.current_position)) * (target - self.current_position), axis=0)
        self.new_vector = tar  - obs
        self.new_vector[0], self.new_vector[1] = self.new_vector[1], self.new_vector[0]
        self.new_vector *= np.array([1, -1])
        print(self.new_vector)
        self._publish_twist_msg
        #exit()

    def target_function(self, dist):
        #return np.exp(dist)
        return dist
        #return np.double(norm.pdf(dist)) * 10

    def obstacle_function(self, dist):
        return dist
        #return 1 / np.maximum(0.1, dist)
        #return -np.log(dist)
        #return np.double(norm.pdf(dist)) * 5


    def _publish_twist_msg(self):
        msg = Twist()
        msg.linear.x = 0
        self.twist_pub.publish(msg)
        self._rotate_robot()
        msg.linear.x = 0.2
        self.twist_pub.publish(msg)

    def _rotate_robot(self):
        #angle = np.arccos(self.new_vector @ self.vector /
        #        np.linalg.norm(self.new_vector) * np.linalg.norm(self.vector))
        msg = Twist()

        new_pose = self.current_position + self.new_vector
        angle = self._calculate_angle2()
        print(angle)
        diff = 1
        while diff > 0.1:
            msg.angular.z = 0.15
            self.twist_pub.publish(msg)
            #angle = np.arctan2(new_pose[0] - self.current_position[0], new_pose[1] - self.current_position[1])
            #angle = np.arccos(self.new_vector @ np.array([1, 0]) / max(np.linalg.norm(self.new_vector), 0.01))
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(self.current_orientation)
            diff = angle - yaw
            print(diff)
            time.sleep(0.1)
        msg.angular.z = 0
        self.twist_pub.publish(msg)


    def _calculate_angle2(self):
        norm_vector = self.new_vector / np.linalg.norm(self.new_vector)
        goal = self.current_position + norm_vector
        #goal = self.current_position + np.array([1, 0])
        angle_to_goal = (np.arctan2(self.current_position[1] - goal[1], self.current_position[0] - goal[0]) +np.pi) % np.pi
        angle_as_quanternion = list(tf.transformations.quaternion_from_euler(0, 0, angle_to_goal, axes="sxyz"))
        return angle_to_goal




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
