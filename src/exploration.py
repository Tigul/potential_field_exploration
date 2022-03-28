import rospy
import numpy as np
import threading
import tf
import time
import tf2_ros

from scipy.stats import norm
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Twist
from tf2_geometry_msgs.msg import PoseStamped, Pose

class Explorer:
    def __init__(self):
        self.map = None
        self.map_metadata = None
        self.potential_field = None
        rospy.Subscriber("/map_metadata", MapMetaData, self.update_meta_data )
        rospy.Subscriber("/map", OccupancyGrid, self.update_map)
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        #self.gauss = np.outer(self._gaussian_window, self._gaussian_window)
        self.vector = None
        self.new_vector = None
        self.current_position = None

        self.tf_listener = tf.TransformListener()


    def update_map(self, msg):
        if self.map_metadata:
            self.map = np.reshape(msg.data, (self.map_metadata.height, self.map_metadata.width))
            self.current_position = np.array(self.tf_listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))[0])
            self.current_position = np.array([self.current_position[0], self.current_position[1]])
            self.vector = self.calculate_new_vector(self._create_sub_map(self.current_position, 200))
            self._publish_twist_msg()

    def update_meta_data(self, msg):
        self.map_metadata = msg

    def calculate_new_vector(self, map):
        height, width = map.shape
        sum_targets = np.double(0)
        sum_obstacles = np.double(0)
        for x in range(height):
            for y in range(width):
                if map[x][y] < 0:
                    direction = np.array([x, y]) - self.current_position
                    distance = np.linalg.norm(direction)
                    scaled_dist = self.target_function(distance)
                    sum_targets += scaled_dist * direction
                if map[x][y] > 0:
                    direction = np.array([x, y]) - self.current_position
                    distance = np.linalg.norm(direction)
                    scaled_dist = self.obstacle_function(distance)
                    sum_obstacles += scaled_dist * direction
        self.new_vector  =  sum_targets - sum_obstacles
        print(self.new_vector)

    def target_function(self, dist):
        return np.double(norm.pdf(dist)) * 10

    def obstacle_function(self, dist):
        return np.double(norm.pdf(dist)) * 5

    def _create_sub_map(self, sub_origin, size):
        """
        Creates a smaller map from the overall occupancy map, the new map is centered
        around the point specified by "sub_origin" and has the size "size". The
        resolution of the costmap stayes the same for the sub costmap.
        :param sub_origin: The point in global coordinate frame, around which the
            sub costmap should be centered.
        :param size: The size the sub costmap should have.
        :return The sub costmap, represented as 2d numpy array.
        """
        # To ensure this is a numpy array
        sub_origin = np.array(sub_origin)
        # Since origin obtained from the meta data uses bottom left corner as reference.
        sub_origin *= -1
        # Calculates origin of sub costmap as vector between origin and given sub_origin
        origin = np.array([self.map_metadata.origin.position.x, self.map_metadata.origin.position.y])
        new_origin = np.array(origin) + sub_origin
        # Convert from vector in meter to index values
        new_origin /= self.map_metadata.resolution
        #new_origin = np.array([abs(int(x)) for x in new_origin])
        new_origin = np.abs(new_origin)
        # Offset to top left corner, for easier slicing
        new_origin = (new_origin - size/2).astype(int)
        #print(new_origin)

        # slices a submap with size "size" around the given origin
        sub_map = self.map[new_origin[1]: new_origin[1] + size,
                            new_origin[0]: new_origin[0] + size]
        return sub_map
        # Convert map to fit with the other costmaps
        #sub_map = np.rot90(np.flip(self._convert_map(sub_map, size), 0))
        #return Costmap(self.resolution, size, size, list(sub_origin * -1), sub_map)

    def _publish_twist_msg(self):
        if not self.vector:
            self.vector = self.new_vector
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
        #angle = np.arctan2(new_pose[0] - self.current_position[0], new_pose[1] - self.current_position[1])
        angle = np.arccos(self.new_vector @ np.array([1, 0]) / max(np.linalg.norm(self.new_vector), 0.01))

        while angle > 0.1:
            msg.angular.z = 0.05
            self.twist_pub.publish(msg)
            #angle = np.arctan2(new_pose[0] - self.current_position[0], new_pose[1] - self.current_position[1])
            angle = np.arccos(self.new_vector @ np.array([1, 0]) / max(np.linalg.norm(self.new_vector), 0.01))

            print(angle)
            time.sleep(0.1)
        msg.angular.z = 0
        self.twist_pub.publish(msg)

    def _calculate_angle(self):
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)
        pose = Pose()
        pose.position.x = 1

        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = pose
        pose_stamped.header.frame_id = "base_footprint"
        pose_stamped.header.stamp = rospy.Time.now()

        try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_pose_stamped = tf_buffer.transform(pose_stamped, "map", rospy.Duration(1))
            #return output_pose_stamped.pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        pose = np.array([output_pose_stamped.pose.position.x, output_pose_stamped.pose.position.y])
        robot_vector = pose - self.current_position

        angle = np.arccos(self.new_vector @ robot_vector / np.linalg.norm(self.new_vector) * np.linalg.norm(robot_vector))
        return angle





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
