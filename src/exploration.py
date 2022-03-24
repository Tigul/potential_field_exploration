import rospy
import numpy as np
import threading

from nav_msgs.msg import OccupancyGrid, MapMetaData

class Explorer:
    def __init__(self):
        self.map = None
        self.map_metadata = None
        self.potential_field = None
        rospy.Subscriber("/map_metadata", MapMetaData, self.update_meta_data )
        rospy.Subscriber("/map", OccupancyGrid, self.update_map)
        self.gauss = np.outer(self._gaussian_window, self._gaussian_windon)


    def update_map(self, msg):
        if self.map_metadata:
            self.map = np.reshape(msg.data, (self.map_metadata.height, self.map_metadata.width))

    def update_meta_data(self, msg):
        self.map_metadata = msg

    def calculate_potential_field(self):
        ptf = np.array((self.map_metadata.height, self.map_metadata.width))
        for x in range(self.map_metadata.height):
            for y in range(self.map_metadata.width):
                pass

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
