import rospy
import cv2
import  math
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry

if __name__ == "__main__":
    rospy.init_node("test_line")
    map_meta = rospy.wait_for_message("/map_metadata", MapMetaData)
    map = rospy.wait_for_message("/map", OccupancyGrid).data
    map = np.array(map).reshape((map_meta.height, map_meta.width)).astype('uint8')

    edges = cv2.Canny(map, 25, 1)
    plt.imshow(edges)
    plt.show()

    lines = cv2.HoughLines(edges, 1, np.pi/180, 1)
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
            cv2.line(edges, pt1, pt2, (0, 0, 255), 3, cv2.LINE_AA)
            print(f"Point1: {pt1}, Point2: {pt2}")
    plt.imshow(edges)
    plt.show()
