import rospy
from sensor_msgs.msg import PointCloud2, CompressedImage
import sensor_msgs.point_cloud2 as pc2
import cv2
import message_filters

import numpy as np

idx = 0


def callback(velodyne_msg: PointCloud2, camera_msg: CompressedImage):
    global idx
    points = pc2.read_points(velodyne_msg, skip_nans=True)
    points = np.asarray(list(points), dtype=np.float32)
    pointcloud = points[:, 0:4]
    np_arr = np.frombuffer(camera_msg.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    bin_path = "../data/" + str(idx) + ".bin"
    img_path = "../data/" + str(idx) + ".png"
    with open(bin_path, "wb") as f:
        np.save(f, pointcloud)
    cv2.imwrite(img_path, image_np)
    idx += 1
    if idx > 3:
        exit(0)




if __name__ == '__main__':
    rospy.init_node('slice', anonymous=True)
    velodyne_sub = message_filters.Subscriber("/velodyne_points", PointCloud2)
    camera_sub = message_filters.Subscriber("/webcam/image_raw/compressed", CompressedImage)
    ts = message_filters.ApproximateTimeSynchronizer([velodyne_sub, camera_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback)
    rospy.spin()
