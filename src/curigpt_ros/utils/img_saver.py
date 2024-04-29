#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os

class ImageSaver:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        self.save_dir = "../assets/img/realtime_rgb"
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        self.image_count = 0

    def callback(self, data):
        try:
            # Convert ROS image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # Save the image
        img_filename = os.path.join(self.save_dir, "realtime_rgb.png")
        cv2.imwrite(img_filename, cv_image)
        # print(f"Saved {img_filename}")
        self.image_count += 1

def main():
    rospy.init_node('image_saver', anonymous=True)
    image_saver = ImageSaver()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()