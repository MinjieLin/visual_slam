#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import CameraInfo, Image


msg = CameraInfo()
pub = None

def callback(data):
	msg.header.stamp = data.header.stamp
	pub.publish(msg);

def talker():
	global pub
	pub = rospy.Publisher('/usb_cam/camera_info', CameraInfo, queue_size=10)
	rospy.init_node('info_subscriber', anonymous=True)
	msg.header.frame_id = "usb_cam"
	msg.height = 480
	msg.width = 640
	msg.distortion_model = "plumb_bob"
	msg.D = [0.0115826290815515, -0.1424046390890178, 0.03065724102625946, 0.0001954542904077437, 0.0]
	msg.K = [530.0209803541096, 0.0, 314.6111575598102, 0.0, 529.8981709625571, 265.6956752531768, 0.0, 0.0, 1.0]
	msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
	msg.P = [525.9034659871668, 0.0, 313.747642454904, 0.0, 0.0, 519.9081604233438, 276.4498964781748, 0.0, 0.0, 0.0, 1.0, 0.0]

	rospy.Subscriber("/usb_cam/image_raw", Image, callback)

	rospy.spin();
    
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
