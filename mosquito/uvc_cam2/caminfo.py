#!/usr/bin/env python
import roslib; roslib.load_manifest('uvc_cam')
import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.srv import SetCameraInfo

def talker():
    pub = rospy.Publisher('camera_info', String)
    rospy.init_node('talker')
    while not rospy.is_shutdown():
        str = "hello world %s"%rospy.get_time()
        rospy.loginfo(str)
        pub.publish(String(str))
        rospy.sleep(1.0)
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
