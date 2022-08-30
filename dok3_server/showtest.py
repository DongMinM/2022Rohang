import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

br = CvBridge()
def callback(msg):
    image = msg.format


rospy.init_node('testview')
rospy.Subscriber('jps',Image,callback)

while True:
    pass