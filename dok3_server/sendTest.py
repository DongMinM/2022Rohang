import rospy
from std_msgs.msg import String, Float32MultiArray
import datetime
import numpy as np
import time

rospy.init_node('test_datas')
pub1 = rospy.Publisher('isAuto',String,queue_size=1)
pub2 = rospy.Publisher('gpsTime',String,queue_size=1)
pub3 = rospy.Publisher('gps',Float32MultiArray,queue_size=1)
pub4 = rospy.Publisher('heading',String,queue_size=1)
gps = Float32MultiArray()
while True:
    t = str(datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-5]+'0')
    pub1.publish('1')
    pub2.publish(t)
    gps.data = np.array([10,20,30])
    pub3.publish(gps)
    pub4.publish('4')
    time.sleep(0.1)