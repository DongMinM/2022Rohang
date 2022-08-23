import rospy
from std_msgs.msg import String
import pandas as pd
import numpy as np
from queue import Queue

class Saver:

    def __init__(self):
        self.pd = pd.DataFrame()

        rospy.init_node('dataSaver')
        rospy.Subscriber('isAuto',String,self.callback_isAuto)
        rospy.Subscriber('gps',String,self.callback_gps)
        self.isAuto_q = Queue()
        self.gps_q = Queue()
        self.write()

    def callback_isAuto(self,msg):
        self.isAuto_q.put(msg)
        

    def callback_gps(self,msg):
        self.gps_q.put(msg)


    def write(self):
        isAuto = self.isAuto_q.get()
        gps = self.gps_q.get()
        print(isAuto)
        print(gps)

if __name__ == '__main__':
    saver = Saver()