import rospy
from std_msgs.msg import String, Float32MultiArray
import pandas as pd
import numpy as np
from queue import Queue
import time
import threading

class Saver:

    def __init__(self):
        self.df = pd.DataFrame()

        rospy.init_node('dataSaver')
        rospy.Subscriber('isAuto',String,self.callback_isAuto)
        rospy.Subscriber('gpsTime',String,self.callback_time)
        rospy.Subscriber('gps',Float32MultiArray,self.callback_gps)
        rospy.Subscriber('heading',String,self.callback_heading)

        self.isAuto_q = Queue()
        self.gps_q = Queue()
        self.time_q = Queue()
        self.heading_q = Queue()

        self.trigger = 0
        threading.Thread(target = self.set_trigger).start()
        self.write()


    def callback_isAuto(self,msg):
        self.isAuto_q.put(msg.data)

    def callback_time(self,msg):
        self.time_q.put(msg.data)

    def callback_gps(self,msg):
        self.gps_q.put(msg.data)

    def callback_heading(self,msg):
        self.heading_q.put(msg.data)

    def write(self):
      print('Wait trigger')
      while True:
         if self.trigger == '1':
            print('on')
            t1 = time.time()
            isAuto = self.isAuto_q.get()
#            print(isAuto)
            t = self.time_q.get()
#            print(t)
            gps = self.gps_q.get()
#            print(gps)
            heading = self.heading_q.get()
#            print(heading)
            print('save')
            data =[[isAuto,round(gps[0],6),round(gps[1],6),round(gps[2],2),t[2:4],t[5:7],t[8:10],t[11:13],t[14:16],t[17:19],t[20:22],heading]]
            self.df = self.df.append(data,ignore_index=True)
            self.df.to_csv('Log.csv',index=False,header=None,sep=',')

         else:
            pass

    def set_trigger(self):
        while True:
            trigger = input()
            print(trigger)
            if trigger == '1' or trigger == '0':
                self.trigger = trigger
            else :
                print('Check')

if __name__ == '__main__':
     saver = Saver()



