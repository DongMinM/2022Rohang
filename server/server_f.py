
import socket
import cv2
from cv2 import IMWRITE_JPEG_CHROMA_QUALITY
import numpy as np
import threading
import time
import base64

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

import signal
import os

from queue import Queue
# 참고1 https://walkinpcm.blogspot.com/2016/05/python-python-opencv-tcp-socket-image.html
# https://millo-l.github.io/Python-TCP-image-socket-%EA%B5%AC%ED%98%84%ED%95%98%EA%B8%B0-Server-Client/

'''  
thread 1 : wait client
thread 2 : receive datas
thread 3 : show image
'''



class ServerSocket:

    def __init__(self, ip, port):
        ## server ip and port
        self.TCP_IP = ip
        self.TCP_PORT = port

        self.img_client = 0

        rospy.init_node('server')

        # Set queues
        self.isAuto_q   = Queue()
        self.wayPoint_q = Queue()
        self.gps_q      = Queue()
        self.imu_q      = Queue()
        self.speed_q    = Queue()
        self.state_q    = Queue()
        self.image_q    = Queue()

        # callback threads start
        threading.Thread(target = self.pub_isAuto).start()
        threading.Thread(target = self.pub_wayPoint).start()
        threading.Thread(target = self.pub_gps).start()
        threading.Thread(target = self.pub_imu).start()
        threading.Thread(target = self.pub_speed).start()
        threading.Thread(target = self.pub_state).start()
        threading.Thread(target = self.pub_image).start()

        # server socket open (Main thread)
        self.socketOpen()

    ## close server socket
    def socketClose(self):
        self.sock.close()
        print(u'Server socket [ TCP_IP: ' + self.TCP_IP + ', TCP_PORT: ' + str(self.TCP_PORT) + ' ] is closed')

    ## open server socket
    def socketOpen(self):

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)           # server socket
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)         # server socket option
        self.sock.bind((self.TCP_IP, self.TCP_PORT))                            # server socket bind
        self.sock.listen(2)                                                     # number of client can be access at the same time (but not work for this code)
        print(u'Server socket [ TCP_IP: ' + self.TCP_IP + ', TCP_PORT: ' + str(self.TCP_PORT) + ' ] is opened')

        # accept every client, any time
        while True:
            try:
                client_socket, addr = self.sock.accept()
                print('접속 : ',(addr))

                # thread that receive datas
                threading.Thread(target=self.receivedatas, args =(client_socket,addr)).start()

            except KeyboardInterrupt:
                self.socketClose()

    ## receive data from client socket
    def receivedatas(self,client_socket,addr):
        try:
            while True:

                length = self.recvall(client_socket, 64)
                stringData = self.recvall(client_socket, int(length))
                header = stringData[0]

                # isAuto (a)
                if header == 97:
                        if not self.isAuto_q.empty():
                            self.isAuto_q.get()
                        self.isAuto_q.put(stringData)

                # wayPoint (w)
                elif header == 119:
                    if not self.wayPoint_q.empty():
                        self.wayPoint_q.get()
                    self.wayPoint_q.put(stringData)

                # gps  (g)
                elif header == 103:
                    if not self.gps_q.empty():
                        self.gps_q.get()
                    self.gps_q.put(stringData)

                # imu (i)
                elif header == 105:
                    if not self.imu_q.empty():
                        self.image_q.get()
                    self.imu_q.put(stringData)

                # speed (s)
                elif header == 115:
                    if not self.speed_q.empty():
                        self.speed_q.get()
                    self.speed_q.put(stringData)

                # state (t)
                elif header == 116:
                    if not self.state_q.empty():
                        self.state_q.get()
                    self.state_q.put(stringData)

                # image
                else:
                    if not self.image_q.empty():
                        self.image_q.get()
                    self.img_client = addr
                    self.image_q.put(stringData)

        ## exception / client is out
        except Exception as e:
                print(e)
                ## if img_client is out, stop img_show
                if addr == self.img_client:
                    self.img_update = 0
                print('이미지 커넥션 종료 : ',(addr))
                time.sleep(1)


    ## recv all data (client socket, data count)
    def recvall(self, sock, count):
        buf = b''
        while count:
            newbuf = sock.recv(count)
            if not newbuf: return None
            buf += newbuf
            count -= len(newbuf)
        return buf


    def pub_isAuto(self):
        pub_isAuto = rospy.Publisher('isAuto',String, queue_size=1)
        while True:
            isAuto = self.isAuto_q.get()
            isAuto = isAuto.decode('utf8')
            print(f'ifAuto : {isAuto[1:]}')
            pub_isAuto.publish(isAuto[1:])

    def pub_wayPoint(self):
        pub_wayPoint = rospy.Publisher('wayPoint',String, queue_size=1)
        while True:
            wayPoint = self.wayPoint_q.get()
            wayPoint = wayPoint.decode('utf8')
            wayPoint = wayPoint.split(',')
            print(f'Way Point : {wayPoint[1]}')
            pub_wayPoint.publish(wayPoint[1])

    def pub_gps(self):
        pub_gpsTime = rospy.Publisher('gpsTime',String, queue_size=1)
        pub_latitude = rospy.Publisher('latitude',String, queue_size=1)
        pub_longitude = rospy.Publisher('longitude',String, queue_size=1)
        pub_altitude = rospy.Publisher('altitude',String, queue_size=1)
        while True:
            gps_data = self.gps_q.get()
            gps_data = gps_data.decode('utf8')
            gps_data = gps_data.split(',')
            print(f'Gps :{gps_data[1:]}')

            pub_gpsTime.publish(gps_data[1])
            pub_latitude.publish(gps_data[2])
            pub_longitude.publish(gps_data[3])
            pub_altitude.publish(gps_data[4])

    def pub_imu(self):
        pub_roll = rospy.Publisher('roll',String, queue_size=1)
        pub_pitch = rospy.Publisher('pitch',String, queue_size=1)
        pub_yaw = rospy.Publisher('yaw',String, queue_size=1)
        while True:
            imu_data = self.imu_q.get()
            imu_data = imu_data.decode('utf8')
            print(f'imu : {imu_data[1:]}')
            imu_data = imu_data.split(',')

            pub_roll.publish(imu_data[1])
            pub_pitch.publish(imu_data[2])
            pub_yaw.publish(imu_data[3])


    def pub_speed(self):
        pub_speed = rospy.Publisher('speed',String,queue_size=1)
        while True:
            speed_data = self.speed_q.get()
            speed_data = speed_data.decode('utf8')
            print(f'Speed : {speed_data[1:]}')
            pub_speed.publish(speed_data[1:])

    def pub_state(self):
        pub_state = rospy.Publisher('state',String,queue_size=1)
        while True:
            state_data = self.state_q.get()
            state_data = state_data.decode('utf8')
            print(f'State {state_data[1:]}')
            pub_state.publish(state_data[1:])

    def pub_image(self):
        pub_image = rospy.Publisher('img',CompressedImage,queue_size=1)
        msg = CompressedImage()
        msg.format = 'jpeg'
        while True:
            frame = self.image_q.get()
            frame = np.frombuffer(base64.b64decode(frame), dtype='uint8')
            frame = cv2.imdecode(frame, 1)

            msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
            pub_image.publish(msg)

def handler(signum, frame):
    print('**Server Killed**')
    os.system("sudo kill -9 `sudo lsof -t -i:8080`")


# 9502 -> 8080
if __name__ == "__main__":
    signal.signal(signal.SIGTSTP, handler)
    server = ServerSocket('192.168.0.37', 8080)
    