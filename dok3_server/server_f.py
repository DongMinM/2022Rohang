import socket
import cv2
from cv2 import IMWRITE_JPEG_CHROMA_QUALITY
import numpy as np
import threading
import time
import base64

import rospy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String, Float32MultiArray

import signal
import os
from cv_bridge import CvBridge
from queue import Queue
import sys
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

        self.img_client = ' '

        rospy.init_node('server')
        self.br = CvBridge()

        # Set queues
        self.isAuto_q   = Queue(maxsize=2)
        self.wayPoint_q = Queue(maxsize=2)
        self.gps_q      = Queue(maxsize=2)
        self.imu_q      = Queue(maxsize=2)
        self.speed_q    = Queue(maxsize=2)
        self.state_q    = Queue(maxsize=2)
        self.image_q    = Queue(maxsize=2)
        self.head_q     = Queue(maxsize=2)
        self.jpsmap_q   = Queue(maxsize=2)
        self.traj_q     = Queue(maxsize=2)
        self.battery_q  = Queue(maxsize=2)

        # server socket open ( start )
        threading.Thread(target = self.pub_isAuto).start()
        threading.Thread(target = self.pub_wayPoint).start()
        threading.Thread(target = self.pub_gps).start()
        threading.Thread(target = self.pub_imu).start()
        threading.Thread(target = self.pub_speed).start()
        threading.Thread(target = self.pub_state).start()
        threading.Thread(target = self.pub_head).start()
        threading.Thread(target = self.pub_jps).start()
        threading.Thread(target = self.pub_battery).start()
        threading.Thread(target = self.pub_image).start()
        self.lock = threading.Lock()

        self.socketOpen()

    ## close server socket
    def socketClose(self):
        self.sock.close()
        print(u'Server socket is closed')

    ## open server socket
    def socketOpen(self):

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)           # server socket
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)         # server socket option
        self.sock.bind((self.TCP_IP, self.TCP_PORT))                            # server socket bind
        self.sock.listen(2)                                                     # number of client can be access at the same time (but not work for this code)
        print(u'Server socket is opened')

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
#                    print(1)
                    self.q_clear(self.isAuto_q)
                    self.isAuto_q.put(stringData)

                # wayPoint (w)
                elif header == 119:
                    self.q_clear(self.wayPoint_q)
                    self.wayPoint_q.put(stringData)

                # gps  (g)
                elif header == 103:
                    self.q_clear(self.gps_q)
                    self.gps_q.put(stringData)

                # imu (i)
                elif header == 105:
                    self.q_clear(self.imu_q)
                    self.imu_q.put(stringData)

                # speed (s)
                elif header == 115:
                    self.q_clear(self.speed_q)
                    self.speed_q.put(stringData)

                # state (t)
                elif header == 116:
                    self.q_clear(self.state_q)
                    self.state_q.put(stringData)

                # heading (h)
                elif header == 104:
                    self.q_clear(self.head_q)
                    self.head_q.put(stringData)

                # jps map (j)
                elif header == 106:
                    self.q_clear(self.jpsmap_q)
                    self.jpsmap_q.put(stringData)

                # trajectory (o)
                elif header == 111:
                    self.q_clear(self.traj_q)
                    self.traj_q.put(stringData)

                # battery (b)
                elif header == 98:
                    self.q_clear(self.battery_q)
                    self.battery_q.put(stringData)

                # image
                else:
                    self.img_client = addr
                    self.q_clear(self.image_q)
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

    ## clear queue when queue is not empty -- for realTime
    def q_clear(self,queue):
        self.lock.acquire()
        if not queue.empty():
            queue.get()
        self.lock.release()

    def pub_isAuto(self):
        pub_isAuto = rospy.Publisher('isAuto',String, queue_size=2)
        while True:
            isAuto = self.isAuto_q.get()
            isAuto = isAuto.decode('utf8')
            print(f'ifAuto : {isAuto}')
            pub_isAuto.publish(isAuto[1:])

    def pub_wayPoint(self):
        pub_wayPoint = rospy.Publisher('wayPoint',String, queue_size=2)
        while True:
            wayPoint = self.wayPoint_q.get()
            wayPoint = wayPoint.decode('utf8')
            wayPoint = wayPoint.split(',')
#            print(f'Way Point : {wayPoint[1]}')
            pub_wayPoint.publish(wayPoint[1])

    def pub_gps(self):
        pub_gpsTime = rospy.Publisher('gpsTime',String, queue_size=2)
        pub_latitude = rospy.Publisher('latitude',String, queue_size=2)
        pub_longitude = rospy.Publisher('longitude',String, queue_size=2)
        pub_altitude = rospy.Publisher('altitude',String, queue_size=2)
        pub2saver = rospy.Publisher('gps',Float32MultiArray,queue_size=2)
        gps = Float32MultiArray()
        while True:
            gps_data = self.gps_q.get()
            gps_data = gps_data.decode('utf8')
            gps_data = gps_data.split(',')
#            print(f'Gps :{gps_data[1:]}')

            pub_gpsTime.publish(gps_data[1])
            pub_latitude.publish(gps_data[2])
            pub_longitude.publish(gps_data[3])
            pub_altitude.publish(gps_data[4])
            gps.data = np.array(gps_data[2:]).astype(float)
            pub2saver.publish(gps)

    def pub_imu(self):
        pub_roll = rospy.Publisher('roll',String, queue_size=2)
        pub_pitch = rospy.Publisher('pitch',String, queue_size=2)
        pub_yaw = rospy.Publisher('yaw',String, queue_size=2)
        while True:
            imu_data = self.imu_q.get()
            imu_data = imu_data.decode('utf8')
#            print(f'imu : {imu_data[1:]}')
            imu_data = imu_data.split(',')

            pub_roll.publish(imu_data[1])
            pub_pitch.publish(imu_data[2])
            pub_yaw.publish(imu_data[3])


    def pub_speed(self):
        pub_speed = rospy.Publisher('speed',String,queue_size=2)
        while True:
            speed_data = self.speed_q.get()
            speed_data = speed_data.decode('utf8')
#            print(f'Speed : {speed_data[1:]}')
            pub_speed.publish(speed_data[1:])

    def pub_state(self):
        pub_state = rospy.Publisher('state',String,queue_size=2)
        while True:
            state_data = self.state_q.get()
            state_data = state_data.decode('utf8')
#            print(f'State {state_data[1:]}')
            pub_state.publish(state_data[1:])

    def pub_image(self):
        pub_image = rospy.Publisher('img',CompressedImage,queue_size=2)
        msg = CompressedImage()
        msg.format = 'jpeg'
        while True:
            frame = self.image_q.get()
            frame = np.frombuffer(base64.b64decode(frame), dtype='uint8')
            frame = cv2.imdecode(frame, 1)

            msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
            pub_image.publish(msg)
#            print('123')
    
    def pub_head(self):
        pub_head = rospy.Publisher('heading',String,queue_size=2)
        while True:
            heading = self.head_q.get()
            heading = heading.decode('utf8')
            pub_head.publish(heading[1:])
    
    def pub_battery(self):
        pub_battery = rospy.Publisher('battery',String,queue_size=2)
        while True:
            battery = self.battery_q.get()
            battery = battery.decode('utf8')
            battery = battery[3:5]
            pub_battery.publish(battery)



    def pub_jps(self):
        pub_jpsmap = rospy.Publisher('jps',Image,queue_size=2)
        while True:

            jps_map_byte = self.jpsmap_q.get()[1:]
            jps_map_str = jps_map_byte.decode('utf8')
            l = 0
            x_n = None
            y_n = None
            traj_pnts = []

            if jps_map_str == '[]':
                
                jps_map_image = np.zeros((100,100,3))
                jps_map_image[:,:] = [255,255,255]

            else:

                jps_map_float = np.array(jps_map_str[1:-1].split(',')).astype('float')
                l = int(len(jps_map_float)**0.5)
                jps_map_float = jps_map_float.reshape(l,l)

                jps_map_image = np.zeros((l,l,3))
                # 0 -> 바탕
                x0,y0 = np.where(jps_map_float==0)
                # 1 -> 장애물
                x1,y1 = np.where(jps_map_float==1)
                # 2 -> 드론
                x2,y2 = np.where(jps_map_float==2)
                # WP
                x3,y3 = np.where(jps_map_float==3)
                # GP
                x4,y4 = np.where(jps_map_float==4)
                # trajectory
                traj_pnts = self.get_traj()

                jps_map_image[x0,y0] = [255,255,255]

                jps_map_image[x1,y1] = [0,0,0]
                jps_map_image[x2,y2] = [0,255,0]
                jps_map_image[x3,y3] = [0,0,255]
                jps_map_image[x4,y4] = [255,0,0]
                jps_map_image[0, 0,] = [255,255,255]

                if len(x4) > 0:
                    x_n = int((x4[0]/(l))*600)
                    y_n = int((y4[0]/(l))*600)

            resized_jps = cv2.resize(jps_map_image,(600,600),interpolation=cv2.INTER_AREA)

            if x_n != None:
               cv2.putText(resized_jps,f"WP {int(self.heading)/2}",(y_n-15,x_n-5),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),1)


            if len(traj_pnts) >= 2:
                for pnt in traj_pnts:
                    if pnt[0] > 0 and pnt[0] < 600 and pnt[1] > 0 and pnt[1] < 600:
                        resized_jps[[600-int(pnt[0])],[int(pnt[1])]] = [255,0,0]


            img_encode = np.array(cv2.imencode('.png', resized_jps)[1])
            msg = self.br.cv2_to_imgmsg(img_encode)
            pub_jpsmap.publish(msg)

    def get_traj(self):

            traj_byte = self.traj_q.get()[1:]
            traj_str = traj_byte.decode('utf8')
            traj_float = np.array(traj_str[1:-1].split(',')).astype('float')
            traj_float.astype('int')
            l = int(len(traj_float)/2)
            traj_reshape = traj_float.reshape(2,l).T
            traj_pnt = np.unique(traj_reshape,axis = 0)
            return traj_pnt



def handler(signum, frame):
    print('**Server Killed**')
    os.system("sudo kill -9 `sudo lsof -t -i:8080`")
    sys.exit()

# 9502 -> 8080
if __name__ == "__main__":
    signal.signal(signal.SIGTSTP, handler)
    server = ServerSocket('192.168.0.37', 8080)
