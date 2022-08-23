from lib.postion_estmation   import ArUcoPosEstimator
import numpy as np


class LandingmarkerDetector:

    def __init__(self,datahub, traj):
        self.datahub = datahub
        self.traj = traj

        self.marker = ArUcoPosEstimator()

    def detect(self,ID_aim):

        cnt = 0
        marker_pos = np.zeros((6,))

        while cnt < 30:
            resize_width = np.shape(self.datahub.img_bottom)[1]

            # Save your OpenCV2 image as a jpeg 
            ids,x,y,z = self.marker.run(ID_aim,self.datahub.img_bottom,self.datahub.cam_mtx,self.datahub.dist_coeff,"DICT_5X5_1000",resize_width)
            
            if ids != ID_aim:
                raise
            
            try:
                
                body_pos = self.traj.ned2xyz(self.datahub.posvel_ned[:3])                
                

                marker_pos[0] = body_pos[0] + x[95]*0.002
                marker_pos[1] = body_pos[1] - y[95]*0.002
                # marker_pos[0] = 0
                # marker_pos[1] = 0
                marker_pos[2] = 0

                marker_pos = self.traj.xyz2ned(marker_pos)

                print(marker_pos[:3])

            except:
                cnt += 1


        return marker_pos