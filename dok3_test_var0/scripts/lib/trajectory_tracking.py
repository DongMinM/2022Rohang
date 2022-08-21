from threading import local
import numpy as np
import copy
import asyncio
import time

import rospy
from lib.trajectory_generator import TrajectoryGenerator
from lib.jps                  import JPS

from std_msgs.msg       import Float32MultiArray
from sensor_msgs.msg    import PointCloud
from geometry_msgs.msg  import Point32
from nav_msgs.msg       import Odometry
from mavsdk.offboard    import VelocityNedYaw,\
                               PositionNedYaw,\
                               VelocityBodyYawspeed

import matplotlib.pyplot as plt     
from matplotlib import colors 
from mpl_toolkits.mplot3d import axes3d, Axes3D
from matplotlib.axes import Axes

class TrajectoryTracker:


    def __init__(self, drone, datahub):

        self.drone = drone 
        self.datahub = datahub

        self.delt = datahub.delt
        self.period = datahub.traj_update_period

        self.generator = TrajectoryGenerator(self.delt)




        
    async def trajectory_tracking(self, x_des, wp, v_mean): # input : NED frame



        n_update = int(self.period/self.delt)       # timesteps for update period

        traj_log = np.zeros((3,1))

        while True:


            if len(wp) != 0 and np.shape(wp)[1] != 0:
                
                
                wp = wp


            else:

                wp = np.array([])




            x_0 = self.datahub.posvel_ned # initial state = current state



            traj,tk = self.generator.generate(x_0,x_des,wp,v_mean)

            self.datahub.traj = traj # for visualizer


            end_traj = time.time()


            if len(traj[0]) > n_update: # if the path spends over update period


                for i in range(n_update):


                    if len(wp) != 0:


                        if np.shape(wp)[1] == 0:

                            wp = np.array([])
                        

                        elif i >= tk[0]-3:

                            print("waypoint passed : ",self.datahub.posvel_ned)
                            wp = np.delete(wp,0,axis=1) # discard the waypoint passed through

                            # break


                    vel = traj[3:,0]
                    pos = traj[:3,0]

                    traj = traj[:,1:]

                    traj_log = np.hstack((traj_log, np.reshape(self.datahub.posvel_ned[:3],(3,1)) ))

                    await self.drone.offboard.set_velocity_ned(
                            VelocityNedYaw(vel[0], vel[1], vel[2], 0.0))
                    # await self.drone.offboard.set_position_ned(
                    #         PositionNedYaw(pos[0], pos[1], pos[2], 0.0))

                    await asyncio.sleep(self.datahub.delt)   
                
                end_tracking = time.time()
                # print(end_tracking-start_tracking)

            else:


                for i in range(len(traj[0])):


                    vel = traj[3:,0]
                    pos = traj[:3,0]

                    traj = traj[:,1:]

                    traj_log = np.hstack((traj_log, np.reshape(self.datahub.posvel_ned[:3],(3,1)) ))

                    await self.drone.offboard.set_velocity_ned(
                            VelocityNedYaw(vel[0], vel[1], vel[2], 0.0))
                    # await self.drone.offboard.set_position_ned(
                    #         PositionNedYaw(pos[0], pos[1], pos[2], 0.0))

                    await asyncio.sleep(self.datahub.delt)     

                break

