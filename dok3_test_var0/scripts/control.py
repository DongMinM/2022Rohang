import numpy as np
import asyncio
import mavsdk
import time
from lib.trajectory_tracking import TrajectoryTracker
from lib.postion_estmation   import ArUcoPosEstimator
from mavsdk.offboard    import OffboardError,\
                               VelocityNedYaw,\
                               PositionNedYaw,\
                               VelocityBodyYawspeed


class Controller:

    def __init__(self, drone, datahub):

        self.drone = drone
        self.datahub = datahub
        self.delt = datahub.delt

        self.traj = TrajectoryTracker(self.drone,self.datahub)
        self.marker = ArUcoPosEstimator()




    async def arm(self):

        print("Action : armed")
        await self.drone.action.arm()




    async def disarm(self):

        print("Action : disarmed")
        await self.drone.action.disarm()




    async def takeoff(self):

        print("Action : takeoff ...")


        target__pos = self.datahub.waypoints + np.reshape(self.datahub.posvel_ned[:3], (3,1))

        destination = np.vstack((target__pos,np.zeros((3,1))))
        destination = np.reshape(destination, (6,))
        wp = np.array([])

        await self.traj.trajectory_tracking(destination,wp,2)

        self.datahub.state = "Hold"
        self.datahub.action = "hold"






    async def hold(self):

        print("Action : hold ...")
        while self.datahub.state == "Hold":

            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            
            await asyncio.sleep(0.1)




    async def wp_guidance(self):

        print("Action : waypoint guidance ...")
        dest_position = np.reshape(self.datahub.waypoints[:,-1],(3,1))
        destination = np.vstack((dest_position,np.zeros((3,1))))
        destination = np.reshape(destination, (6,))

        if len(self.datahub.waypoints[0]) == 1:

            wp = np.array([])

        else:

            wp = self.datahub.waypoints[:,:-1]

        await self.traj.trajectory_tracking(destination,wp,self.datahub.v_mean)

        self.datahub.waypoints = None
        self.datahub.mission_input = None
        self.datahub.state = "Hold"
        self.datahub.action = "hold"




    async def park(self):

        print("Action : park ...")
        height = self.datahub.posvel_ned[2]

        marker_pos_from_cam_center = np.array([0,0,0])
        resize_width = resize_width = np.shape(self.datahub.img_bottom)[1]
        if height > 5:
            marker_pos_from_cam_center = self.marker.run(90,self.datahub.img_bottom,self.datahub.cam_max,self.datahub.dist_coeff,"DICT_5X5_1000",resize_width)

        elif height <= 5:
            marker_pos_from_cam_center = self.marker.run(90,self.datahub.img_bottom,self.datahub.cam_max,self.datahub.dist_coeff,"DICT_5X5_1000",resize_width)


        self.datahub.state = "Land"
        self.datahub.action = "land"
        self.datahub.mission_input = None




    async def land(self):

        print("Action : land")
        await self.drone.action.land()




    async def offboard_start(self):

        print("Offboard Starting : Setting initial setpoint ...")

        await self.drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, 0.0, 0.0))
        
        await self.drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, 0.0, 0.0))

        await self.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

        print("Offboard Start")

        try:
            await self.drone.offboard.start()

            return True

        except OffboardError as error:

            print(f"Starting offboard mode failed with error code: \
                {error._result.result}")

            return False
