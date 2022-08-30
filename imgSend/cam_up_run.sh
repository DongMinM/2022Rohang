(roslaunch pointgrey_camera_driver camera_up.launch&
rosrun dynamic_reconfigure dynparam set /camera_up/camera_nodelet "{'video_mode':'format7_mode2','frame_rate':20,'shutter_speed':0}");




while true
do
    sleep 1
done

