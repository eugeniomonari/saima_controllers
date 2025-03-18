#!/usr/bin/env python3

import subprocess
import time
import os

if __name__ == "__main__":
    # USB toggle command
    cmd = "echo 'G7!l,P3j' | sudo -S sh -c \"echo '0' > /sys/bus/usb/devices/6-2/authorized\""
    subprocess.run(cmd, shell=True, check=True)
    time.sleep(4)
    
    # Re-enable the USB device
    cmd = "echo 'G7!l,P3j' | sudo -S sh -c \"echo '1' > /sys/bus/usb/devices/6-2/authorized\""
    subprocess.run(cmd, shell=True, check=True)
    time.sleep(4)
    
    # Source the ROS workspace setup.bash using bash -c
    ros_setup_cmd = "source /home/saima/camera_ws/devel/setup.bash && echo 'ROS setup sourced'"
    subprocess.run(f"bash -c '{ros_setup_cmd}'", shell=True, check=True)
    
    # Add your workspace to the ROS_PACKAGE_PATH
    os.environ['ROS_PACKAGE_PATH'] = '/home/saima/camera_ws/src:' + os.environ['ROS_PACKAGE_PATH']
    
    # Run the ROS node
    subprocess.run(["rosrun", "bobot_inail", "testOrbbecTracking"])
    
