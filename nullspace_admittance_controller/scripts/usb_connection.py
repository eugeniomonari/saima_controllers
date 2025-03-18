#!/usr/bin/env python3


import subprocess
import time

if __name__ == "__main__":
    cmd = "echo 'G7!l,P3j' | sudo -S sh -c \"echo '0' > /sys/bus/usb/devices/6-2/authorized\""
    subprocess.run(cmd, shell=True, check=True)
    time.sleep(4)
    cmd = "echo 'G7!l,P3j' | sudo -S sh -c \"echo '1' > /sys/bus/usb/devices/6-2/authorized\""
    subprocess.run(cmd, shell=True, check=True)
    #reset_usb()  # Reset USB first
    #run_ros_node()  # Then run the ROS node
    time.sleep(4)
    subprocess.run(["rosrun", "bobot_inail", "testOrbbecTracking"])
    
