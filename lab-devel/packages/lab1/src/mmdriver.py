#!/usr/bin/env python3
import rospy
import os
from duckietown_msgs.msg import WheelsCmdStamped # Import the message for the wheel comm
class Driver():#CHANGE CLASSNAME to the name of your class
    def __init__(self):
        self.veh_name = os.environ['VEHICLE_NAME']
# USING PARAMETER TO GET THE NAME OF THE VEHICLE
# THIS WILL BE USEFUL TO SPECIFY THE NAME OF THE TOPIC
# INITIALIZE YOUR VARIABLES HERE (SUBSCRIBERS OR PUBLISHERS)

    def drive(self): # CHANGE TO THE NAME OF YOUR FUNCTION
        print("Driving the MM " + self.veh_name + " around the block") # Just for testin
#WRITE THE CODE TO MAKE THE MM GO AROUND THE BLOCK
if __name__ == "__main__": ## The main function which will be called when your python sc
# Initialize the node
    try:
        rospy.init_node('driving')
        drive = Driver() # Create obj of the Driver class
        rospy.sleep(3) # Delay to wait enough time for the code to run
        # Keep the line above - you might be able to reduce the delay a bit,
        while not rospy.is_shutdown(): # Run ros forever - you can change
            # this as well instead of running forever
         drive.drive() # calling your node function
    except rospy.ROSInterruptException:
        pass