#!/usr/bin/env python3

#drive one meter straight ninety repeat make square 


import rospy
import os
from duckietown_msgs.msg import WheelsCmdStamped # Import the message for the wheel comm
class Driver():#CHANGE CLASSNAME to the name of your class
    def __init__(self):
        self.veh_name = os.environ['VEHICLE_NAME']

        self.drivePub = rospy.Publisher('/ee483mm10/wheels_driver_node', WheelsCmdStamped, queue_size=10)
        self.turnPub = rospy.Publisher('/ee483mm10/wheels_driver_node', WheelsCmdStamped, queue_size=10)

# USING PARAMETER TO GET THE NAME OF THE VEHICLE
# THIS WILL BE USEFUL TO SPECIFY THE NAME OF THE TOPIC
# INITIALIZE YOUR VARIABLES HERE (SUBSCRIBERS OR PUBLISHERS)

    def drive(self): # CHANGE TO THE NAME OF YOUR FUNCTION
        #input commands here i guess 
        #/ee483mm/wheels_driver_node is the topic being published to 
        self.cmdDrive = WheelsCmdStamped()
        self.cmdDrive.header.stamp = rospy.Time.now()
        self.cmdDrive.vel_left = 1
        self.cmdDrive.vel_right= 1
        self.drivePub.publish(self.cmdDrive)
        print("Driving the MM " + self.veh_name + " around the block") # Just for testin

    def turn(self):
        self.cmdTurn = WheelsCmdStamped()
        self.cmdTurn.header.stamp = rospy.Time.now()
        self.cmdTurn.vel_left = 1
        self.cmdTurn.vel_right = 1
        self.turnPub.publish(self.cmdTurn)
        print("turning")

#WRITE THE CODE TO MAKE THE MM GO AROUND THE BLOCK
if __name__ == "__main__": ## The main function which will be called when your python sc
# Initialize the node
    try:
        rospy.init_node('driving')
        drive = Driver() # Create obj of the Driver class
        rospy.sleep(3) # Delay to wait enough time for the code to run
        # Keep the line above - you might be able to reduce the delay a bit,
        cycles_max = 4
        cycle = 0
        while not rospy.is_shutdown() and cycle < cycles_max: # Run ros forever - you can change
            # this as well instead of running forever
            drive.drive()
            rospy.sleep(1)
            drive.turn()
            rospy.sleep(1)
            cycle += 1
    except rospy.ROSInterruptException:
        pass