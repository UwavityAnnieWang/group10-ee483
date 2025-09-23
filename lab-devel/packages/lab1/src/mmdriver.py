#!/usr/bin/env python3

#drive one meter straight ninety repeat make square 


import rospy
import os
from duckietown_msgs.msg import WheelsCmdStamped # Import the message for the wheel comm
class Driver():#CHANGE CLASSNAME to the name of your class
    def __init__(self):
        self.veh_name = os.environ['VEHICLE_NAME']

        self.drivePub = rospy.Publisher('/ee483mm15/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)

# USING PARAMETER TO GET THE NAME OF THE VEHICLE
# THIS WILL BE USEFUL TO SPECIFY THE NAME OF THE TOPIC
# INITIALIZE YOUR VARIABLES HERE (SUBSCRIBERS OR PUBLISHERS)

    def drive(self): # CHANGE TO THE NAME OF YOUR FUNCTION
        #input commands here i guess 
        #/ee483mm/wheels_driver_node is the topic being published to 
        self.cmdDrive = WheelsCmdStamped()
        self.cmdDrive.header.stamp = rospy.Time.now()
        self.cmdDrive.vel_left = 0.5
        self.cmdDrive.vel_right= 0.5
        self.drivePub.publish(self.cmdDrive)
        print("Driving the MM " + self.veh_name + " around the block") # Just for testin

        startTime = rospy.Time.now().to_sec()
        funcTime = 2.5

        while rospy.Time.now().to_sec() < float(startTime + funcTime):
                self.drivePub.publish(self.cmdDrive)

    def turnEnd(self):
        self.cmdTurn = WheelsCmdStamped()
        self.cmdTurn.header.stamp = rospy.Time.now()
        self.cmdTurn.vel_left = 0.7
        self.cmdTurn.vel_right = 0
        self.drivePub.publish(self.cmdTurn)
        print("turning")

        startTime = rospy.Time.now().to_sec()
        funcTime = .8

        while rospy.Time.now().to_sec() < float(startTime + funcTime):
                self.drivePub.publish(self.cmdTurn)

    def turnMid(self):
        self.cmdTurn = WheelsCmdStamped()
        self.cmdTurn.header.stamp = rospy.Time.now()
        self.cmdTurn.vel_left = 0.7
        self.cmdTurn.vel_right = 0
        self.drivePub.publish(self.cmdTurn)
        print("turning")

        startTime = rospy.Time.now().to_sec()
        funcTime = .7

        while rospy.Time.now().to_sec() < float(startTime + funcTime):
                self.drivePub.publish(self.cmdTurn)

    def stop(self):
        self.cmdSleep = WheelsCmdStamped()
        self.cmdSleep.header.stamp = rospy.Time.now()
        self.cmdSleep.vel_left = 0
        self.cmdSleep.vel_right = 0
        self.drivePub.publish(self.cmdSleep)
        print("stopped")

        startTime = rospy.Time.now().to_sec()
        funcTime = 2

        while rospy.Time.now().to_sec() < float(startTime + funcTime):
                self.drivePub.publish(self.cmdSleep)


#WRITE THE CODE TO MAKE THE MM GO AROUND THE BLOCK
if __name__ == "__main__": ## The main function which will be called when your python sc
# Initialize the node
    try:
        rospy.init_node('driver_node')
        drive = Driver() # Create obj of the Driver class
        rospy.sleep(3) # Delay to wait enough time for the code to run
        # Keep the line above - you might be able to reduce the delay a bit,
        cycles_max = 4
        cycle = 0
        #startTime = rospy.Time.now().to_sec()
        #print(startTime)
        while not rospy.is_shutdown() and cycle < cycles_max: # Run ros forever - you can change

            drive.drive()

            drive.stop()

            if cycle == 0 or cycle == 3:
                drive.turnEnd()
            else:
                drive.turnMid()

            drive.stop()

            cycle += 1
    except rospy.ROSInterruptException:
        pass