#!/usr/bin/env python

#robot.py implementation goes here

#Kevin Kohlhase
#Amy Han

import rospy



class Robot():

    def __init__(self):
        """Setup ROS things"""
        
        rospy.init_node('robot')
       

        self.simComplete_publisher = rospy.Publisher(
			"/sim_complete",
			Bool,
			queue_size = 10
		)
        
        rospy.sleep(1)

        #call Astar here


        #call MDP here

        rospy.sleep(1)

        self.simComplete_publisher.publish(True)

        rospy.sleep(1)

        rospy.signal_shutdown("shutting down")



     



if __name__ == '__main__':
     rb = Robot()

        



