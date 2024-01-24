#!/usr/bin/env python3

import rospy
import actionlib

# Import all the necessary ROS message types:
from tuos_ros_msgs.msg import SearchAction, SearchFeedback, SearchResult, SearchGoal

# Import the tb3 modules from tb3.py
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan

# Import some other useful Python Modules
from math import sqrt, pow

class SearchActionServer():
    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):
        ## TODO: create a "simple action server" with a callback function, and start it...
        node_name = "/search_action_server"
        self.actionserver = actionlib.SimpleActionServer(node_name, SearchAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()

        # pull in some useful publisher/subscriber functions from the tb3.py module:
        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()

        rospy.loginfo("The 'Search Action Server' is active...")

        # The action's "callback function":
    def action_server_launcher(self, goal: SearchGoal):
        rate = rospy.Rate(10)

        ## TODO: Implement some checks on the "goal" input parameter(s)
        success = True
        if goal.approach_distance > self.tb3_lidar.min_distance:
            print(f"Invalid approach distance! Approach distance should be less than {self.tb3_lidar.min_distance:.2f}")          
            success = False
        if goal.fwd_velocity > 0.26:
            print(f"Invalid fwd velocity! Fwd velocity should be less than 0.26 m/s")          
            success = False
        if not success:
            ## TODO: abort the action server if an invalid goal has been requested...
            self.result.total_distance_travelled = 0.0
            self.result.closest_object_distance = self.tb3_lidar.min_distance
            self.result.closest_object_angle = self.tb3_lidar.closest_object_position
            self.actionserver.set_aborted(self.result)            
            return
        
        ## TODO: Print a message to indicate that the requested goal was valid
        print(f"Robot will be requested to move to {goal.approach_distance:.2f}m from obj at {goal.fwd_velocity:.2f}m/s")

        # Get the robot's current odometry from the Tb3Odometry() class:
        self.posx0 = self.tb3_odom.posx
        self.posy0 = self.tb3_odom.posy
        # Get information about objects up ahead from the Tb3LaserScan() class:
        self.closest_object = self.tb3_lidar.min_distance
        self.closest_object_location = self.tb3_lidar.closest_object_position

        print(f"Closest Obj: {self.closest_object:.2f} m @ {self.closest_object_location} degrees")

        ## TODO: set the robot's forward velocity (as specified in the "goal")...
        self.vel_controller.set_move_cmd(linear=goal.fwd_velocity)

        ## TODO: establish a conditional statement so that the  
        ## while loop continues as long as the distance to the closest object
        ## ahead of the robot is always greater than the "approach distance"
        ## (as specified in the "goal")...
        while goal.approach_distance < self.closest_object:
            # update LaserScan data:
            self.closest_object = self.tb3_lidar.min_distance
            self.closest_object_location = self.tb3_lidar.closest_object_position

            ## TODO: publish a velocity command to make the robot start moving 
            self.vel_controller.publish()

            # check if there has been a request to cancel the action mid-way through:
            if self.actionserver.is_preempt_requested():
                ## TODO: take appropriate action if the action is cancelled (peempted)...
                rospy.loginfo("Cancelling robot search request.")

                self.vel_controller.stop()
                self.result.total_distance_travelled = self.distance
                self.result.closest_object_angle = self.closest_object_location
                self.result.closest_object_distance = self.closest_object

                self.actionserver.set_preempted(self.result)                
                success = False
                # exit the loop:
                break

            # determine how far the robot has travelled so far:
            self.distance = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))

            ## TODO: update all feedback message values and publish a feedback message:
            self.feedback.current_distance_travelled = self.distance

            ## TODO: update all result parameters:
            self.result.total_distance_travelled = self.distance
            self.result.closest_object_angle = self.closest_object_location
            self.result.closest_object_distance = self.closest_object

            rate.sleep()
        
        if success:
            rospy.loginfo("approach completed successfully.")
            ## TODO: Set the action server to "succeeded" and stop the robot..
            self.actionserver.set_succeeded(self.result)
            self.vel_controller.stop()

if __name__ == '__main__':
    rospy.init_node("search_action_server")
    SearchActionServer()
    rospy.spin()