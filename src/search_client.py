#!/usr/bin/env python3

import rospy
import actionlib
import time

# Import all the necessary ROS message types:
from tuos_ros_msgs.msg import SearchAction, SearchFeedback, SearchResult, SearchGoal

class SearchActionClient():
    goal = SearchGoal

    def __init__(self):
        node_name = "search_action_client"
        action_server_name = "/search_action_server"
        
        rospy.init_node(node_name)
        self.rate = rospy.Rate(1)

        self.client = actionlib.SimpleActionClient(action_server_name, 
                    SearchAction)
        self.client.wait_for_server()
        rospy.on_shutdown(self.shutdown_ops)

    def shutdown_ops(self):
        if not self.action_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            self.client.cancel_goal()
            rospy.logwarn("Goal Cancelled...")
        
        # get the result:
        rospy.sleep(1) # wait for the result to come in
        print("RESULT:")
        print(f"  * Action State = {self.client.get_state()}")
        # print(f"  * {self.captured_images} image(s) saved to {self.client.get_result()}")

        self.ctrl_c = True

    def send_goal(self, approach_distance, approach_vel):
        self.goal.approach_distance = approach_distance
        self.goal.fwd_velocity = approach_vel
        
        # send the goal to the action server:
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)      

    def print_result(self, result:SearchResult):
        print(f"RESULT: Action State = {self.client.get_state()}")
        print(f"RESULT: Distance travelled - {result.total_distance_travelled}m")
        print(f"RESULT: Closest Obj: {result.closest_object_distance:.2f} m @ {result.closest_object_angle} degrees")

    def feedback_callback(self, feedback_data: SearchFeedback):
        print(f"FEEDBACK: Current distance travelled: {feedback_data.current_distance_travelled:.2f} m.")
        #if(self.captured_images >= 5):
        #    self.client.cancel_goal()

    def main_loop(self):
        time.sleep(5) #wait for robot to be initialised
        self.send_goal(approach_distance= 0.5, approach_vel= 0.2)
        i = 1
        print("While we're waiting, let's do our seven-times tables...")
        while self.client.get_state() < 2:
            print(f"STATE: Current state code is {self.client.get_state()}")
            print(f"TIMES TABLES: {i} times 7 is {i*7}")
            i += 1
            self.rate.sleep()
        self.action_complete = True
        self.print_result(self.client.get_result())
        

if __name__ == '__main__':
    client = SearchActionClient()
    try:
        client.main_loop()
    except rospy.ROSInterruptException:
        pass
