#!/usr/bin/env python3
# A simple ROS publisher node in Python

import rospy                                                            # [1]
from geometry_msgs.msg import Twist

class Publisher():                                                      # [2]
    
    def __init__(self):                                                 # [3]
        self.node_name = "circle_move_publisher"
        topic_name = "cmd_vel"

        self.pub = rospy.Publisher(topic_name, Twist, queue_size=10)
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10) # hz                                 # [4]
                
        self.ctrl_c = False                                             # [5]
        rospy.on_shutdown(self.shutdownhook) 
        
        rospy.loginfo(f"The '{self.node_name}' node is active...")      # [6]

    def shutdownhook(self):                                             # [7]
        self.pub.publish(Twist())
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.ctrl_c = True

    def main_loop(self):                                                # [8]
        while not self.ctrl_c:
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.2
            vel_cmd.angular.z = 0.4 
            #publisher_message = f"rospy time is: {rospy.get_time()}"
            self.pub.publish(vel_cmd)
            self.rate.sleep()

if __name__ == '__main__':                                              # [9]
    publisher_instance = Publisher()
    try:
        publisher_instance.main_loop()
    except rospy.ROSInterruptException:
        pass