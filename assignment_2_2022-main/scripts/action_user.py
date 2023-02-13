#!/usr/bin/env python
import rospy
import actionlib
import actionlib.msg
import assignment_2_2022.msg
from std_srvs.srv import *
import sys
import select
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Twist
from assignment_2_2022.msg import Posxy_velxy
from colorama import Fore, Style, init

init()

# callback function for the subscriber
def handle_odometry_message(msg):
    global publisher
    # Extract position information
    pos = msg.pose.pose.position
    # Extract velocity information
    velocity = msg.twist.twist.linear
    # Create a custom message
    posxy_velxy = Posxy_velxy()
    # Set the parameters of the custom message
    posxy_velxy.msg_pos_x = pos.x
    posxy_velxy.msg_pos_y = pos.y
    posxy_velxy.msg_vel_x = velocity.x
    posxy_velxy.msg_vel_y = velocity.y
    # Publish the custom message
    publisher.publish(posxy_velxy)

def action_client():
    # Create the action client
    client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2022.msg.PlanningAction)
    # Wait for the action server to start
    client.wait_for_server()
    
    goal_status = False
	
    while not rospy.is_shutdown():
        # Prompt the user for target position
        print(Fore.GREEN + "Enter target position or type 'c' to cancel:")
        x_input = input(Fore.MAGENTA + "X position of target: ")
        y_input = input(Fore.MAGENTA + "Y position of target: ")
        
 	# Check if the user wants to cancel the goal
        if x_input == "c" or y_input == "c":
            # Cancel the current goal
            client.cancel_goal()
            goal_status = False
        else:
            # Convert input to float
            x = float(x_input)
            y = float(y_input)
            # Create a new goal
            goal = assignment_2_2022.msg.PlanningGoal()
            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
					
            # Send the goal to the action server
            client.send_goal(goal)
            goal_status = True


def main():
    # Initialize the ROS node
    rospy.init_node('action_user')
    global publisher
    # Create a publisher for the custom message
    publisher = rospy.Publisher("/posxy_velxy", Posxy_velxy, queue_size = 1)
    # Subscribe to the odometry topic
    rospy.Subscriber("/odom", Odometry, handle_odometry_message)
    # Call the action client function
    action_client()

if __name__ == '__main__':
    main
