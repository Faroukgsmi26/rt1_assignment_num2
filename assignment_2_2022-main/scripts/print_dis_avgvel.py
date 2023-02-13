import rospy
import math
import time
from assignment_2_2022.msg import Posxy_velxy
from colorama import init
init()
from colorama import Fore, Back, Style

class PrintInformation:
    def __init__(self):
        # Read the frequency for printing information from the parameters
        self.frequency = rospy.get_param("frequency")

        # Record the time the information was last printed
        self.last_print_time = 0

        # Subscribe to the topic for the position and velocity information
        self.subscriber = rospy.Subscriber("/posxy_velxy", Posxy_velxy, self.callback)

    def callback(self, msg):
        # Compute the period between printouts in milliseconds
        period = (1.0 / self.frequency) * 1000

        # Get the current time in milliseconds
        current_time = time.time() * 1000

        # Check if the time since the last printout is greater than the specified period
        if current_time - self.last_print_time > period:
            # Read the desired position from the parameters
            target_x = rospy.get_param("des_pos_x")
            target_y = rospy.get_param("des_pos_y")

            # Get the current position from the message
            robot_x = msg.msg_pos_x
            robot_y = msg.msg_pos_y

            # Compute the distance from the desired position
            distance = round(math.dist([target_x, target_y], [robot_x, robot_y]), 2)

            # Get the velocity information from the message
            velocity_x = msg.msg_vel_x
            velocity_y = msg.msg_vel_y

            # Compute the average speed of the robot
            average_speed = round(math.sqrt(velocity_x**2 + velocity_y**2), 2)

            # Log the distance and average speed information
            rospy.loginfo(Fore.GREEN + "Distance to target: %s [m]", distance)
            rospy.loginfo(Fore.MAGENTA + "Robot average speed: %s [m/s]", average_speed)

            # Update the last print time
            self.last_print_time = current_time

def main():
    # Initialize the ROS node
    rospy.init_node('print_distance_and_speed')

    # Create an instance of the `PrintInformation` class
    print_info = PrintInformation()

    # Wait for messages
    rospy.spin()

if __name__ == "__main__":
    main()
