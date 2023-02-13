#! /usr/bin/env python

import rospy
from assignment_2_2022.srv import goal_rc, goal_rcResponse
import actionlib
import actionlib.msg
import assignment_2_2022.msg

class Service:
    def __init__(self):
        self.goal_cancelled = 0
        self.goal_reached = 0

        self.srv = rospy.Service('goal_service', goal_rc, self.handle_data_request)
        self.sub_result = rospy.Subscriber('/reaching_goal/result', assignment_2_2022.msg.PlanningActionResult, self.result_callback)

    def result_callback(self, msg):
        status = msg.status.status

        if status == 2:
            self.goal_cancelled += 1
        elif status == 3:
            self.goal_reached += 1
    
    def handle_data_request(self, req):
        return goal_rcResponse(self.goal_reached, self.goal_cancelled)

def main():
    rospy.init_node('goal_service')
    goal_service = Service()
    rospy.spin()

if __name__ == "__main__":
    main()
