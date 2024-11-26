#!/usr/bin/env python3
import rospy
import actionlib
from actionlib_tutorials.msg import FibonacciAction, FibonacciFeedback, FibonacciResult

def execute(goal):
    feedback = FibonacciFeedback()
    result = FibonacciResult()

    a, b = 0, 1
    for i in range(goal.order):
        feedback.sequence.append(a)
        a, b = b, a + b
        server.publish_feedback(feedback)
        rospy.sleep(0.5)

    result.sequence = feedback.sequence
    server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('action_server_node')
    server = actionlib.SimpleActionServer('fibonacci', FibonacciAction, execute, False)
    server.start()
    rospy.spin()
