#!/usr/bin/env python3
import rospy
import actionlib
from actionlib_tutorials.msg import FibonacciAction, FibonacciGoal

def action_client():
    client = actionlib.SimpleActionClient('fibonacci', FibonacciAction)
    client.wait_for_server()

    goal = FibonacciGoal(order=10)
    client.send_goal(goal)
    client.wait_for_result()

    result = client.get_result()
    rospy.loginfo("Fibonacci sequence: %s", result.sequence)

if __name__ == '__main__':
    rospy.init_node('action_client_node')
    action_client()
