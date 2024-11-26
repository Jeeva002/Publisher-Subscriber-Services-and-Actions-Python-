#!/usr/bin/env python3
import rospy
from std_srvs.srv import Empty

def service_client():
    rospy.wait_for_service('my_service')
    try:
        service = rospy.ServiceProxy('my_service', Empty)
        service()
        rospy.loginfo("Service call successful.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    service_client()
