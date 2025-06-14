#!/usr/bin/env python3
import rospy
import sys

def main():
    rospy.init_node('mission_control_node', anonymous=True)
    print("In standby mode for 5 seconds", file=sys.stdout, flush=True)
    rospy.sleep(10)
    print("Test task finished successfully", file=sys.stdout, flush=True)

if __name__ == '__main__':
    main()