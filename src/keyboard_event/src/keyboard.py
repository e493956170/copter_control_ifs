#!/usr/bin/env python

import rospy

from std_msgs.msg import String
import sys,select,tty,termios


if __name__ =='__main__':
    try:
        pub=rospy.Publisher('keyboard_event',String,queue_size=10)
        rospy.init_node('keyboard_event')
        print "python node inited."
        old_attr=termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        rate=rospy.Rate(50)

        while not rospy.is_shutdown():
            if select.select([sys.stdin],[],[],0)[0]==[sys.stdin]:
                print(sys.stdin.read(1))
                pub.publish(sys.stdin.read(1))
            rate.sleep()
        termios.tcsetattr(sys.stdin,termios.TCSADRAIN,old_attr)
    except rospy.ROSInitException:
        pass