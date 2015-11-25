#!/usr/bin/env python

import rospy
from lwr_controllers import Effect
from lwr_controllers import Primitive


def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('primitives', anonymous=True)
    rate = rospy.Rate(1) # 10hz

    Effect eff

    eff.type = "Spring"
    eff.data = [0,0,0,0.05]
 
    Primitive prim
    prim.type = "Sphere"
    prim.pos = [0,0,0]
    prim.data = 0.05

    while not rospy.is_shutdown():
        rospy.loginfo(prim)
        pub.publish(prim)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
