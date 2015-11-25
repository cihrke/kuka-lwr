#!/usr/bin/env python

import rospy
from lwr_controllers.msg import Effect
from lwr_controllers.msg import Primitive

# Node example class.
class NodeExample():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
	prim_pub = rospy.Publisher('/lwr/HapiController/primitives', Primitive, queue_size=10)
	eff_pub = rospy.Publisher('/lwr/HapiController/effects', Effect, queue_size=10)
	rate = rospy.Rate(1) 

	eff = Effect()
	eff.type = "Spring"
	eff.data = [0,5,0,0.15]
	
	prim = Primitive()
	prim.type = "Sphere"
	prim.surface = "FrictionSurface"
	prim.param_surface = [0,0,0,0]
	prim.position = [0,5,0]
	prim.data = [0.02]

	while not rospy.is_shutdown():
	    rospy.loginfo(prim)
	    prim_pub.publish(prim)
	    rospy.loginfo(eff)
	    eff_pub.publish(eff)
	    rate.sleep()

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('pytalker')
    
    try:
        ne = NodeExample()
    except rospy.ROSInterruptException:
        pass
