#!/usr/bin/env python

import rospy
from lwr_controllers.msg import Effect
from lwr_controllers.msg import Primitive


# Node example class.
class H3DPublisher():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        self.eff_pub = rospy.Publisher('/lwr/HapiController/effects', Effect, queue_size=1)
        self.prim_pub = rospy.Publisher('/lwr/HapiController/primitives', Primitive, queue_size=1)
        rospy.sleep(1)

        self.effect = Effect()

        self.primitive = Primitive()

    def publish_primitive(self, prim_type, surface_type, surface_param, position, data):
        self.primitive.type = prim_type
        self.primitive.surface = surface_type
        self.primitive.param_surface = surface_param
        self.primitive.position = position
        self.primitive.data = data
        rospy.loginfo(self.primitive)
        self.prim_pub.publish(self.primitive)

    def publish_effect(self, effect_type, data):
        self.effect.type = "Spring"
        self.effect.data = data
        rospy.loginfo(self.effect)
        self.eff_pub.publish(self.effect)

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('pytalker')
    
    try:
        ne = H3DPublisher()
        ne.publish_primitive('Sphere', 'FrictionSurface', [0, 0, 0, 0], [0, 5, 0], [0.02])
        ne.publish_effect('Spring', [0, 5, 0, 0.15])
    except rospy.ROSInterruptException:
        pass
