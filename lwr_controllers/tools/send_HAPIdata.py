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

    def publish_primitive(self, prim_type, surface_type, position, surface_param=None, end_position=None, third_vertex=None, length=None, radius=None):
        self.primitive.type = prim_type
        self.primitive.surface = surface_type
        self.primitive.position = position
        if surface_param is not None:
            self.primitive.surface_parameters = surface_param
        if end_position is not None:
            self.primitive.end_position = end_position
        if third_vertex is not None:
            self.primitive.third_vertex = third_vertex
        if length is not None:
            self.primitive.length = length
        if radius is not None:
            self.primitive.radius = radius
        
        rospy.loginfo(self.primitive)
        self.prim_pub.publish(self.primitive)

    def publish_effect(self, effect_type, position, torque=None, spring_constant=None, damping=None):
        self.effect.type = effect_type
        self.effect.position = position
        if torque is not None:
            self.effect.torque = torque
        if spring_constant is not None:
            self.effect.spring_constant = spring_constant
        if damping is not None:
            self.effect.damping = damping
        
        rospy.loginfo(self.effect)
        self.eff_pub.publish(self.effect)

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('pytalker')
    
    try:
        ne = H3DPublisher()
        ne.publish_primitive('Sphere', 'FrictionSurface', [0, 0, 0, 0], [0, 0.5, 0], radius=0.02)
        ne.publish_effect('Spring', [0, 0.5, 0], spring_constant=0.15)
    except rospy.ROSInterruptException:
        pass
