#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, National Institute of Advanced Industrial Science and Technology (AIST)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of National Institute of Advanced Industrial
#    Science and Technology (AIST) nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Toshio Ueshiba
#
import rospy
from geometry_msgs.msg   import Point
from aist_lenti_mark.msg import QuadStamped

######################################################################
#  class QuadGenerator                                               #
######################################################################
class QuadGenerator(object):
    def __init__(self):
        super(QuadGenerator, self).__init__()

        self._quad = QuadStamped()
        self._quad.header.frame_id = rospy.get_param('~frame', 'marker_frame')
        self._quad.top_left        = Point(-1.0, -0.5, 0)
        self._quad.top_right       = Point( 1.0, -1.0, 0)
        self._quad.bottom_left     = Point(-1.0,  1.0, 0)
        self._quad.bottom_right    = Point( 1.0,  1.0, 0)

        self._quad_pub = rospy.Publisher('~quad', QuadStamped, queue_size=1)

    def run(self):
        rate = rospy.Rate(30.0)
        while not rospy.is_shutdown():
            self._quad.header.stamp = rospy.Time.now()
            self._quad_pub.publish(self._quad)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('quad_generator', anonymous=True)

    quad_generator = QuadGenerator()
    quad_generator.run()