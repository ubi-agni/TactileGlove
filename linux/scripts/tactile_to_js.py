#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2015, CITEC, Bielefeld University
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
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


import sys
import rospy
import thread
from sensor_msgs.msg import JointState
from tactile_msgs.msg import TactileState

js = JointState()
mapping = []


def receive_cb(msg):

    for channel in msg.sensors:
        # find tactile_glove channel in the tactile state
        if channel.name == "tactile glove":
            # for each joint, extract the correct index of the channel data vector
            for i, name in enumerate(js.name):
                r_in = mapping[name]['range_in']
                r_out = mapping[name]['range_out']
                if mapping[name]['idx'] < len(channel.values):
                    val = channel.values[mapping[name]['idx']]
                    js.position[i] = r_out[0] + (r_out[1] - r_out[0]) * \
                        (float(val) - r_in[0]) / (r_in[1] - r_in[0])
            js.header.stamp = rospy.Time.now()
            if not rospy.is_shutdown():
                pub.publish(js)


if __name__ == '__main__':

    rospy.init_node('tactile_to_js')

    mapping = rospy.get_param('~tactile_to_js_mapping')
    print(mapping)

    for k, v in mapping.iteritems():
        js.name.append(k)
        js.position.append(0.0)
        js.velocity.append(0.0)
        js.effort.append(0.0)

    pub = rospy.Publisher("/joint_states", JointState, queue_size=1)
    sub = rospy.Subscriber("/tactile_states", TactileState, receive_cb, queue_size=1)
    rospy.spin()
