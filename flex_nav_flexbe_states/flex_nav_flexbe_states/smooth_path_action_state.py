#!/usr/bin/env python

###############################################################################
#  Copyright (c) 2023-2024
#  Capable Humanitarian Robotics and Intelligent Systems Lab (CHRISLab)
#  Christopher Newport University
#
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#    1. Redistributions of source code must retain the above copyright notice,
#       this list of conditions and the following disclaimer.
#
#    2. Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#
#    3. Neither the name of the copyright holder nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
#       THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#       "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#       LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#       FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#       COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#       INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#       BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#       LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#       CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#       LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
#       WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#       POSSIBILITY OF SUCH DAMAGE.
###############################################################################

from unittest import result
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from nav2_msgs.action import SmoothPath
from builtin_interfaces.msg import Duration


class SmoothPathActionState(EventState):
    """
    Smooth a plan if one is available.

    smoother_server must be declared as a lifecycle node at launch.
    Start the smoother_server node from the launch file or the terminal with 'ros2 run nav2_smoother smoother_server'

    -- smoother_topic   String      The smoother to talk to

    ># path             Path        Path to be smoothed

    #> smooth_path      Path        The smoothed path

    <= smoothed         Successfully smoothed a path
    <= empty            The plan is empty
    <= failed           Failed to create a plan

    """

    def __init__(self, smoother_topic):
        """Initialize state."""
        super().__init__(outcomes=['smoothed', 'failed'], input_keys=['plan'], output_keys=['smooth_path'])

        ProxyActionClient.initialize(SmoothPathActionState._node)

        self._action_topic = smoother_topic
        self._client = ProxyActionClient({self._action_topic: SmoothPath})
        self._return = None

    def execute(self, userdata):
        """Execute each tic of state machine."""
        if self._client.has_result(self._action_topic):
            result = self._client.get_result(self._action_topic)

            ProxyActionClient._result[self._action_topic] = None

            if result.was_completed is True:
                userdata.smooth_path = result.path
                self._return = 'smoothed'
            else:
                self._return = 'failed'

        return self._return

    def on_enter(self, userdata):
        """Execute on entering state."""
        self._return = None
        result = SmoothPath.Goal(path=userdata.plan)
        result.max_smoothing_duration = Duration()
        result.max_smoothing_duration.sec = 1

        try:
            Logger.loginfo('%s    Requesting a plan' % (self.name))
            self._client.send_goal(self._action_topic, result)
        except Exception as e:
            Logger.logwarn('%s    Failed to send plan request: %s' % (self.name, str(e)))
            userdata.smooth_path = None
            return 'failed'

    def on_exit(self, userdata):
        """Execute when exiting state."""
        if self._action_topic in ProxyActionClient._result:
            ProxyActionClient._result[self._action_topic] = None

        if self._client.is_active(self._action_topic):
            Logger.logerr('%s    Canceling active goal' % (self.name))
            self._client.cancel(self._action_topic)
