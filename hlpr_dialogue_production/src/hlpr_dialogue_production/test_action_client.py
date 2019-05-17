#!/usr/bin/env python

# Copyright (c) 2017, Elaine Short, SIM Lab
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# 
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# 
# * Neither the name of the SIM Lab nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
import smach
import smach_ros
import actionlib
import hlpr_dialogue_production.msg as dialogue_msgs
import sys


if __name__ == "__main__":
    rospy.init_node("test_action_client")
    client = actionlib.SimpleActionClient("HLPR_Dialogue",dialogue_msgs.DialogueActAction)
    client.wait_for_server()
    client.send_goal(dialogue_msgs.DialogueActGoal(text_or_key="Hello! My name is Moe."))
    client.wait_for_result()
    client.send_goal(dialogue_msgs.DialogueActGoal(text_or_key="I am a robot!"))
    client.wait_for_result()
    client.send_goal(dialogue_msgs.DialogueActGoal(text_or_key="I was built at the Socially Intelligent Machines lab."))    #client.send_goal(dialogue_msgs.DialogueActGoal(text_or_key="Hello!"))
    #text_or_key = "Hello world"
    #if len(sys.argv) > 1:
    #  text_or_key = sys.argv[1]
    #client.send_goal(dialogue_msgs.DialogueActGoal(text_or_key=text_or_key))
    client.wait_for_result()
    print client.get_result()
