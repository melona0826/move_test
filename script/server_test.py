#!/usr/bin/env python
#
# Copyright 2015, 2016 Thomas Timm Andersen (original version)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from math import pi

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

def deg_2_rad(val) :
    return val * pi /180

p_inital = [-0.04483205476869756, -0.733215634022848, -2.0507729689227503, -1.6661017576800745, 1.5725396871566772, -0.004635636006490529]
p2 = [-0.5244196097003382, -0.9306796232806605, -1.9886143843280237, -1.5470336119281214, 1.6937694549560547, -0.4688757101642054]
p3 = [-0.8407376448260706, -1.1964519659625452, -1.85187274614443, -1.468623463307516, 1.763596773147583, -0.7817509810077112]
p4 = [-0.8403776327716272, -1.2418330351458948, -1.5807822386371058, -1.694381062184469, 1.763309121131897, -0.7816074530230921]
p5 = [-0.6450203100787562, -1.0765708128558558, -1.6781514326678675, -1.728870693837301, 1.7219785451889038, -0.5876944700824183]
p6 = [-0.3020437399493616, -0.9095323721515101, -1.745720688496725, -1.791403595601217, 1.6370047330856323, -0.25304919878114873]
pos_list = [p_inital , p2 , p3 , p4 , p5 , p6 ]


count = 0
client = None

def move1(req):
    global joints_pos
    global count
    if req.data :
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = JOINT_NAMES
        try:
            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos = joint_states.position
            g.trajectory.points = [
                JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
                JointTrajectoryPoint(positions=pos_list[count], velocities=[0]*6, time_from_start=rospy.Duration(2.0))]
            client.send_goal(g)
            client.wait_for_result()
        except KeyboardInterrupt:
            client.cancel_goal()
            raise
        except:
            raise

        count = count + 1

        return [True , 'Success']

def main():
    global client
    rospy.init_node("test_move", anonymous=True, disable_signals=True)
    client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
    print "Waiting for server..."
    client.wait_for_server()
    print "Connected to server"
    parameters = rospy.get_param(None)
    index = str(parameters).find('prefix')
    if (index > 0):
        prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
        for i, name in enumerate(JOINT_NAMES):
            JOINT_NAMES[i] = prefix + name

    print("Ready !")

    srv = rospy.Service('move_next_position' , SetBool, move1)

    rospy.spin()

if __name__ == '__main__':
    main()
