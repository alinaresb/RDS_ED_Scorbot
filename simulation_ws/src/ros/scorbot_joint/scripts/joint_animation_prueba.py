#!/usr/bin/env python

import roslib; roslib.load_manifest('scorbot_joint')
import rospy, math, time

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def jointTrajectoryCommand():

    rospy.init_node('joint_control')
    
    print rospy.get_rostime().to_sec()
    while rospy.get_rostime().to_sec() == 0.0:
        time.sleep(0.1)
        print rospy.get_rostime().to_sec()
    

    pub = rospy.Publisher('/scorbot/trajectory_controller/command', JointTrajectory, queue_size=10)
    jt = JointTrajectory()

    jt.header.stamp.nsecs = 0.0
    jt.header.stamp.secs = 0.0
    jt.header.seq = 0.0
    jt.header.frame_id = ""


    jt.joint_names.append("base")
    jt.joint_names.append("shoulder")
    jt.joint_names.append("elbow")
    jt.joint_names.append("pitch")
    jt.joint_names.append("roll")


    p = JointTrajectoryPoint()
    p.positions.append(10)
    p.positions.append(-10)
    p.positions.append(-10)
    p.positions.append(-5)
    p.positions.append(1)
    p.time_from_start.secs = 0
    p.time_from_start.nsecs = 0
    p.velocities = []
    p.accelerations = []
    p.effort = []


    jt.points.append(p)

    pub.publish(jt)
    rospy.spin()

if __name__ == '__main__':
    try:
        jointTrajectoryCommand()
    except rospy.ROSInterruptException: pass
