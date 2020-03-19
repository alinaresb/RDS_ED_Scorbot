#!/usr/bin/env python

import roslib; roslib.load_manifest('scorbot_joint')
import rospy, math, time

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def jointTrajectoryCommand():
    # Initialize the node
    rospy.init_node('joint_control')

    print rospy.get_rostime().to_sec()
    while rospy.get_rostime().to_sec() == 0.0:
        time.sleep(0.5)
        print rospy.get_rostime().to_sec()

    pub = rospy.Publisher('/scorbot/trajectory_controller/command', JointTrajectory, queue_size=10)
    jt = JointTrajectory()

    jt.header.stamp = rospy.Time.now()
    jt.header.frame_id = ""

    jt.joint_names.append("base")
    jt.joint_names.append("elbow")
    jt.joint_names.append("gripper_finger_left_joint")
    jt.joint_names.append("gripper_finger_right_joint")
    jt.joint_names.append("pitch")
    jt.joint_names.append("roll")
    jt.joint_names.append("shoulder")
    print jt.joint_names
    n = 1500
    dt = 0.1
    rps = 0.5
    for i in range (n):
        p = JointTrajectoryPoint()
        theta = rps*2.0*math.pi*i*dt
        x1 = -0.5*math.sin(2*theta)
        x2 =  0.5*math.sin(1*theta)

        p.positions.append(x1)
        p.positions.append(x2)
        p.positions.append(x2)
        p.positions.append(x2)
        p.positions.append(x2)
        p.positions.append(x2)
        p.positions.append(x2)
        jt.points.append(p)

        # set duration
        jt.points[i].time_from_start = rospy.Duration.from_sec(dt*10)
        rospy.loginfo("test: angles[%d][%f, %f] - duration: %f",n,x1,x2,dt)

    pub.publish(jt)
    rospy.spin()

if __name__ == '__main__':
    try:
        jointTrajectoryCommand()
    except rospy.ROSInterruptException: pass
