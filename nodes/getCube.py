#!/usr/bin/env python3.8
from __future__ import print_function
from cmath import pi

import roslib
roslib.load_manifest('strategy')
import sys
import rospy
import tf
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Quaternion, Twist
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import Int32, Float32

def main(args):
    #definitions
    vel_back = Twist()
    vel_back.linear.x = -0.1
    rospy.init_node('mission_controller', anonymous=False)
    listener = tf.TransformListener()
    pose_pub = rospy.Publisher("/planner/move_base/goal", MoveBaseActionGoal, queue_size=1, latch=True)
    vel_pub = rospy.Publisher("/planner/cmd_vel", Twist, queue_size=1, latch=True)
    claw_pub = rospy.Publisher("/claw_pos", Float32, queue_size=1)
    arm_pub  = rospy.Publisher("/motHeight", Int32, queue_size=1)
    listener.waitForTransform('/map', '/aruco_cube_2', rospy.Time(), rospy.Duration(4.0))
    #obtem a coordenada do cubo referente ao mapa
    trans, rot = listener.lookupTransform('/map', '/aruco_cube_2', rospy.Time())
    print(trans)

    claw_pub.publish(Float32(120)) #abre a garra

    #move robô 20cm do centro do cubo
    movimento = MoveBaseActionGoal()
    movimento.goal.target_pose.header.frame_id = "map"
    movimento.goal.target_pose.pose.position.x = trans[0] - 0.3
    movimento.goal.target_pose.pose.position.y = trans[1]
    #movimento.goal.target_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,pi))
    movimento.goal.target_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,0))
    pose_pub.publish(movimento)
    rospy.wait_for_message("/planner/move_base/result", MoveBaseActionResult)
    rospy.sleep(rospy.Duration(1))
    movimento.goal.target_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,pi))
    pose_pub.publish(movimento)
    #espera chegar na GOAL
    rospy.wait_for_message("/planner/move_base/result", MoveBaseActionResult)
    rospy.sleep(rospy.Duration(1))

    movimento.goal.target_pose.pose.position.x = trans[0] - 0.1
    pose_pub.publish(movimento)
    rospy.wait_for_message("/planner/move_base/result", MoveBaseActionResult)
    rospy.sleep(rospy.Duration(3))
    """
    #vai pra trás burramente
    vel_pub.publish(vel_back)
    #aguarda 1s
    rospy.sleep(rospy.Duration(1))
    #para de andar
    vel_pub.publish(Twist())
    """
    #fecha e levanta a garra
    claw_pub.publish(Float32(0))
    rospy.sleep(rospy.Duration(1))
    arm_pub.publish(Int32(3178))
    rospy.sleep(rospy.Duration(2))
    arm_pub.publish(Int32(-3178))

if __name__ == '__main__':
    main(sys.argv)
