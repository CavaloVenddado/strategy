#!/usr/bin/env python3.8
from __future__ import print_function

import roslib
roslib.load_manifest('strategy')
import sys
import rospy
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import PolygonStamped, Point32
from std_srvs.srv import SetBool, SetBoolResponse

def pub_limits():
    obstacle_msg = ObstacleArrayMsg() 
    obstacle_msg.header.stamp = rospy.Time.now()
    obstacle_msg.header.frame_id = "map"
    obstacle_msg.obstacles.append(ObstacleMsg())
    #Obstaculo Dinamico Esquerdo
    obstacle_msg.obstacles[0].id = 0
    obstacle_msg.obstacles[0].polygon.points = [Point32(), Point32(), Point32(), Point32()]
    #inferior direito
    obstacle_msg.obstacles[0].polygon.points[0].x = -0.02
    obstacle_msg.obstacles[0].polygon.points[0].y = 0.3
    obstacle_msg.obstacles[0].polygon.points[0].z = 0
    #inferior esquerdo
    obstacle_msg.obstacles[0].polygon.points[1].x = -0.02
    obstacle_msg.obstacles[0].polygon.points[1].y = 0.58
    obstacle_msg.obstacles[0].polygon.points[1].z = 0
    #superior esquerdo
    obstacle_msg.obstacles[0].polygon.points[2].x = 0.34
    obstacle_msg.obstacles[0].polygon.points[2].y = 0.58
    obstacle_msg.obstacles[0].polygon.points[2].z = 0
    #superior direito
    obstacle_msg.obstacles[0].polygon.points[3].x = 0.34
    obstacle_msg.obstacles[0].polygon.points[3].y = 0.30
    obstacle_msg.obstacles[0].polygon.points[3].z = 0
    #ponto central dentro do cubo
    obstacle_msg.obstacles.append(ObstacleMsg())
    obstacle_msg.obstacles[1].id = 1
    obstacle_msg.obstacles[1].polygon.points = [Point32()]#[0.16, 0.48, 0]
    obstacle_msg.obstacles[1].polygon.points[0].x = 0.16
    obstacle_msg.obstacles[1].polygon.points[0].y = 0.48
    obstacle_msg.obstacles[1].polygon.points[0].z = 0
    obstacle_msg.obstacles[1].radius = 0.3

    #Obstaculo Dinamico Direito
    obstacle_msg.obstacles.append(ObstacleMsg())
    obstacle_msg.obstacles[2].id = 2
    obstacle_msg.obstacles[2].polygon.points = [Point32(), Point32(), Point32(), Point32()]
    #inferior direito
    obstacle_msg.obstacles[2].polygon.points[0].x = -0.02
    obstacle_msg.obstacles[2].polygon.points[0].y = -0.58
    obstacle_msg.obstacles[2].polygon.points[0].z = 0
    #inferior esquerdo
    obstacle_msg.obstacles[2].polygon.points[1].x = -0.02
    obstacle_msg.obstacles[2].polygon.points[1].y = -0.3
    obstacle_msg.obstacles[2].polygon.points[1].z = 0
    #superior esquerdo
    obstacle_msg.obstacles[2].polygon.points[2].x = 0.34
    obstacle_msg.obstacles[2].polygon.points[2].y = -0.30
    obstacle_msg.obstacles[2].polygon.points[2].z = 0
    #superior direito
    obstacle_msg.obstacles[2].polygon.points[3].x = 0.34
    obstacle_msg.obstacles[2].polygon.points[3].y = -0.58
    obstacle_msg.obstacles[2].polygon.points[3].z = 0
    #ponto central dentro do cubo
    obstacle_msg.obstacles.append(ObstacleMsg())
    obstacle_msg.obstacles[3].id = 3
    obstacle_msg.obstacles[3].polygon.points = [Point32()] #[0.16, -0.48, 0]
    obstacle_msg.obstacles[3].polygon.points[0].x = 0.16
    obstacle_msg.obstacles[3].polygon.points[0].y = -0.48
    obstacle_msg.obstacles[3].polygon.points[0].z = 0
    obstacle_msg.obstacles[3].radius = 0.3

    #Linha Cores
    obstacle_msg.obstacles.append(ObstacleMsg())
    obstacle_msg.obstacles[4].id = 4
    obstacle_msg.obstacles[4].polygon.points = [Point32(), Point32()]
    #Ponto Esquerdo
    obstacle_msg.obstacles[4].polygon.points[0].x = -0.98
    obstacle_msg.obstacles[4].polygon.points[0].y = 1.13
    obstacle_msg.obstacles[4].polygon.points[0].z = 0
    obstacle_msg.obstacles[4].radius = 0.12
    #Ponto Direito
    obstacle_msg.obstacles[4].polygon.points[1].x = -0.98
    obstacle_msg.obstacles[4].polygon.points[1].y = -1.13
    obstacle_msg.obstacles[4].polygon.points[1].z = 0
    obstacle_msg.obstacles[4].radius = 0.12
    pub.publish(obstacle_msg)

def pub_empty():
    pub.publish(ObstacleArrayMsg())

def srvCallback(data):
    response = SetBoolResponse()
    response.success = True
    if data.data:
        pub_limits()
        response.message = "limits activated!"
    else:
        pub_empty()
        response.message = "limits deactivated!"
    return response

if __name__ == '__main__':
    rospy.init_node('Arena_Limiter', anonymous=False)
    rospy.loginfo("node started")
    pub = rospy.Publisher('/planner/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
    srv = rospy.Service("/arena_limits", SetBool, srvCallback)
    rospy.spin()