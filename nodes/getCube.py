#!/usr/bin/env python3.8
from __future__ import print_function
from asyncore import loop
from cmath import pi
import math

import numpy as np
import roslib
roslib.load_manifest('strategy')
import sys
import rospy
import tf
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Quaternion, Twist
from move_base_msgs.msg import MoveBaseActionResult
from cube_detectors.srv import DetectionsOrganized, DetectionsOrganizedResponse
from std_msgs.msg import Int32, Float32
from std_srvs.srv import SetBool, SetBoolResponse
from std_srvs.srv import Empty, EmptyResponse

def main(args):
    #definitions
    vel_back = Twist()
    vel_back.linear.x = -0.03
    rospy.init_node('mission_controller', anonymous=False)
    listener = tf.TransformListener()

    #Criação de Publishers
    pose_pub = rospy.Publisher("/planner/move_base/goal", MoveBaseActionGoal, queue_size=1, latch=True)
    vel_pub = rospy.Publisher("/planner/cmd_vel", Twist, queue_size=1, latch=True)
    claw_pub = rospy.Publisher("/claw_pos", Float32, queue_size=1)
    arm_pub  = rospy.Publisher("/motHeight", Int32, queue_size=1, latch=True)

    #Conexão com Serviços Exteriores
    dynamic_obstacle = rospy.ServiceProxy("/arena_limits", SetBool, persistent=False)
    detector_enabled = rospy.ServiceProxy("/enable_detection", SetBool, persistent=False)
    list_cleaner = rospy.ServiceProxy("/list_cleanner", Empty, persistent=False)
    detect_organizer = rospy.ServiceProxy("/get_all_detections_organized", DetectionsOrganized, persistent=False)

    quadrante = 1
    while True:
        rospy.loginfo("Start cycle!")
        rospy.sleep(rospy.Duration(1))

        #Ativa os obstaculos dinamicos
        dynamic_obstacle(True)
        #detector_enabled(True) #disable detection

        movimento = MoveBaseActionGoal()
        movimento.goal.target_pose.header.frame_id = "map"

        #Se direciona ao quadrante(6,2/3)
        if quadrante == 1:
            movimento.goal.target_pose.pose.position.x = -0.64
            movimento.goal.target_pose.pose.position.y = 0.43
            movimento.goal.target_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,0))
            pose_pub.publish(movimento)
            rospy.wait_for_message("/planner/move_base/result", MoveBaseActionResult)
            rospy.sleep(rospy.Duration(1))
        else:
            movimento.goal.target_pose.pose.position.x = -0.64
            movimento.goal.target_pose.pose.position.y = -0.43
            movimento.goal.target_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,0))
            pose_pub.publish(movimento)
            rospy.wait_for_message("/planner/move_base/result", MoveBaseActionResult)
            rospy.sleep(rospy.Duration(1))

        #Limpa a lista de cubos, ativa o detector e organiza a lista
        list_cleaner()
        #detector_enabled(True)
        rospy.sleep(rospy.Duration(3))
        organized = detect_organizer()
        rospy.loginfo(organized)
        melhor_cubo = organized.cubes.detections[0]
        rospy.sleep(rospy.Duration(1))

        if organized.cubes.detections != 0:
            #obtem a coordenada do cubo referente ao mapa
            listener.waitForTransform('/map', '/{}'.format(melhor_cubo.name), rospy.Time(), rospy.Duration(4.0))
            trans, rot = listener.lookupTransform('/map', '/{}'.format(melhor_cubo.name), rospy.Time())
            print("Trans:", trans)
        
        #detector_enabled(False)

        claw_pub.publish(Float32(180)) #abre a garra

        #Desativa os obstaculos dinamicos
        dynamic_obstacle(False)

        #TEMPORARIO
        movimento.goal.target_pose.pose.position.x = -0.32
        movimento.goal.target_pose.pose.position.y = 0.43
        movimento.goal.target_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,0))
        pose_pub.publish(movimento)
        rospy.wait_for_message("/planner/move_base/result", MoveBaseActionResult)
        rospy.sleep(rospy.Duration(1))
        

        #move robô 30cm do centro do cubo
        movimento.goal.target_pose.pose.position.y = trans[1]
        movimento.goal.target_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,pi))
        pose_pub.publish(movimento)
        rospy.wait_for_message("/planner/move_base/result", MoveBaseActionResult)
        rospy.sleep(rospy.Duration(1))
        movimento.goal.target_pose.pose.position.x = trans[0] - 0.2
        pose_pub.publish(movimento)
        rospy.wait_for_message("/planner/move_base/result", MoveBaseActionResult)
        rospy.sleep(rospy.Duration(1))

        #movimento burro
        rospy.loginfo("MovBurro")
        rospy.sleep(rospy.Duration(1.5))
        vel_pub.publish(vel_back)
        rospy.sleep(rospy.Duration(1))
        vel_pub.publish(Twist())


        #fecha e levanta a garra
        claw_pub.publish(Float32(0))
        rospy.sleep(rospy.Duration(1))
        arm_pub.publish(Int32(3178))
        rospy.sleep(rospy.Duration(2))

        #Solução Temporária para trajetorias em areas proibidas
        movimento.goal.target_pose.pose.position.x = -0.64
        movimento.goal.target_pose.pose.position.y = 0.43
        movimento.goal.target_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,0))
        pose_pub.publish(movimento)
        rospy.wait_for_message("/planner/move_base/result", MoveBaseActionResult)
        rospy.sleep(rospy.Duration(1))
        movimento.goal.target_pose.pose.position.x = -0.64
        movimento.goal.target_pose.pose.position.y = 0
        movimento.goal.target_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,0))
        pose_pub.publish(movimento)
        rospy.wait_for_message("/planner/move_base/result", MoveBaseActionResult)
        rospy.sleep(rospy.Duration(1))
        movimento.goal.target_pose.pose.position.x = 0.92
        movimento.goal.target_pose.pose.position.y = 0
        movimento.goal.target_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,0))
        pose_pub.publish(movimento)
        rospy.wait_for_message("/planner/move_base/result", MoveBaseActionResult)
        rospy.sleep(rospy.Duration(1))
        
        '''
        #vai pra trás burramente
        vel_pub.publish(vel_back)
        #aguarda 1 seg
        rospy.sleep(rospy.Duration(0.5))
        #para de andar
        vel_pub.publish(Twist())
        '''
        if len(organized.cubes.detections) == 0:
            if melhor_cubo.name == "green":
                movimento.goal.target_pose.pose.position.x = -0.64
                movimento.goal.target_pose.pose.position.y = 0.64
                movimento.goal.target_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,math.radians(325)))
                pose_pub.publish(movimento)
                rospy.wait_for_message("/planner/move_base/result", MoveBaseActionResult)
                rospy.sleep(rospy.Duration(1))
                movimento.goal.target_pose.pose.position.x = -0.81
                movimento.goal.target_pose.pose.position.y = 0.81
                movimento.goal.target_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,math.radians(325)))
                pose_pub.publish(movimento)
                rospy.wait_for_message("/planner/move_base/result", MoveBaseActionResult)
                rospy.sleep(rospy.Duration(1))
            elif melhor_cubo.name == "red":
                movimento.goal.target_pose.pose.position.x = -0.64
                movimento.goal.target_pose.pose.position.y = 0
                movimento.goal.target_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,0))
                pose_pub.publish(movimento)
                rospy.wait_for_message("/planner/move_base/result", MoveBaseActionResult)
                rospy.sleep(rospy.Duration(1))
                movimento.goal.target_pose.pose.position.x = -0.81
                movimento.goal.target_pose.pose.position.y = 0.81
                movimento.goal.target_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,0))
                pose_pub.publish(movimento)
                rospy.wait_for_message("/planner/move_base/result", MoveBaseActionResult)
                rospy.sleep(rospy.Duration(1))
            elif melhor_cubo.name == "yellow":
                movimento.goal.target_pose.pose.position.x = -0.64
                movimento.goal.target_pose.pose.position.y = 0.32
                movimento.goal.target_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,0))
                pose_pub.publish(movimento)
                rospy.wait_for_message("/planner/move_base/result", MoveBaseActionResult)
                rospy.sleep(rospy.Duration(1))
            elif melhor_cubo.name == "blue":
                movimento.goal.target_pose.pose.position.x = -0.64
                movimento.goal.target_pose.pose.position.y = 0.64
                movimento.goal.target_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,0))
                pose_pub.publish(movimento)
                rospy.wait_for_message("/planner/move_base/result", MoveBaseActionResult)
                rospy.sleep(rospy.Duration(1))
            elif melhor_cubo.name != "green" or "red" or "yellow" or "blue":
                dynamic_obstacle(True)
                movimento.goal.target_pose.pose.position.x = 0.92
                movimento.goal.target_pose.pose.position.y = 0.64
                movimento.goal.target_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,pi))
                pose_pub.publish(movimento)
                rospy.wait_for_message("/planner/move_base/result", MoveBaseActionResult)
                rospy.sleep(rospy.Duration(1))
                arm_pub.publish(Int32(-3178))
                rospy.sleep(rospy.Duration(1))
                claw_pub.publish(Float32(120))
                rospy.sleep(rospy.Duration(1))

                #Solução Temporária para trajetorias em areas proibidas
                movimento.goal.target_pose.pose.position.x = 0.92
                movimento.goal.target_pose.pose.position.y = 0
                movimento.goal.target_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,pi))
                pose_pub.publish(movimento)
                rospy.wait_for_message("/planner/move_base/result", MoveBaseActionResult)
                rospy.sleep(rospy.Duration(1))
                movimento.goal.target_pose.pose.position.x = -0.64
                movimento.goal.target_pose.pose.position.y = 0
                movimento.goal.target_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,0))
                pose_pub.publish(movimento)
                rospy.wait_for_message("/planner/move_base/result", MoveBaseActionResult)
                rospy.sleep(rospy.Duration(1))
        else:
            quadrante = 0

        
        '''
        #Quadrante Verde
        movimento.goal.target_pose.pose.position.x = -0.64
        movimento.goal.target_pose.pose.position.y = 0.64
        movimento.goal.target_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,math.radians(325)))
        pose_pub.publish(movimento)
        rospy.wait_for_message("/planner/move_base/result", MoveBaseActionResult)
        movimento.goal.target_pose.pose.position.x = -0.81
        movimento.goal.target_pose.pose.position.y = 0.81
        movimento.goal.target_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,math.radians(325)))
        pose_pub.publish(movimento)
        rospy.wait_for_message("/planner/move_base/result", MoveBaseActionResult)
        
        #Quadrante Azul
        movimento.goal.target_pose.pose.position.x = -0.64
        movimento.goal.target_pose.pose.position.y = 0.64
        movimento.goal.target_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,0))
        pose_pub.publish(movimento)
        rospy.wait_for_message("/planner/move_base/result", MoveBaseActionResult)

        #Quadrante Amarelo
        movimento.goal.target_pose.pose.position.x = -0.64
        movimento.goal.target_pose.pose.position.y = 0.32
        movimento.goal.target_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,0))
        pose_pub.publish(movimento)
        rospy.wait_for_message("/planner/move_base/result", MoveBaseActionResult)

        #Quadrante Vermelho
        movimento.goal.target_pose.pose.position.x = -0.64
        movimento.goal.target_pose.pose.position.y = 0
        movimento.goal.target_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,0))
        pose_pub.publish(movimento)
        rospy.wait_for_message("/planner/move_base/result", MoveBaseActionResult)
        '''

        #Abaixa a garra
        #arm_pub.publish(Int32(-3178))
        #rospy.sleep(rospy.Duration(1))
        #claw_pub.publish(Float32(120))
        #rospy.sleep(rospy.Duration(1))


if __name__ == '__main__':
    main(sys.argv)
