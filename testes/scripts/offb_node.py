#!/usr/bin/env python

import rospy
import mavros
from geometry_msgs.msg import PoseStamped
# mavros_msgs e a package onde estao os arquivos msg e srv
from mavros_msgs.msg import State 
from mavros_msgs.srv import CommandBool, SetMode

# callback method for state sub
current_state = State() 
# offb_set_mode = SetMode
def state_cb(state):
    global current_state
    current_state = state

## publica os valores do setpoint
local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
## recebe o dados do modo de voo (classe State)
state_sub = rospy.Subscriber('/mavros/state', State, state_cb)
## servico de aramar o drone
arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
## servico de escolher o modo de voo
set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode) 

## transforma a variavel em classe
pose = PoseStamped()
## SETa os parametros da posicao 
pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = 2

# MAIN FUNCTION BEGIN #
def position_control():
    ## inicia e nomeia a node
    rospy.init_node('offb_node', anonymous=True)
    ## armazena a 
    prev_state = current_state
    rate = rospy.Rate(20.0) # MUST be more then 2Hz

    # send a few setpoints before starting
    for i in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()
    
    # wait for FCU connection
    while not current_state.connected:
        rate.sleep()

    last_request = rospy.get_rostime()
    while not rospy.is_shutdown():
        now = rospy.get_rostime()
        if current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
            set_mode_client(base_mode=0, custom_mode="OFFBOARD")
            last_request = now 
        else:
            if not current_state.armed and (now - last_request > rospy.Duration(5.)):
               arming_client(True)
               last_request = now 

        # older versions of PX4 always return success==True, so better to check Status instead
        if prev_state.armed != current_state.armed:
            rospy.loginfo("Vehicle armed: %r" % current_state.armed)
        if prev_state.mode != current_state.mode: 
            rospy.loginfo("Current mode: %s" % current_state.mode)
        prev_state = current_state

        # Update timestamp and publish pose 
        pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose)
        rate.sleep()
# MAIN FUNCTION END #

if __name__ == '__main__':
    try:
        position_control()
    except rospy.ROSInterruptException:
        pass