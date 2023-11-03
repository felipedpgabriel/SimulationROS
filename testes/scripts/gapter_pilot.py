#!/usr/bin/env python
import rospy          # base dos comandos
import math           # biblioteca matematica, nao utilizada 
import numpy as np    # ESTUDAR, nao utilizada (array)
import time           # ESTUDAR, nao utilizada
from std_msgs.msg import String, Header    # tipagem publish e subscriber
from threading import Thread    # Classe thread (threading.py a partir da linha 750)
from tf.transformations import quaternion_from_euler    
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State 
from mavros_msgs.srv import *
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped

#global variable
latitude =8.5455934
longitude=47.3977421

teste = State() 
def modoAtual(modoVoo):
    global teste 
    teste = modoVoo

def setGuidedMode():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        #http://wiki.ros.org/mavros/CustomModes for custom modes
        isModeChanged = flightModeService(custom_mode='GUIDED') #return true or false
        rospy.loginfo(isModeChanged)
        current_state = State()
        rospy.loginfo(current_state.mode)
    except rospy.ServiceException:
        print ("service set_mode call failed. GUIDED Mode could not be set. Check that GPS is enabled")
        
def setOffboardMode():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        #http://wiki.ros.org/mavros/CustomModes for custom modes
        isModeChanged = flightModeService(custom_mode="OFFBOARD") #return true or false
        rospy.loginfo(isModeChanged)
    except rospy.ServiceException:
        print ("service set_mode call failed. GUIDED Mode could not be set. Check that GPS is enabled")

def setLandMode():
    rospy.wait_for_service('/mavros/cmd/land')
    try:
        landService = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        #http://wiki.ros.org/mavros/CustomModes for custom modes
        isLanding = landService(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
        global teste
        teste = "LAND"
    except rospy.ServiceException:
        print ("service land call failed. The vehicle cannot land")
          
def setArm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        armService(True)
    except rospy.ServiceException:
        print ("Service arm call failed")
        
def setDisarm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        armService(False)
    except rospy.ServiceException:
        print ("Service arm call failed")


def setTakeoffMode():
    rospy.wait_for_service('/mavros/cmd/takeoff')
    try:
        takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL) 
        takeoffService(altitude = 2, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
    except rospy.ServiceException:
        print ("Service takeoff call failed")


def setAutoMode():
    rate = rospy.Rate(20)
    rospy.Subscriber('/mavros/state', State, modoAtual)
    destino = rospy.Publisher('/mavros/setpoint_position/local',PoseStamped,queue_size=10)
    setpoint = PoseStamped()
    for i in range (150):
        setpoint.pose.position.x = 0
        setpoint.pose.position.y = 0
        setpoint.pose.position.z = 2
        setpoint.header.stamp = rospy.Time.now()
        destino.publish(setpoint)
        rate.sleep()
    
    

    while not rospy.is_shutdown() and teste !="LAND":
        rospy.loginfo(f'Mode de voo: {teste.mode}')
        if teste.mode != "OFFBOARD":
            rospy.wait_for_service('/mavros/cmd/arming')
            try:
                armar = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
                armar(True)
                rospy.loginfo ("Drone Armado")
            except rospy.ServiceException:
                rospy.loginfo ("Chamada para a Service falhou:")

            rospy.wait_for_service('/mavros/set_mode')
            try:
                modo_de_voo = rospy.ServiceProxy('/mavros/set_mode', SetMode)
                modo_de_voo(custom_mode="OFFBOARD") #return true or false
            except rospy.ServiceException:
                rospy.loginfo("Chamada para a Service falhou (Offboard)")
        else:
            print ("Chegou aqui")
            for i in range (150):
                setpoint.pose.position.x = 0
                setpoint.pose.position.y = 0
                setpoint.pose.position.z = 2
                setpoint.header.stamp = rospy.Time.now()
                destino.publish(setpoint)
                rate.sleep()
            print ("Chegou aqui")
            for i in range (150):
                setpoint.pose.position.x = 5
                setpoint.pose.position.y = 2
                setpoint.pose.position.z = 3
                setpoint.header.stamp = rospy.Time.now()
                destino.publish(setpoint)
                rate.sleep()
            for i in range (150):
                setpoint.pose.position.x = 0
                setpoint.pose.position.y = 0
                setpoint.pose.position.z = 4
                setpoint.header.stamp = rospy.Time.now()
                destino.publish(setpoint)
                rate.sleep()
            print ("Chegou aqui")
            setLandMode()
        


def globalPositionCallback(globalPositionCallback):
    global latitude
    global longitude
    latitude = globalPositionCallback.latitude
    longitude = globalPositionCallback.longitude
    #print ("longitude: %.7f" %longitude)
    #print ("latitude: %.7f" %latitude)

def menu():
    print ("Press")
    print ("1: to set mode to GUIDED")
    print ("2: to set mode to STABILIZE")
    print ("3: to set mode to ARM the drone")
    print ("4: to set mode to DISARM the drone")
    print ("5: to set mode to TAKEOFF")
    print ("6: to set mode to LAND")
    print ("7: to set mode to AUTO")
    print ("8: print GPS coordinates")
    
def myLoop():
    x='1'
    while ((not rospy.is_shutdown())and (x in ['1','2','3','4','5','6','7','8'])):
        menu()
        x = input("Enter your input: ")
        if (x=='1'):
            setGuidedMode()
        elif(x=='2'):
            setOffboardMode()
        elif(x=='3'):
            setArm()
        elif(x=='4'):
            setDisarm()
        elif(x=='5'):
            setTakeoffMode()
        elif(x=='6'):
            setLandMode()
        elif(x=='7'):
            setAutoMode()
        elif(x=='8'):
            global latitude
            global longitude
            print ("longitude: %.7f" %longitude)
            print ("latitude: %.7f" %latitude)
        else: 
            print ("Exit")
        
        
    

if __name__ == '__main__':
    rospy.init_node('gapter_pilot_node', anonymous=True)
    rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPositionCallback)
    velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    # spin() simply keeps python from exiting until this node is stopped
    
    #listener()
    myLoop()
    #rospy.spin()
