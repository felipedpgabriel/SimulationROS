#!/usr/bin/env python

############################################################################
##                            Equipe Hawkings                             ##
############################################################################
## author: Felipe Gabriel                                                 ##
## last update(d/m/a): 06/07/2021                                         ##
############################################################################
## Uma ROS Subscriber escrita do zero para testar a compreensao dos       ##
## codigos do tutorial do ros wiki. Vai receber dados do randomSensor.py  ##
############################################################################

# Imports BEGIN #
## rospy: ferramentas do ros para python
## std_msgs.msg.Int8MultiArray: message type para passar um vetor
import rospy
from std_msgs.msg import Int8MultiArray
# Imports END #
def angleStatus(acell):
    moveFT = moveDE = angle = ''
    if acell.data[2] != 10:
        if acell.data[0] > 0:
            moveFT = 'para frente'
        elif acell.data[0] < 0:
            moveFT = 'para tras'
        else:
            pass
        if acell.data[1] > 0:
            moveDE = 'para a esquerda'
        elif acell.data[1] < 0:
            moveDE = 'para a direita'
        else:
            pass
    else:
        angle = 'estavel'
    if moveFT and moveDE != '':
        status = moveFT + ' e ' + moveDE
    elif moveFT != '':
        status = moveFT
    elif moveDE != '':
        status = moveDE
    else:
        status = angle
    ## loginfo: uma especie de print 3 em 1 -> print na tela, de fato; armazena nos arquivos log e no topic rosout
    rospy.loginfo(status)
# Funcao Subscriber BEGIN #
def angle():
    # Setup BEGIN #
    ## rospy.init_node: nicia e nomeia a node / gera nomes exclusivos para cada node dessa iniciada
    ## Subscriber: inscreve a node no topic, especifica o tipo de dado a ser lido, define a funcao a ser executada
    ## spin: mantem a node ativa ate ser, de fato, encerrada 
    rospy.init_node('angle', anonymous=False)
    rospy.Subscriber('sensor', Int8MultiArray, angleStatus)
    rospy.spin()
# Funcao Subscriber END #
## condicional: Roda a funcao no proprio arquivo
if __name__ == '__main__':
    angle()
