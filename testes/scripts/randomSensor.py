#!/usr/bin/env python

############################################################################
##                            Equipe Hawkings                             ##
############################################################################
## author: Felipe Gabriel                                                 ##
## last update(d/m/a): 06/07/2021                                         ##
############################################################################
## Uma ROS Publisher escrita do zero para testar a compreensao dos        ##
## codigos do tutorial do ros wiki Vai enviar dados para angle.py         ##
############################################################################

# Imports BEGIN #
## rospy: ferramentas do ros para python
## random.randint: biblioteca "aleatoria" para simular dados de um sensor
## std_msgs.msg.Int8MultiArray: message type para passar um vetor
import rospy
from random import randint
from std_msgs.msg import Int8MultiArray
# Imports END #
# Funcao Publisher BEGIN #
def acell():
    # Setup BEGIN #
    ## pub: define o topico, o message type e o tamanho da pilha (nao sei oque e)
    ## rospy.init_node: inicia e nomeia a node / gera nomes exclusivos para cada node dessa iniciada
    ## rate: frequencia de envio dos dados
    pub = rospy.Publisher('sensor', Int8MultiArray, queue_size=10)
    rospy.init_node('acell', anonymous=False)
    rate = rospy.Rate(8)
    # Setup END #
    # Loop BEGIN #
    ## acell: valores randomicos para simular dados do sensor acelerometro
    ## condicional: gera valores aleatorios para X e Y apenas se o Z nao estiver "estavel"
    while not rospy.is_shutdown():
        acellZ = randint(7, 10)
        acellX = acellY = 0
        if acellZ != 10:
            acellX = randint(-7, 7)
            acellY = randint(-7, 7)
        # PULO DO GATO  BEGIN #
        ## array: reune os dados em um vetor
        ## acellXYZ: armazena a array configurada para o message type
        array = [acellX, acellY, acellZ]
        acellXYZ = Int8MultiArray(data=array)
        # PULO DO GATO  END #
        
        ## publish: publica os valores no topic
        ## sleep: espera a quantidade correta de tempo para manter a frenquencia de envio
        rospy.loginfo(acellXYZ.data) 
        pub.publish(acellXYZ);
        rate.sleep()
    # Loop END #
# Funcao Publisher END #
## condicional: Roda a funcao no proprio arquivo
## except: Interrompe quando Ctrl+C e pressionado 
if __name__ == '__main__':
    try:
        acell()
    except rospy.ROSInterruptException:
        pass
