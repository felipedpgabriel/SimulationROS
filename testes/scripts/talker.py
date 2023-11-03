#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

# A linha 1 - linha shebang - indica ao sistema qual interpretador usar ao executar o arquivo.
# Linhas 2 a 37 - especificacoes das licensas.
import rospy    # (IMP) Modulo com funcionalidades do ROS.
from std_msgs.msg import String    # (IMP) Tipagem das Messages.

def talker():    # Define a funcao 
    pub = rospy.Publisher('chatter', String, queue_size=10)    #(IMP) Configura a pub - topic:'chatter', message type: String, limite da fila: 10.
    rospy.init_node('talker', anonymous=True)    # Nomeia a node, garante nomes exclusivos para Nodes clonadas.
    rate = rospy.Rate(10)    #10hz    #(IMP) Determina a frenquencia de envio.
    while not rospy.is_shutdown():    # Verifica se o programa foi fechado (comando Ctrl-C enviado)
        hello_str = "hello world %s" % rospy.get_time()    # Message "hello world" seguido do tempo de execucao.
        rospy.loginfo(hello_str)    # Printa a mensagem na tela, escreve no arquivo log da Node e no topic: rosout.
        pub.publish(hello_str)    #(IMP) Publica a Message no Topic.
        rate.sleep()    #(IMP) Faz um delay sufuciente para manter a frenquencia determinada na variavel rate.
# Essa condicional garante que a funcao definida nesse arquivo seja executada aqui e,
# caso seja importada, apenas execute a funcao chamada.
if __name__ == '__main__':    
    try:    # Executa a pub se for possivel. 
        talker()
    except rospy.ROSInterruptException:    # Encerra o codigo, caso a execucao seja interrompida (Ctrl-C)
        pass
