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

# A linha 1 - linha shebang - indica ao sistema qual interpretador usar ao executar o arquivo.
# Linhas 2 a 37 - especificacoes das licensas.
import rospy    # (IMP) Funcoes com funcionalidades do ROS.
from std_msgs.msg import String    # (IMP) Tipagem das Messages.

def callback(data):    # Funcao a ser chamada quando receber a Message
    rospy.loginfo(rospy.get_caller_id() + ' I heard %s', data.data)    # Printa a mensagem '' na tela, seguido da Message recebida
    # rospy.loginfo(rospy.get_caller_id() + f' I heard {data.data}')    # Versão com f print

def listener():
    rospy.init_node('listener', anonymous=True)    # Nomeia a node e garante nomes exclusivos para Nodes clonadas.
    rospy.Subscriber('chatter', String, callback)    # (IMP)Inscreve a node no topic:'chatter', indica tipagem e realiza a acao (callback)
    rospy.spin()    # Mantem a node ative ate que seja parada (tipo um while)
# Essa condicional garante que a funcao definida nesse arquivo seja executada aqui e,
# caso seja importada, apenas execute a funcao chamada.
if __name__ == '__main__':
    listener()
