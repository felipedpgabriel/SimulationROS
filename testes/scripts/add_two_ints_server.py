#!/usr/bin/env python

## Funcionou normalmente sem esta linha
from __future__ import print_function
## (IMP) Importa (da biblioteca <package>.srv) a "srv type" criada dentro de uma pasta srv
from testes.srv import AddTwoInts,AddTwoIntsResponse
## (IMP) Funcoes basicas do ROS
import rospy

# Funcoes a serem realizadas no SERVICE 
def handle_add_two_ints(req):
    ## Print para indicar a operacao desejada no service
    ## Para utilizar os valores enviados pelo client, deve-se utilizar o nome presente do file.srv
    ## Exeplo: req.a -> indica o parametro (req) e o nome da variavel do file.srv (a)
    print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
    ## (IMP) Utiliza a classe <file.srv name>Response para o return
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():
    ## Cria e nomeia a node
    rospy.init_node('add_two_ints_server')
    ## (IMP) Define o nome do service, a tipagem e a funcao a ser chamada
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    ## Apenas uma mensagem na tela, nenhuma funcao especifica
    print("Ready to add two ints.")
    ## Mantem a node funcionando ate ser encerrada
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
