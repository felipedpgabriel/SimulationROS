#!/usr/bin/env python

## Funcionou normalmente sem esta linha
from __future__ import print_function
## Pega parametros passados externamente
import sys
## (IMP) Funcoes basicas do ROS
import rospy
## Importa (da biblioteca <package>.srv) a "srv type" criada dentro de uma pasta srv.
## O "*" permite utilizar qualquer "tipagem" srv dentro da pasta 
from testes.srv import AddTwoInts

def add_two_ints_client(x, y):
    ## (IMP) Bloqueia a funcao ate o servico ser realizado 
    rospy.wait_for_service('add_two_ints')
    try:
        ## (IMP) Solicita o servico indicando o nome do servico e o tipo de mensagem utilizada
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        ## (IMP) Usa o servico indicando as variaveis
        resp1 = add_two_ints(x, y)
        ## retorna a resposta (Response)
        return resp1.sum
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

## explica como usar o servico
def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    ## se o comando tiver 3 parametros
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s+%s"%(x, y))
    print("%s + %s = %s"%(x, y, add_two_ints_client(x, y)))