#!/usr/bin/env python3

############################################################################
##                            Equipe Hawkings                             ##
############################################################################
## author: Felipe Gabriel                                                 ##
## last update(d/m/a): 02/12/2021                                         ##
############################################################################
## Codigo de teste de movimentacao, que utiliza o módulo motion_control.  ##
## Ele define tres posicoes para o drone ir, realizando um trajeto, e o   ##
## numero de ciclos, para realizar o trajetos uma certa quantidade de     ##
## vezes.                                                                 ##
############################################################################

from re import X
import rospy    # Biblioteca padrao do ROS para python
import unittest
import math
import numpy as np
import time
from rospy.exceptions import ROSException
from std_msgs.msg import String, Header
from drone_control.motion_control import MotionControl    # Modulo ROS de movimentacao de um drone

# Define uma classe para poder utilizar as funcoes da MotionControl
class Drone(MotionControl):
	def __init__(self):
		MotionControl.__init__(self)
	

	def trajeto_simples(self, modo, erro, coo1 = 0, coo2 = 0, coo3 = 0):
		# MODO
			# 1: x
			# 2: y
			# 3: z
			# 4: COMPLETO
		if(modo==1):
			while True:
				self.setpoint_x(coo1)
				if self.chegou(erro) == True:
					rospy.loginfo(self.posicao.pose.position)
					break
		if(modo==2):
			while True:
				self.setpoint_y(coo1)
				if self.chegou(erro) == True:
					rospy.loginfo(self.posicao.pose.position)
					break
		if(modo==3):
			while True:
				self.setpoint_z(coo1)
				if self.chegou(erro) == True:
					rospy.loginfo(self.posicao.pose.position)
					break
		if(modo==4):
			while True:
				self.setpoint(coo1, coo2, coo3)
				if self.chegou(erro) == True:
					rospy.loginfo(self.posicao.pose.position)
					break


   # Define as coordenadas e as mensagens para cada destino do drone
	def trajeto_completo(self, x, y, z, erro, msg_inicial = None, msg_final = None):
		if (msg_inicial is not None):
			rospy.loginfo(msg_inicial)
		while True:
			self.setpoint(x, y, z)
			if self.chegou(erro) == True:
				if (msg_final is not None):
					rospy.loginfo(msg_final)
				break
   
   # Missao a ser realizada
	def run(self):
		ciclos = int(input("Quantas voltas deseja realizar?: "))
		if ciclos != 0:
			contCiclos = 0
			for i in range (150):
				self.setpoint(0, 0, 0)
			self.armar(True)
			erroTrajeto = 0.2
			erro_TakeOff_Land = 0.1
			while not rospy.is_shutdown() and contCiclos < ciclos:
				if self.modoDeVoo.mode != "OFFBOARD":
					self.setModoDeVoo("OFFBOARD")
					if self.modoDeVoo.mode == "OFFBOARD":
						self.trajeto_completo(0, 0, 2, erro_TakeOff_Land, "Decolando...")
				else:
					self.trajeto_completo(1, 1, 3, erroTrajeto, None, "Chegou à posição 1.")
					self.trajeto_simples(1, erroTrajeto, 3)
					self.trajeto_simples(2, erroTrajeto, 3)
					self.trajeto_simples(3, erroTrajeto, 5)
					self.trajeto_completo(0, 0, 5, erroTrajeto, None, "Chegou à posição 2.")
					contCiclos += 1
			self.trajeto_completo(0, 0, 1, erro_TakeOff_Land, None, None)
			self.pousar()
			rospy.loginfo("Missão encerrada!")


	def testeCV(self):
		for i in range (150):
			self.setpoint(0, 0, 0)
		self.armar(True)
		while not rospy.is_shutdown():
			if self.modoDeVoo.mode != "OFFBOARD":
				self.setModoDeVoo("OFFBOARD")
			else:
				self.trajeto_completo(0, 0, 1, 0.3)


if __name__ == "__main__":
	try:
		rospy.init_node('mission', anonymous=True)
		Interprise = Drone()
		Interprise.run()
		#Interprise.testeCV()
	except rospy.ROSInterruptException:
		pass
