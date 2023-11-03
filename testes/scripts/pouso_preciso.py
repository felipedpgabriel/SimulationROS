#!/usr/bin/env python3
import rospy
import math
import time
import numpy as np
import cv2

from drone_control.motion_control_Aruco import MotionControl
from geometry_msgs.msg import TwistStamped, PoseStamped


class Auto(MotionControl):
	def __init__(self):
		MotionControl.__init__(self)
	

	def set_deslocamento_x(self, modo, deslocamento1, deslocamento2=0, deslocamento3=0):
		# MODO
			# 1: x
			# 2: y
			# 3: z
			# 4: COMPLETO
		# + = direita, frente e cima
		# - = esquerda, trás e baixo
		pos = self.posicao.pose.position
		if (modo == 1):
			self.setpoint_x(pos.x + deslocamento1)
		elif (modo == 2):
			self.setpoint_y(pos.y + deslocamento1)
		elif (modo == 3):
			self.setpoint_z(pos.z + deslocamento1)
		elif (modo == 4):
			self.setpoint(pos.x + deslocamento1, pos.y + deslocamento2, pos.z + deslocamento3)
		else:
			rospy.logwarn(f"Opcao {modo} de modo invalida!")
		

	def analisa_posicao(self):
		x = "real: {:.4f} | tvec: {:.2f} | pos_cam: {:.2f}".format(self.posicao.pose.position.x, self.tvec[0], self.pos_camera[0][0])
		y = "real: {:.4f} | tvec: {:.2f} | pos_cam: {:.2f}".format(self.posicao.pose.position.y, self.tvec[1], self.pos_camera[1][0])
		z = "real: {:.4f} | tvec: {:.2f} | pos_cam: {:.2f}".format(self.posicao.pose.position.z, self.tvec[2], self.pos_camera[2][0])

		return f"X --> {x}\nY --> {y}\nZ --> {z}\n"


	def pouso_preciso(self):
		while (self.pos_camera[2] > 0.1):
			while(abs(self.pos_camera[0])/100 > 0.2):
				self.setpoint_x((-self.tvec[0])/100)
			while(abs(self.pos_camera[1])/100 > 0.2):
				self.setpoint_y((-self.tvec[1])/100)
			z_inicial = self.pos_camera[2]/100
			while(self.pos_camera[2]/100 > z_inicial-0.3):
				self.setpoint_z(z_inicial-0.3)
		self.armar(False)


	def run(self):
		for i in range (150):
			self.setpoint(0,0,0)
		self.armar(True)
		self.detect_aruco()
		while not rospy.is_shutdown():
			for c in range(600):
				if(self.modoDeVoo.mode != "OFFBOARD"):
					self.setModoDeVoo("OFFBOARD")
				else:  
					self.setpoint(0,0,1)
					if (self.chegou(0.15) == True):
						if ((c+1)%120 == 0):
							rospy.loginfo(self.analisa_posicao())
						# self.pouso_preciso()            
	

if __name__ == "__main__":
	rospy.init_node('pouso_preciso')
	drone = Auto()
	drone.run()

