#!/usr/bin/env python

############################################################################
##                            Equipe Hawkings                             ##
############################################################################
## author: Felipe Gabriel                                                 ##
## last update(d/m/a): 02/12/2021                                         ##
############################################################################
## Modulo ROS que agrupa os status principais para o controle de um drone ##
## de forma "paralela". Servirah de base para conseguir movimentar drones ##
## via ROS.                                                               ##
############################################################################

import rospy    # Biblioteca padrao do ROS para python
from mavros_msgs.srv import *    # Tipagens ROS de messages nos Services
# Tipagens ROS de messages usadas como variaveis
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from mavros_msgs.msg import State
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError #Conexão do OpenCV com o ROS
import cv2 #Biblioteca do OpenCV2
import cv2.aruco as aruco #Aruco ;)
import numpy
import math
import threading

ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

ARUCO_TYPE = "DICT_4X4_50"

# Classe das principais funcoes de movimentacao de um drone
class MotionControl:
    def __init__(self):
        ### Variaveis ###
        ## Instancia os objetos que armazenarao parametros utilizados nos metodos ##

        self.modoDeVoo = State()    # Modo de voo atual
        self.destino = PoseStamped()    # Coordenas x, y e z -> destino.pose.position.(x, y ou z)
        self.rate = rospy.Rate(60)    # Frenquencia de envio dos dados (Hz)
        self.posicao = PoseStamped()    # Posicao atual do drone
        self.bridge = CvBridge()      #Conexão do OpenCV com ROS
        self.imagem_ventral = Image()
        self.aruco_thread = threading.Thread(target=self.detect_aruco, daemon=True)
        self.tvec
        # self.cv_image = 

        ### Definicao de Parametros ###
        ## Abrange os Services e os Publishers, que servem para "setar" parametros ao drone ##

            ### Services ###
                ### Checagem de Servicos ###
                ## Tenta conectar com os servicos ##

        service_timeout = 30    # Tempo para conectar aos Services
        rospy.loginfo("Conectando ROS Services...")
        try:
            rospy.wait_for_service('/mavros/set_mode',service_timeout)
            rospy.wait_for_service('/mavros/cmd/land',service_timeout)
            rospy.wait_for_service('/mavros/cmd/arming',service_timeout)
            rospy.loginfo("Services conectadas com sucesso!")
        except:
            rospy.logerr("Erro ao conectar ROS Services!")

                ### Atribuicao de Servicos ###

        self.set_modoDeVoo = rospy.ServiceProxy('/mavros/set_mode', SetMode)    # Altera o modo de voo
        self.pouso = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)    # Inicia o modo de pouso (land_mode)
        self.droneArmado = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)    # Bool para drone armado ou nao

            ### Publishers ###

        self.set_destino = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)    # Publica as coordenadas de destino
        self.image_pub = rospy.Publisher("/detected_markers",Image, queue_size=1) #Publica um frame no tópico que detecta um Aruco
        self.id_pub = rospy.Publisher("/arudo_ID", String, queue_size=1) #Publica o id do Aruco detectado
        
        ### Retorno de Parametros ###
        ## Abrange os Services, que retornam parametros referentes ao drone ##

            ### Subscribers ###

        self.modoDeVoo_atual = rospy.Subscriber('/mavros/state', State, self.get_modoDeVoo)    # Retorna o modo de voo
        self.posicao_atual = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.get_posicao)    # Retorna a posicao atual do drone
        self.detectar_aruco = rospy.Subscriber('/iris/cam_ventral/image_raw', Image, self.aruco) #Retorna os frames referentes à câmera frontal
       

    ### Callbacks ###
    ## Funcoes de "callbacks" dos subscribers ##

    def get_modoDeVoo(self, modoVoo):
        self.modoDeVoo = modoVoo
    
    
    def get_posicao(self, posicao=PoseStamped):
        self.posicao.pose.position.x = posicao.pose.position.x
        self.posicao.pose.position.y = posicao.pose.position.y
        self.posicao.pose.position.z = posicao.pose.position.z
    
    def aruco(self, imagem):
        self.imagem_ventral = imagem
    
    
    ### Funções de reconhecimento de Aruco ###
    
    #Checa se a matriz é uma matriz de rotação válida
    
    def isRotationMatrix(self,R):
        Rt = numpy.transpose(R)
        shouldBeIdentity = numpy.dot(Rt, R)
        I = numpy.identity(3, dtype=R.dtype)
        n = numpy.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6


    def rotationMatrixToEulerAngles(self,R):
        assert (self.isRotationMatrix(R))

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return numpy.array([x, y, z])
    
    # Função de detecção de aruco

    def detect_aruco(self):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(self.imagem_ventral, "bgr8")
        except CvBridgeError as e:
            print(e)

        # camera /cam_ventral
        self.camera_matrix = numpy.array([[277.191356, 0.000000, 320.500000], [0.000000, 277.191356, 240.500000], [0.000000, 0.000000, 1.000000]])
        self.distortion = numpy.array([0.000000, 0.000000, 0.000000, 0.000000, 0.000000])

        R_flip  = numpy.zeros((3,3), dtype=numpy.float32)
        R_flip[0,0] = 1.0
        R_flip[1,1] =-1.0
        R_flip[2,2] =-1.0

        #estimando posição e desenhando os eixos

        self.gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        self.aruco_dict = aruco.Dictionary_get(ARUCO_DICT[ARUCO_TYPE])
        self.parameters = aruco.DetectorParameters_create()
        (self.corners, self.ids_list, self.rejected) = aruco.detectMarkers(self.gray, self.aruco_dict, parameters = self.parameters)
        self.markers_img = aruco.drawDetectedMarkers(self.cv_image, self.corners, self.ids_list)  # detect the sruco markers and display its aruco id.

        if len(self.corners) > 0:
            self.ret = aruco.estimatePoseSingleMarkers(self.corners, 5, self.camera_matrix, self.distortion)
            self.rvec, self.tvec = self.ret[0][0,0,:], self.ret[1][0,0,:]
            aruco.drawAxis(self.markers_img, self.camera_matrix, self.distortion, self.rvec, self.tvec, 5)

            font =cv2.FONT_HERSHEY_COMPLEX
            str_position = "MARKER Position x=%4.0f y=%4.0f z=%4.0f"%(self.tvec[0], self.tvec[1], self.tvec[2])
            cv2.putText(self.markers_img, str_position, (0,100), font, 0.4, (0, 255, 0), 2, cv2.LINE_AA)

            #-- Obtain the rotation matrix tag->camera
            R_ct    = numpy.matrix(cv2.Rodrigues(self.rvec)[0])
            R_tc    = R_ct.T

            #-- Get the attitude in terms of euler 321 (Needs to be flipped first)
            roll_marker, pitch_marker, yaw_marker = self.rotationMatrixToEulerAngles(R_flip*R_tc)

            #-- Print the marker's attitude respect to camera frame
            str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker),
                                math.degrees(yaw_marker))
            cv2.putText(self.markers_img, str_attitude, (0, 150), font, 0.4, (0, 255, 0), 2, cv2.LINE_AA)


            #-- Now get Position and attitude f the camera respect to the marker
            self.pos_camera = -R_tc*numpy.matrix(self.tvec).T
            
            str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(self.pos_camera[0], self.pos_camera[1], self.pos_camera[2])
            cv2.putText(self.markers_img, str_position, (0, 200), font, 0.4, (0, 255, 0), 2, cv2.LINE_AA)

            #-- Get the attitude of the camera respect to the frame
            roll_camera, pitch_camera, yaw_camera = self.rotationMatrixToEulerAngles(R_flip*R_tc)
            str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
                                math.degrees(yaw_camera))
            cv2.putText(self.markers_img, str_attitude, (0, 250), font, 0.4, (0, 255, 0), 2, cv2.LINE_AA)

            if self.ids_list is None:
                self.id_pub.publish(self.ids_list)
            else:
                self.ids_str = ''.join(str(e) for e in self.ids_list)
                self.id_pub.publish(self.ids_str)

            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.markers_img, "8UC3"))
            except CvBridgeError as e:
                print(e)



    ### Funcoes de movimentacao ###
    ## Define acoes para a movimentacao do drone ##

    def armar(self, state):
        armado = self.droneArmado(state)
        if armado.success == True:
            rospy.loginfo("Drone Armado.")
        else:
            rospy.logwarn("Erro ao armar drone.")


    def pousar(self):
        self.pouso(altitude = 0, latitude = 0, longitude = 0, yaw = 0)
        rospy.loginfo("Pousando Drone...")
     
    # Verifica se o drone chegou aa posicao determinada, para controla o envio das coordenadas de sertino
    def chegou(self, setMax_erro):
        erro = setMax_erro
        if (abs(self.destino.pose.position.x - self.posicao.pose.position.x) < erro) and (abs(self.destino.pose.position.y - self.posicao.pose.position.y) < erro) and (abs(self.destino.pose.position.z - self.posicao.pose.position.z) < erro):
            return True
        else:
            return False

    # Define o modo de voo
    def setModoDeVoo(self, modoVoo):
        self.set_modoDeVoo(custom_mode=modoVoo)
        if self.modoDeVoo.mode == modoVoo:
            rospy.loginfo(f'Modo de voo atual: {self.modoDeVoo.mode}.')
        else:
            rospy.logwarn("Erro ao alterar modo e voo.")


    def setpoint(self, x, y, z):
        self.destino.pose.position.x = x
        self.destino.pose.position.y = y
        self.destino.pose.position.z = z
        self.destino.header.stamp = rospy.Time.now()
        self.set_destino.publish(self.destino)
        self.rate.sleep()
    

    def setpoint_x(self,x):
        self.destino.pose.position.x = x
        self.destino.header.stamp = rospy.Time.now()
        self.set_destino.publish(self.destino)
        self.rate.sleep()

    
    def setpoint_y(self,y):
        self.destino.pose.position.y = y
        self.destino.header.stamp = rospy.Time.now()
        self.set_destino.publish(self.destino)
        self.rate.sleep()
    

    def setpoint_z(self,z):
        self.destino.pose.position.z = z
        self.destino.header.stamp = rospy.Time.now()
        self.set_destino.publish(self.destino)
        self.rate.sleep()
    