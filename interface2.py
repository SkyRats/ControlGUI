#!/usr/bin/env python

import pygame
import rospy
import mavros_msgs
from mavros_msgs import srv
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState, Image
from std_msgs.msg import String
from mavros_msgs.msg import State
from drone_video import Video
import time

pygame.init()

black = (0,0,0)
white = (255,255,255)
grey = (100,100,100)
darkGreen = (0,30,0)
brightGreen = (100, 200, 100)
brightYellow = (200, 200, 0)
orange = (250, 160, 0)
red = (255, 0, 0)

FONT = pygame.font.Font("/home/caio/catkin_ws/src/dronecontrol/interface/STIX-Bold.otf", 18)

display_width = 920
display_height = 720

xshift = 500        # parametro de deslocamento dos centros dos botoes ao x central da tela
controllerY = 550               # Coordenada y do centro dos botoes
controllerCenterX = display_width/2 - 20      #Coordenada x central da tela
buttonSide = 100    #largura da imagem .png dos botoes
rightControllerCenterX = controllerCenterX + xshift - 2*buttonSide      ## Coordenada x do centro dos botoes da direita
rightControllerCenterY = leftControllerCenterY = controllerY            ## Coordenada y do centro dos botoes
leftControllerCenterX = (controllerCenterX - xshift + 2*buttonSide)/2   ## Coordenada x do centro dos botoes da esquerda
shift = 52                              ## Distancia dos botoes ao centro
cam_height = 400
cam_width = 700
cam_pose = (display_width/2 - cam_height/1.2, display_height/2 - cam_height/1.2)
# largura e altura do retangulo para exibicao de dados
dataWidth = 320
dataHeight = 160
dataRect = [(display_width/2 - dataWidth/2), (display_height-dataHeight-20), dataWidth, dataHeight]
xc, yc = 675, 500
#========== Mensagens a serem publicadas no topico rospy ==========
leftMSG = 'left'
rightMSG = 'right'
downMSG = 'down'
upMSG = 'up'
frontMSG = 'forward'
backMSG = 'back'
yawRightMSG = 'yaw-horario'
yawLeftMSG = 'yaw-antihorario'

position = PoseStamped()
battery = BatteryState()
current_state = State()

click = pygame.mouse.get_pressed()[0]
clock = pygame.time.Clock()
mainDisplay = pygame.display.set_mode((display_width,display_height))
pygame.display.set_caption('--Skyrats: eh nois q voa --')

init_time = time.time()
last_time = init_time
def main():
    class VisualElement:
        def __init__(self, imgName, position):
            self.img = pygame.image.load(imgName)
            self.position = position

        def show(self):
            mainDisplay.blit(self.img, self.position)
            pass

    class Text:
        def __init__(self, text, position):
            pygame.font.init()
            global FONT
            self.surface = FONT.render(str(text), True, brightGreen)
            self.rectangle = self.surface.get_rect()
            self.rectangle.center = position

        def show(self):
        	mainDisplay.blit(self.surface, self.rectangle)


    class Button:
        def __init__(self, imgName, imgPressedName, position, size, msg):
            self.buttonImg = pygame.image.load(imgName)
            self.buttonPressedImg = pygame.image.load(imgPressedName)
            self.buttonPose = position
            self.buttonSize = size
            self.buttonMsg = msg
            self.show()


        def show(self):
            mainDisplay.blit(self.buttonImg, self.buttonPose)

        def active(self):
            mainDisplay.blit(self.buttonPressedImg, self.buttonPose)

        def inside(self):
            position = self.buttonPose
            mousePos = pygame.mouse.get_pos()
            s = self.buttonSize
            x = mousePos[0] - position[0]
            y = mousePos[1] - position[1]
            if (y >= -x and y<= x) and (y >= x - s and y <= s - x):
                return True
            else:
                return False
            pass

        def publishMsg(self, msg=""):
            pub = rospy.Publisher('controle', String, queue_size=10)
            rospy.init_node('interface', anonymous=True)
            rate = rospy.Rate(20) # 10hz
            if msg.len() > 1:
                str = msg
            else:
                str = self.buttonMsg
            pub.publish(str)
            rate.sleep()



	    mainDisplay.fill(grey)
	    mousePos = pygame.mouse.get_pos()

    #=========== Criando os objetos para os botoes =================#
    upButton = Button('/home/caio/catkin_ws/src/dronecontrol/interface/media/upbutton.png', '/home/caio/catkin_ws/src/dronecontrol/interface/media/upbuttonpressed.png', ((leftControllerCenterX), (leftControllerCenterY - shift)), 150, upMSG)
    downButton = Button('/home/caio/catkin_ws/src/dronecontrol/interface/media/downbutton.png', '/home/caio/catkin_ws/src/dronecontrol/interface/media/downbuttonpressed.png', ((leftControllerCenterX), (leftControllerCenterY + shift)), 150, downMSG)
    rightButton = Button('/home/caio/catkin_ws/src/dronecontrol/interface/media/rightbutton.png', '/home/caio/catkin_ws/src/dronecontrol/interface/media/rightbuttonpressed.png', ((rightControllerCenterX + shift), (rightControllerCenterY)), 150, rightMSG)
    leftButton = Button('/home/caio/catkin_ws/src/dronecontrol/interface/media/leftbutton.png', '/home/caio/catkin_ws/src/dronecontrol/interface/media/leftbuttonpressed.png', ((rightControllerCenterX - shift), (rightControllerCenterY)), 150, leftMSG)
    frontButton = Button('/home/caio/catkin_ws/src/dronecontrol/interface/media/upbutton.png', '/home/caio/catkin_ws/src/dronecontrol/interface/media/upbuttonpressed.png', ((rightControllerCenterX), (rightControllerCenterY - shift)), 150, frontMSG)
    backButton = Button('/home/caio/catkin_ws/src/dronecontrol/interface/media/downbutton.png', '/home/caio/catkin_ws/src/dronecontrol/interface/media/downbuttonpressed.png', ((rightControllerCenterX), (rightControllerCenterY + shift)), 150, backMSG)
    yawRightButton = Button('/home/caio/catkin_ws/src/dronecontrol/interface/media/rightbutton.png', '/home/caio/catkin_ws/src/dronecontrol/interface/media/rightbuttonpressed.png', ((leftControllerCenterX + shift), (leftControllerCenterY)), 150, yawRightMSG)
    yawLeftButton = Button('/home/caio/catkin_ws/src/dronecontrol/interface/media/leftbutton.png', '/home/caio/catkin_ws/src/dronecontrol/interface/media/leftbuttonpressed.png', ((leftControllerCenterX - shift), (leftControllerCenterY)), 150, yawLeftMSG)
    button_list = {upButton, downButton, rightButton, leftButton, frontButton, backButton, yawRightButton, yawLeftButton}

    frame = VisualElement('/home/caio/catkin_ws/src/dronecontrol/interface/media/frame.png', (((display_width/2)-((display_height+shift)/2)),0))

    element_list = {frame}

    #============== Inicializacao do subscriber de posicao =========#
    rospy.init_node("ControlGUI", anonymous=True)
    rate = rospy.Rate(60)
    
    #pose_param=""
    #battery_param=""
    #state_param=""
    class Param:
    	def __init__(self, pose_topic, battery_topic, state_topic):

    		self.pose_topic = rospy.get_param("~pose_topic", pose_topic)
    		self.battery_topic = rospy.get_param("~battery_topic", battery_topic)
    		self.state_topic = rospy.get_param("state_topic", state_topic)


		def pose_callback(data):
			global position
	    	position.pose.position.x = data.pose.position.x
	    	position.pose.position.y = data.pose.position.y
	    	position.pose.position.z = data.pose.position.z
	    
		def pose_sub(self, topic_name):
			self.pose_topic = topic_name.data
			position_sub = rospy.Subscriber(self.pose_topic, PoseStamped, pose_callback)
	    #==================== BATERIA ======================#
		def battery_callback(bat_dat):
			global battery

			battery.voltage = bat_dat.voltage
			battery.percentage = bat_dat.percentage
			battery.current = bat_dat.current
	    
		def bat_sub(self, topic_name):
			self.battery_topic = topic_name.data
			battery_subscriber = rospy.Subscriber(self.battery_topic, BatteryState, battery_callback)


		def state_callback(state_data):
			global current_state
			current_state = state_data
	    
		def state_sub(self, topic_name):
			self.state_topic = topic_name.data
			state_status_subscribe = rospy.Subscriber(self.state_topic, State, state_callback)

    #======== IMAGENS ==========#
    cam_image = pygame.Surface((cam_width,cam_height))

    def img_callback(img):
        global cam_image
        global newImage
        cam_image = pygame.image.fromstring(img.data, (320,240), "RGB")
        cam_image = pygame.transform.scale(cam_image, (cam_width, cam_height))
        #mainDisplay.blit(cam_image, cam_pose)

    #============ LOG ===============#

    open('/home/caio/catkin_ws/src/dronecontrol/interface/log'	, 'w').close() # Apaga os dados do log anterior
    file = open('/home/caio/catkin_ws/src/dronecontrol/interface/log', 'a')
    file.write("*********** flight log *************\n")
    file.write("Elapsed Time;X Position;Y Position;Z Position;Voltage;Current\n\n")

    def log():
        global position
        global last_time
        if(time.time()-last_time) > 0.1:

            file.write(str(time.time() - init_time))
            file.write(';')

            file.write(str(position.pose.position.x))
            file.write(';')
            file.write(str(position.pose.position.y))
            file.write(';')
            file.write(str(position.pose.position.z))
            file.write(';')

            file.write(str(battery.voltage))
            file.write(';')
            file.write(str(battery.current))
            file.write('\n')
            last_time = time.time()



    exit=False
    img_sub = rospy.Subscriber('/camera1/image_raw', Image, img_callback)
    video = Video()
    while not exit:

        # if not video.frame_available():
        # #     continue
        # # cam = video.frame()
        # #cam_image = pygame.image.fromstring(img.data, (320,240), "RGB")
        # cam = pygame.transform.scale(cam, (cam_width, cam_height))
        # mainDisplay.blit(cam, cam_pose)
        #cv2.imshow("cam",cam)


        mousePos = pygame.mouse.get_pos() # Pega posicao do mouse
        mainDisplay.fill(grey)
        pygame.draw.rect(mainDisplay, darkGreen, dataRect)
        #### Mostra os botoes de acordo com a posicao do mouse
        for Button in button_list:
            if Button.inside():
                Button.active()
                mousec = pygame.mouse.get_pressed()
                click = mousec[0]
                if click == 1:
                    Button.publishMsg()
            else:
                Button.show()
        #
        #========= Mostra elementos visuais =========#
        for Element in element_list:
            Element.show()
        #
        position_text = Text("Position: ", (350,550))
        battery_text = Text("Battery Tension: ", (350, 625))
        battery_voltage = Text(battery.voltage, (400, 640))
        battery_text2 = Text("Battery Current:", (350, 660))
        battery_current = Text(battery.current, (400, 680))
        x_pose = Text(position.pose.position.x, (380,570))
        y_pose = Text(position.pose.position.y, (380, 585))
        z_pose = Text(position.pose.position.z, (380, 600))

        position_text.show()
        x_pose.show()
        y_pose.show()
        z_pose.show()
        battery_text.show()
        battery_voltage.show()
        battery_text2.show()
        battery_current.show()

        for event in pygame.event.get():
            print('entrou no for')
            if event.type == pygame.QUIT:
                exit=True
                publish("stop")
                file.close()
                #====== Botoes!!!  ==========#
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT:
                    leftButton.active()
                    leftButton.publishMsg()
                else:
                    leftButton.show()

                if event.key == pygame.K_RIGHT:
                    rightButton.active()
                    rightButton.publishMsg()
                else:
                    rightButton.show()

                if event.key == pygame.K_UP:
                    frontButton.active()
                    frontButton.publishMsg()
                else:
                    frontButton.show()

                if event.key == pygame.K_DOWN:
                    backButton.active()
                    backButton.publishMsg()
                else:
                    backButton.show()

                if event.key == pygame.K_w:
                    upButton.active()
                    upButton.publishMsg()
                else:
                    upButton.show()

                if event.key == pygame.K_a:
                    yawLeftButton.active()
                    yawLeftButton.publishMsg()
                else:
                    yawLeftButton.show()

                if event.key == pygame.K_s:
                    downButton.active()
                    downButton.publishMsg()
                else:
                    downButton.show()

                if event.key == pygame.K_d:
                    yawRightButton.active()
                    yawRightButton.publishMsg()
                else:
                    yawRightButton.show()
                if event.key == pygame.K_j:
                    publish("disarm")
            elif event.type == pygame.KEYUP:
                publish("stop")
        log()
        pygame.display.update()
        clock.tick()
        pygame.event.poll()

if __name__ == "__main__":
	main()
