#!/usr/bin/env python

import pygame

import rospy
import mavros_msgs
from mavros_msgs import srv
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import BatteryState, Image
from std_msgs.msg import String
from mavros_msgs.msg import State

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

MAX_VEL = 1
leftvel_to_pub = (0, MAX_VEL, 0)
rightvel_to_pub = (0, -MAX_VEL, 0)
downvel_to_pub = (0, 0, -MAX_VEL)
upvel_to_pub = (0, 0, MAX_VEL)
frontvel_to_pub = (MAX_VEL, 0, 0)
backvel_to_pub = (-MAX_VEL, 0, 0)
yawRightvel_to_pub = (0, 0, 0)
yawLeftvel_to_pub = (0, 0, 0)

position = PoseStamped()
battery = BatteryState()
current_state = State()

click = pygame.mouse.get_pressed()[0]
clock = pygame.time.Clock()
mainDisplay = pygame.display.set_mode((display_width,display_height))
pygame.display.set_caption('--Skyrats: we make it fly --')


def main():

    vel_topic = rospy.get_param("/vel_topic")
    cam_topic = rospy.get_param("/cam_topic")
    rospy.loginfo("Using velocity topic as " + vel_topic)
    rospy.loginfo("Using camera topic as " + cam_topic)

    vel_pub = rospy.Publisher(vel_topic, Twist, queue_size=10)


    #======== IMAGENS ==========#
    cam_image = pygame.Surface((cam_width,cam_height))

    def img_callback(img):
        global cam_image
        cam_image = pygame.image.fromstring(img.data, (img.height,img.width), "RGB")
        cam_image = pygame.transform.scale(cam_image, (cam_width, cam_height))
        mainDisplay.blit(cam_image, cam_pose)

    img_sub = rospy.Subscriber(cam_topic, Image, img_callback)
    #=================================#
    rospy.init_node('interface', anonymous=False)
    rate = rospy.Rate(60) # 60hz

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
        def __init__(self, imgName, imgPressedName, position, size, vel):
            self.buttonImg = pygame.image.load(imgName)
            self.buttonPressedImg = pygame.image.load(imgPressedName)
            self.buttonPose = position
            self.buttonSize = size
            self.vel_to_pub = Twist()
            self.vel_to_pub.linear.x = vel[0]
            self.vel_to_pub.linear.y = vel[1]
            self.vel_to_pub.linear.z = vel[2]
            

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


        def publishVel(self):
            vel_pub.publish(self.vel_to_pub)
            rate.sleep()

    ##### Publisher subscriber "interface", que publica no topico "controle"

    mainDisplay.fill(grey)
    mousePos = pygame.mouse.get_pos()

    #=========== Criando os objetos para os botoes =================#
    upButton = Button('/home/caio/catkin_ws/src/dronecontrol/interface/media/upbutton.png', '/home/caio/catkin_ws/src/dronecontrol/interface/media/upbuttonpressed.png', ((leftControllerCenterX), (leftControllerCenterY - shift)), 150, upvel_to_pub)
    downButton = Button('/home/caio/catkin_ws/src/dronecontrol/interface/media/downbutton.png', '/home/caio/catkin_ws/src/dronecontrol/interface/media/downbuttonpressed.png', ((leftControllerCenterX), (leftControllerCenterY + shift)), 150, downvel_to_pub)
    rightButton = Button('/home/caio/catkin_ws/src/dronecontrol/interface/media/rightbutton.png', '/home/caio/catkin_ws/src/dronecontrol/interface/media/rightbuttonpressed.png', ((rightControllerCenterX + shift), (rightControllerCenterY)), 150, rightvel_to_pub)
    leftButton = Button('/home/caio/catkin_ws/src/dronecontrol/interface/media/leftbutton.png', '/home/caio/catkin_ws/src/dronecontrol/interface/media/leftbuttonpressed.png', ((rightControllerCenterX - shift), (rightControllerCenterY)), 150, leftvel_to_pub)
    frontButton = Button('/home/caio/catkin_ws/src/dronecontrol/interface/media/upbutton.png', '/home/caio/catkin_ws/src/dronecontrol/interface/media/upbuttonpressed.png', ((rightControllerCenterX), (rightControllerCenterY - shift)), 150, frontvel_to_pub)
    backButton = Button('/home/caio/catkin_ws/src/dronecontrol/interface/media/downbutton.png', '/home/caio/catkin_ws/src/dronecontrol/interface/media/downbuttonpressed.png', ((rightControllerCenterX), (rightControllerCenterY + shift)), 150, backvel_to_pub)
    yawRightButton = Button('/home/caio/catkin_ws/src/dronecontrol/interface/media/rightbutton.png', '/home/caio/catkin_ws/src/dronecontrol/interface/media/rightbuttonpressed.png', ((leftControllerCenterX + shift), (leftControllerCenterY)), 150, yawRightvel_to_pub)
    yawLeftButton = Button('/home/caio/catkin_ws/src/dronecontrol/interface/media/leftbutton.png', '/home/caio/catkin_ws/src/dronecontrol/interface/media/leftbuttonpressed.png', ((leftControllerCenterX - shift), (leftControllerCenterY)), 150, yawLeftvel_to_pub)
    button_list = {upButton, downButton, rightButton, leftButton, frontButton, backButton, yawRightButton, yawLeftButton}

    frame = VisualElement('/home/caio/catkin_ws/src/dronecontrol/interface/media/frame.png', (((display_width/2)-((display_height+shift)/2)),0))

    element_list = {frame}

    exit=False
    
    
    while not exit:
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
                    Button.publishVel()
            else:
                Button.show()
        #
        #========= Mostra elementos visuais =========#
        for Element in element_list:
            Element.show()
        #============================================#
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                exit=True
                leftButton.publish("stop")
                file.close()
            #====== Botoes!!!  ==========#
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT:
                    leftButton.active()
                    leftButton.publishVel()
                else:
                    leftButton.show()

                if event.key == pygame.K_RIGHT:
                    rightButton.active()
                    rightButton.publishVel()
                else:
                    rightButton.show()

                if event.key == pygame.K_UP:
                    frontButton.active()
                    frontButton.publishVel()
                else:
                    frontButton.show()

                if event.key == pygame.K_DOWN:
                    backButton.active()
                    backButton.publishVel()
                else:
                    backButton.show()

                if event.key == pygame.K_w:
                    upButton.active()
                    upButton.publishVel()
                else:
                    upButton.show()

                if event.key == pygame.K_a:
                    yawLeftButton.active()
                    yawLeftButton.publishVel()
                else:
                    yawLeftButton.show()

                if event.key == pygame.K_s:
                    downButton.active()
                    downButton.publishVel()
                else:
                    downButton.show()

                if event.key == pygame.K_d:
                    yawRightButton.active()
                    yawRightButton.publishVel()
                else:
                    yawRightButton.show()
            elif event.type == pygame.KEYUP:
                newvel = Twist()
                newvel.linear.x = newvel.linear.y = newvel.linear.z = 0
                vel_pub.publish(newvel)
        #log()# dont log
        pygame.display.update()
        clock.tick()
        pygame.event.poll()

if __name__ == "__main__":
    main()
