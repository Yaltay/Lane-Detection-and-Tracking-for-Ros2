import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ipekyolu_interfaces.msg import VelocityCommand
import math
import numpy as np
import time
"""
In this project, we have used our own Velocity package instead of the traditional cmd_vel.
Additionally, you can customize your camera subscription, making it much easier to use.

For any questions, feel free to contact yaltay556@gmail.com.
"""

MIN_AREA = 500 

MIN_AREA_TRACK = 5000

class IntelSubscriber(Node):
    def __init__(self):
	# Serit takip adında node oluşturup bunun geerekli bağımlıklarını ve değişkenlerini yazalım.
        super().__init__("SeritTakip")
        self.subscription_rgb = self.create_subscription(Image, "camera/raw", self.rgb_frame_callback, 10)
        self.cmd_vel_pub = self.create_publisher(VelocityCommand, 'subsysi/cmd_vel', 10)
	
        self.bridge = CvBridge()
        self.vel_msg=VelocityCommand()
        self.vel_msg.commander = "line"
        self.soldonus = None
        self.soldonusDurumu = False
        self.sagdonus = None
        self.sagdonusDurumu = False
        self.oncekiEmir = ""
        self.createTrackBar()
        self.W = 0
        self.H = 0
        self.frame = None


	# uygun renk ortamını bulmak için trackbar kullanıyoruz.
    def createTrackBar(self):
        cv2.namedWindow("TrackBars")
        cv2.resizeWindow("TrackBars", 480, 300)
        cv2.createTrackbar("Hue_Min", "TrackBars", 26, 179, self.empty)
        cv2.createTrackbar("Hue_Max", "TrackBars", 163, 179, self.empty)
        cv2.createTrackbar("Sat_Min", "TrackBars", 8, 255, self.empty)
        cv2.createTrackbar("Sat_Max", "TrackBars", 145, 255, self.empty)
        cv2.createTrackbar("Val_Min", "TrackBars", 25, 255, self.empty)
        cv2.createTrackbar("Val_Max", "TrackBars", 95, 255, self.empty)


	# trackbardan gelen değerler ile maskeleme işlemi yapıyoruz ve maskelenmiş resmi return ediyoruz.
    def applyMask(self, imgHSV):
        h_min = cv2.getTrackbarPos("Hue_Min", "TrackBars")
        h_max = cv2.getTrackbarPos("Hue_Max", "TrackBars")
        s_min = cv2.getTrackbarPos("Sat_Min", "TrackBars")
        s_max = cv2.getTrackbarPos("Sat_Max", "TrackBars")
        v_min = cv2.getTrackbarPos("Val_Min", "TrackBars")
        v_max = cv2.getTrackbarPos("Val_Max", "TrackBars")
        
        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(imgHSV, lower, upper)
        return mask
    
    def empty(self, x):
        pass
	
	#Resmi HSV türüne dönüştürüyoruz.
    def convertHsv(self, img):
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        return imgHSV
	#Dilation işlemi yapan fonksiyon tanımlıyoruz
    def makeDialation(self, imgCanny, iterations):
        kernel = np.ones((5, 5), np.uint8)
        imgDialation = cv2.dilate(imgCanny, kernel, iterations=iterations)
        return imgDialation


	#makeEroded işlemi yapan fonksiyon tanımlıyoruz
    def makeEroded(self, imgDialation, iterations):
        kernel = np.ones((5, 5), np.uint8)
        imgEroded = cv2.erode(imgDialation, kernel, iterations=iterations)
        return imgEroded
	
	
	#Blurlama  işlemi yapan fonksiyon tanımlıyoruz
    def makeGaussianBlur(self, imgGray):
        kernel = np.ones((5, 5), np.uint8)
        imgBlur = cv2.GaussianBlur(imgGray, (5, 5), 0)
        return imgBlur
    
    

    


    def rgb_frame_callback(self, data):
        self.get_logger().warning("Receiving RGB frame")
        frame = self.bridge.imgmsg_to_cv2(data)
        frame = cv2.resize(frame, (160,120))
        self.frame=frame
        
        cv2.line(self.frame, (int(self.W / 2 + self.W * 0.4), 0), (int(self.W / 2 + self.W * 0.4), self.H), (0, 0, 255),3)
        cv2.line(self.frame, (int(self.W / 2 - self.W * 0.4), 0), (int(self.W / 2 - self.W * 0.4), self.H), (0, 0, 255),3)
        cv2.line(self.frame, (int(self.W / 2 + self.W * 0.25), 0), (int(self.W / 2 + self.W * 0.25), self.H), (0, 255, 0),3)
        cv2.line(self.frame, (int(self.W / 2 - self.W * 0.25), 0), (int(self.W / 2 - self.W * 0.25), self.H), (0, 255, 0),3)
        cv2.line(self.frame, (int(self.W / 2 + self.W * 0.15), 0), (int(self.W / 2 + self.W * 0.15), self.H), (255, 0, 0),3)
        cv2.line(self.frame, (int(self.W / 2 - self.W * 0.15), 0), (int(self.W / 2 - self.W * 0.15), self.H), (255, 0, 0),3)
        cv2.line(self.frame, (int(self.W / 2 + self.W * 0.10), 0), (int(self.W / 2 + self.W * 0.10), self.H), (0, 255, 255),3)
        cv2.line(self.frame, (int(self.W / 2 - self.W * 0.10), 0), (int(self.W / 2 - self.W * 0.10), self.H), (0, 255, 255),3)
        
        hsv = self.convertHsv(frame)
        mask = self.applyMask(hsv)
        blur = self.makeGaussianBlur(mask)
        dilate = self.makeDialation(blur, 5)
        erode = self.makeEroded(dilate, 5)
	# Yukarıda aldığımız resimleri hsv mask blur gibi işlemlere soktuktan sonra proces fonksiyonuna yolluyoruz.
        self.imgProces(erode)
        cv2.imshow("AnaKamera",self.frame)
        cv2.waitKey(1)
        


    def findCoordinats(self,imgDialate):
        self.W = imgDialate.shape[1]
        self.H = imgDialate.shape[0]

        imgA=imgDialate.copy()
        imgB=imgDialate.copy()
        imgC=imgDialate.copy()

        imgA = imgA[self.H//2:self.H , 0:self.W]
        imgB = imgB[0:self.H//2 , 0:self.W//2 ]
        imgC = imgC[0:self.H//2 , self.W//2:self.W ]
        
        A_cx=-1 #Center point coordinats A(x)
        A_cy=-1 #Center point coordinats A(y)

        B_cx=-1 #Center point coordinats B(x)
        B_cy=-1 #Center point coordinats B(y)

        C_cx=-1 #Center point coordinats C(x)
        C_cy=-1 #Center point coordinats C(y)       

        A_area=0 #Area of the A frame 
        B_area=0 #Area of the B frame 
        C_area=0 #Area of the C frame 

        #Finding B area And B center Coordinats 
        contoursB,hierarchyB=cv2.findContours(imgB,1,cv2.CHAIN_APPROX_NONE)
        if len(contoursB)>0:
            global b
            b=max(contoursB,key=cv2.contourArea,default=0)
            M_b=cv2.moments(b)
            for contour in contoursB:
                area=cv2.contourArea(contour)
                if(area>=50):
                    if M_b["m00"] != 0:
                        cx_b = int(M_b['m10'] / M_b['m00'])
                        cy_b = int(M_b['m01'] / M_b['m00'])
                        B_cx=cx_b
                        B_cy=cy_b
                        cv2.circle(imgB, (cx_b, cy_b), 5, (0, 255, 0), -1)

                        B_area=area

        else:
            print("B is Empty")


        #Finding C area And C center Coordinats 
        contoursC,hierarchyC=cv2.findContours(imgC,1,cv2.CHAIN_APPROX_NONE)
        if len(contoursC)>0:
            global c
            c=max(contoursC,key=cv2.contourArea,default=0)
            M_c=cv2.moments(c)
            for contour in contoursC:
                area=cv2.contourArea(contour)
                if(area>=50):
                    if M_c["m00"] != 0:
                        cx_c = int(M_c['m10'] / M_c['m00'])
                        cy_c = int(M_c['m01'] / M_c['m00'])
                        C_cx=cx_c+self.W/2
                        C_cy=cy_c
                        cv2.circle(imgC, (cx_c, cy_c), 5, (0, 255, 0), -1)
                        C_area=area



        else:
            print("C is Empty")


        #Finding A area And A center Coordinats 
        contoursA, hierarchy = cv2.findContours(imgA, 1, cv2.CHAIN_APPROX_NONE)
        if len(contoursA)>0:
            global a
            a=max(contoursA,key=cv2.contourArea,default=0)
            M_a=cv2.moments(a)
            for contour in contoursA:
                area=cv2.contourArea(contour)
                A_area=area
                if area >50:
                    if M_a["m00"] != 0:
                        cx_a = int(M_a['m10'] / M_a['m00'])
                        cy_a = int(M_a['m01'] / M_a['m00'])
                        cv2.circle(imgA,(cx_a,cy_a),5,(0,0,255),-1)
                        cy_a=cy_a+self.H/2
                        A_cx=cx_a
                        A_cy=cy_a

        if(B_area>=C_area):
            C_cx=-1
            C_cy=-1
        else:
            B_cx=-1
            B_cy=-1    

        merkezKoordinatlari=np.array([A_cx,A_cy,B_cx,B_cy,C_cx,C_cy])
        cv2.imshow('A',imgA)
        cv2.imshow('B',imgB) 
        cv2.imshow('C',imgC)
        return imgDialate,merkezKoordinatlari,A_area
        
    def calculateDegree(self,aciKoordinatlari):
            A_cx=aciKoordinatlari[0]
            A_cy=aciKoordinatlari[1]

            B_cx=aciKoordinatlari[2]
            B_cy=aciKoordinatlari[3]

            B=-B_cx+A_cx
            A=-B_cy+A_cy
            self.aciDegeri=math.degrees(math.atan(A/B))
            # print("AÇI DEĞERİ : ",aciDegeri)
            return self.aciDegeri
    

    def Dengele(self, Ax):
        
        if Ax >= 150:
            print("Saga dengele")
            self.vel_msg.velocity.linear = 0.3
            self.vel_msg.velocity.angular = -0.2
            cv2.putText(self.frame,"Saga Dengele",(200,200),cv2.FONT_HERSHEY_COMPLEX,0.5,(0,0,255),2)
        
        elif Ax >= 120 and Ax < 150:
            print("Saga Orta dengele")
            self.vel_msg.velocity.linear = 0.27
            self.vel_msg.velocity.angular = -0.12
            cv2.putText(self.frame,"Saga Orta Dengele",(200,200),cv2.FONT_HERSHEY_COMPLEX,0.5,(0,0,255),2)

        elif Ax >= 100 and Ax < 120:
            print("Saga Orta dengele")
            self.vel_msg.velocity.linear = 0.3
            self.vel_msg.velocity.angular = -0.07
            cv2.putText(self.frame,"Saga az Dengele",(200,200),cv2.FONT_HERSHEY_COMPLEX,0.5,(0,0,255),2)

       
       

        elif Ax > 60 and Ax <100:
            print("Duz Git")
            cv2.putText(self.frame,"Duz git",(200,200),cv2.FONT_HERSHEY_COMPLEX,0.5,(0,0,255),2)
            self.vel_msg.velocity.linear = 0.3
            self.vel_msg.velocity.angular = 0.0
            
            
            
            
            
        elif Ax <= 10:
            print("Sola Dengele")
            cv2.putText(self.frame,"Sola Dengele",(200,200),cv2.FONT_HERSHEY_COMPLEX,0.5,(0,0,255),2)
            self.vel_msg.velocity.linear = 0.28
            self.vel_msg.velocity.angular = 0.18
        elif Ax <= 40 and Ax > 10:
            print("Sola Orta dengele")
            cv2.putText(self.frame,"Sola Orta Dengele",(200,200),cv2.FONT_HERSHEY_COMPLEX,0.5,(0,0,255),2)
            self.vel_msg.velocity.linear = 0.27
            self.vel_msg.velocity.angular = 0.1
        elif Ax <= 60 and Ax > 40:
            print("Sola Orta dengele")
            cv2.putText(self.frame,"Sola az  Dengele",(200,200),cv2.FONT_HERSHEY_COMPLEX,0.5,(0,0,255),2)
            self.vel_msg.velocity.linear = 0.3
            self.vel_msg.velocity.angular = 0.07
            
        
        
        
        self.cmd_vel_pub.publish(self.vel_msg)




    def DonusAlgoritmasi(self, emir):
        pass
        """if emir == "sol":
            print("Sola Don")
            self.vel_msg.velocity.linear = 0.3
            self.vel_msg.velocity.angular = 1.2  # Açıyı daha küçük yaptım
        elif emir == "sag":
            print("Saga Don")
            cv2.putText(self.frame,"Saga Donus Basladi",(200,200),cv2.FONT_HERSHEY_COMPLEX,0.8,(0,0,255),4)

            self.vel_msg.velocity.linear = 0.3
            self.vel_msg.velocity.angular = -1.2 # Açıyı daha küçük yaptım
        elif emir == "dur":
            self.vel_msg.velocity.linear = 0.0
            self.vel_msg.velocity.angular = 0.0
            self.cmd_vel_pub.publish(self.vel_msg)
            
        self.cmd_vel_pub.publish(self.vel_msg)
"""



        
            

    def imgProces(self,frame):
        
        imgCounter,MerkezNoktalari,Aalan = self.findCoordinats(frame)
        
        aX = int(MerkezNoktalari[0])
        aY = int(MerkezNoktalari[1])
        bX = int(MerkezNoktalari[2])
        bY = int(MerkezNoktalari[3])
        cX = int(MerkezNoktalari[4])
        cY = int(MerkezNoktalari[5])
        self.A_noktasi= None
        self.B_noktasi= None
        self.C_noktasi= None

        if MerkezNoktalari[0] != -1:
            self.A_noktasi = True
            cv2.putText(self.frame,"A",(aX,aY),cv2.FONT_HERSHEY_COMPLEX,0.5,(0,0,255),2)
            cv2.circle(self.frame,(aX,aY),7,(0,0,255),-1)
        else:
            self.A_noktasi = False
        if MerkezNoktalari[2] != -1:
            self.B_noktasi = True
            cv2.putText(self.frame,"B",(bX,bY),cv2.FONT_HERSHEY_COMPLEX,0.5,(0,0,255),2)
            cv2.circle(self.frame,(bX,bY),7,(0,0,255),-1)
        else:
            self.B_noktasi = False
        if MerkezNoktalari[4] != -1:
            self.C_noktasi = True
            cv2.putText(self.frame,"C",(cX,cY),cv2.FONT_HERSHEY_COMPLEX,0.5,(0,0,255),2)
            cv2.circle(self.frame,(cX,cY),7,(0,0,255),-1)
        else:
            self.C_noktasi = False

        self.aci_ac = 0
        self.aci_ab = 0

        if(self.A_noktasi and self.B_noktasi):
            cv2.line(self.frame,(aX,aY),(bX,bY),(0,0,255),3)
            aciKoordinatlari = np.array([aX,aY,bX,bY])
            self.aci_ab = self.calculateDegree(aciKoordinatlari=aciKoordinatlari)
            self.aci_ab = math.floor(self.aci_ab)
            cv2.putText(self.frame,str(self.aci_ab),(15,15),cv2.FONT_HERSHEY_COMPLEX,0.7,(0,0,255),2)
        
        
        elif(self.A_noktasi and self.C_noktasi): 
            cv2.line(self.frame,(aX,aY),(cX,cY),(255,255,0),4)
            aciKoordinatlari=np.array([aX,aY,cX,cY])
            self.aci_ac=self.calculateDegree(aciKoordinatlari)
            self.aci_ac=math.floor(self.aci_ac)
            cv2.putText(self.frame,str(self.aci_ac),(400,15),cv2.FONT_HERSHEY_COMPLEX,0.7,(0,255,0),2)


        

        
        self.aci_ac = abs(self.aci_ac)
        self.aci_ab = abs(self.aci_ab)
        print(self.aci_ac ,"aci ac")
        print(self.aci_ab ,"aci ab")
        


        if (self.aci_ab >30 and self.aci_ab< 70) and self.soldonusDurumu == False  :
            print("Sola Don")
            self.soldonus = True
        if (self.aci_ac >30 and self.aci_ac< 70) and self.sagdonusDurumu == False  :
            print("Saga Don")
            self.sagdonus = True


        if ((self.sagdonus == True or self.soldonus == True) and (self.C_noktasi == False and self.B_noktasi == False)):
            if self.sagdonus == True:
                print("***** Saga Donus Basladi *******")


                self.sagdonusDurumu = True
                self.sagdonus = False

            if self.soldonus == True:
                print("***** Sola Donus Basladi *******")


                self.soldonusDurumu = True
                self.soldonus = False

        if((self.aci_ac<=90 and self.aci_ac>=40) or (self.aci_ab<=90 and self.aci_ab>=40)):
            self.inRange = True
        else:
            self.inRange = False
        
        if (self.sagdonusDurumu == True and self.inRange == False):
            self.DonusAlgoritmasi("sag")
            print("Turn Right")
        elif (self.sagdonusDurumu == True and self.inRange == True):
            print("Turn Right")    
            self.sagdonusDurumu = False  


        if (self.soldonusDurumu== True and self.inRange == False):
            print("Sola Donus")
            self.DonusAlgoritmasi("sol")
        elif(self.soldonusDurumu == True and self.inRange == True):
            print("Stop")
            self.DonusAlgoritmasi("dur")
            self.soldonusDurumu = False


        if(self.A_noktasi and (self.C_noktasi or self.B_noktasi) and (self.soldonusDurumu==False or self.sagdonusDurumu==None )):
            print(aX)
            self.Dengele(aX)


        cv2.imshow("Frame",frame)
        
        




def main(args = None):
    rclpy.init(args = args)
    intel_subscriber = IntelSubscriber()
    rclpy.spin(intel_subscriber)
    intel_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main":
    main()
