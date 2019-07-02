import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)

class xymove(object): # buttom 4 dc motors for moving
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(7, GPIO.OUT)
        GPIO.setup(11, GPIO.OUT)
        GPIO.setup(13, GPIO.OUT)
        GPIO.setup(15, GPIO.OUT)

    def forward(self,tf):
        self.__init__()
        GPIO.output(7, False)
        GPIO.output(11, True)
        GPIO.output(13, True)
        GPIO.output(15, False)
        time.sleep(tf)
        GPIO.cleanup()

    def reverse(self,tf):
        self.__init__()
        GPIO.output(7,True)
        GPIO.output(11,False)
        GPIO.output(13,False)
        GPIO.output(15,True)
        time.sleep(tf)
        GPIO.cleanup()

    def turnLeft(self,tf):
        self.__init__()
        GPIO.output(7, True)
        GPIO.output(11, True)
        GPIO.output(13, True)
        GPIO.output(15, False)
        time.sleep(tf)
        GPIO.cleanup()

    def turnRight(self,tf):
        self.__init__()
        GPIO.output(7, False)
        GPIO.output(11, False)
        GPIO.output(13, False)
        GPIO.output(15, True)
        time.sleep(tf)
        GPIO.cleanup()

    def pivotLeft(self,tf):
        self.__init__()
        GPIO.output(7, True)
        GPIO.output(11, False)
        GPIO.output(13, True)
        GPIO.output(15, False)
        time.sleep(tf)
        GPIO.cleanup()

    def pivotRight(self,tf):
        self.__init__()
        GPIO.output(7, False)
        GPIO.output(11, True)
        GPIO.output(13, False)
        GPIO.output(15 ,True)
        time.sleep(tf)
        GPIO.cleanup()

class PWMServo: #up and down movement
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(33, GPIO.OUT)
        self.pwm = GPIO.PWM(33, 50)
        self.pwm.start(6)

    def change_duty_cycle(self, duty):
        self.pwm.ChangeDutyCycle(duty)
        time.sleep(0.3)

class ZMove(PWMServo):

    def __init__(self):
        super().__init__()
        self.pwm.start(6)

    def rise(self,angle):
        duty=float(angle)/18.0+2.0
        self.change_duty_cycle(duty)
        self.pwm.stop()
        GPIO.cleanup()

class dcmove(object): #middle dc motor for seeking with camera
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(36,GPIO.OUT)
        GPIO.setup(38,GPIO.OUT)
        GPIO.setup(32,GPIO.OUT)
        self.pwm = GPIO.PWM(32, 100)
        self.pwm.start(20)

    def forward(self,tf):
        self.__init__()
        GPIO.output(36,GPIO.HIGH)
        GPIO.output(38,GPIO.LOW)
        GPIO.output(32,GPIO.HIGH)
        time.sleep(tf)
        self.pwm.stop()
        GPIO.cleanup()

    def reverse(self,tf):
        self.__init__()
        GPIO.output(36,GPIO.LOW)
        GPIO.output(38,GPIO.HIGH)
        GPIO.output(32,GPIO.HIGH)
        time.sleep(tf)
        self.pwm.stop()	
        GPIO.cleanup()
        
class distance(object): #Hc-sr04 distance sensor
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(16, GPIO.OUT) #trigger pin
        GPIO.setup(18, GPIO.IN) #echo pin

    def measure(self):
        self.__init__()
        GPIO.output(16, False)

        GPIO.output(16, True)
        time.sleep(0.00001)
        GPIO.output(16, False)

        while GPIO.input(18)==0:
            pulse_start = time.time()

        while GPIO.input(18)==1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start

        distance = pulse_duration * 17150
        distance = round(distance, 2)
        print(distance-0.5)
        if distance > 2 and distance < 400:
            return distance - 0.5 #cm
        else:
            print("out of range!")
            return 36 # makeshift for my autonomous code
class Colour(object):
    def __init__(self):
        self.camera = PiCamera()
        self.camera.resolution = (480, 320)
        self.camera.framerate = 30
        self.rawCapture = PiRGBArray(self.camera, size=(480, 320))

    def detect_red(self):
        time.sleep(0.5)
        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
            image = frame.array
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            #Red Color Range
            low=np.array([140,150,0])
            high=np.array([180,255,255])
            mask=cv2.inRange(hsv,low,high)
            count=0
            left=0
            right=0
            for i in range(320): #searching white pixels in the mask
                for j in range(480):
                    if np.all(mask[i, j] == (255, 255, 255)):
                        count +=1
                        if j<160:
                            right +=1 #reverse
                        elif j>320:
                            left +=1
            middle=count-(left+right)
    
            if count>200:
                if left>right:
                    if left>middle:
                        self.rawCapture.truncate(0)
                        return ["red","left"]
                    else:
                        self.rawCapture.truncate(0)
                        return ["red","middle"]
                else:
                    if right>middle:
                        self.rawCapture.truncate(0)
                        return ["red","right"]
                    else:
                        self.rawCapture.truncate(0)
                        return ["red","middle"]
            self.rawCapture.truncate(0)
            return [0,0]
            
        cv2.destroyAllWindows()

    def detect_blu(self):
        time.sleep(0.5)
        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
            image = frame.array
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            #Blue Color Range
            low=np.array([94,80,2])
            high=np.array([126,255,255])
            mask=cv2.inRange(hsv,low,high)
            count=0
            left=0
            right=0
            for i in range(320):
                for j in range(480):
                    if np.all(mask[i, j] == (255, 255, 255)):
                        count +=1
                        if j<160:
                            right +=1
                        elif j>320:
                            left +=1
            middle=count-(left+right)
    
            if count>100:
                if left>right:
                    if left>middle:
                        self.rawCapture.truncate(0)
                        return ["blue","left"]
                    else:
                        self.rawCapture.truncate(0)
                        return ["blue","middle"]
                else:
                    if right>middle:
                        self.rawCapture.truncate(0)
                        return ["blue","right"]
                    else:
                        self.rawCapture.truncate(0)
                        return ["blue","middle"]
            self.rawCapture.truncate(0)
            return [0,0]
            
        cv2.destroyAllWindows()


    def detect_grn(self):
        time.sleep(0.5)
        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
            image = frame.array
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            #Green Color Range
            low=np.array([25,125,72])
            high=np.array([100,255,255])
            mask=cv2.inRange(hsv,low,high)
            count=0
            left=0
            right=0
            for i in range(320):
                for j in range(480):
                    if np.all(mask[i, j] == (255, 255, 255)):
                        count +=1
                        if j<160:
                            right +=1
                        elif j>320:
                            left +=1
            middle=count-(left+right)
    
            if count>200:
                if left>right:
                    if left>middle:
                        self.rawCapture.truncate(0)
                        return ["green","left"]
                    else:
                        self.rawCapture.truncate(0)
                        return ["green","middle"]
                else:
                    if right>middle:
                        self.rawCapture.truncate(0)
                        return ["green","right"]
                    else:
                        self.rawCapture.truncate(0)
                        return ["green","middle"]
            self.rawCapture.truncate(0)
            return [0,0]
            
        cv2.destroyAllWindows()

class ldr(object): #light sensor
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(31, GPIO.IN)
        
    def measure(self):
        self.__init__()
        if GPIO.input(31):
            print("dark environment")
        else:
            print("bright environment")
        
class pir(object): #movement sensor
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(35, GPIO.IN)
        
    def measure(self):
        self.__init__()
        if GPIO.input(35):
            print("motion detected!")
        else:
            print("no motion")
        
class ntc(object): #heat sensor
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(37, GPIO.IN)
        
    def measure(self):
        self.__init__()
        if GPIO.input(37):
            print("cool environment")
        else:
            print("hot environment")

if __name__ == '__main__':
    
    xy=xymove() #bottom 4 dc motors for moving 4 directions
    dc=dcmove() #middle dc motor for searching colours
    z=ZMove() #servo motor
    dist=distance() #distance sensor
    cl=Colour() # image processing class
    l=ldr()
    p=pir()
    n=ntc()
    xy.forward(0.1)
    dc.reverse(0.1)

#	Not Autonomous Code
    x=1
    while x!=0:
        a=input("xy,dc,z=? ")
        t=float(input("Time or Angle? "))
        
        if a=="z":
            z.rise(100/18+2)
        elif a=="dc":
            c=int(input("1-forward,2-reverse=? "))
            if c==1:
                dc.forward(t)
            elif c==2:
                dc.reverse(t)
        elif a=="xy":
            d=int(input("1-forward,2-reverse,3-turnleft,4-turnright=? "))
            if d==1:
                xy.forward(t)
            elif d==2:
                xy.reverse(t)
            elif d==3:
                xy.turnLeft(t)
            elif d==4:
                xy.turnRight(t)
        
        x=int(input("for exit press 0 "))
        
#	Autonomous Code
        
    xyt45=0.4 #45 degree turning time for vehicle
    dct45=0.8 #45 degree turning time for dc motor and camera
    speed=55  #cm in 1 second
    count=0   #place of dc motor and camera 1: 45 degree left 2: 45 degree right
    a=0       # 0: target's not detected 1: target's detected

    while True:
        L=cl.detect_red()
        if L[0]=="red": #moving
            
            if L[1]=="left":
                xy.turnLeft(xyt45)
            elif L[1]=="right":
                xy.turnRight(xyt45)
                
            while dist.measure()>35:
                d=dist.measure()
                t=d/speed
                xy.forward(t/2) #moving half the way
                K=cl.detect_red()
                if K[0]!="red":
                    break

            if K[0]=="red" and dist.measure()<=35:
                p.measure()#Sensor
                count=0
                break                
    
        else:#seeking
            dc.forward(dct45)
            L=cl.detect_red()
            if L[0]=="red":
                a=1
                count=1
            if a!=1:
                dc.reverse(2*dct45)
                L=cl.detect_red()
                if L[0]=="red":
                    a=1
                    count=2
            if a==1:#correcting position of camera and vehicle
                if count==1:
                    dc.reverse(dct45)
                    xy.turnLeft(xyt45)
                else:
                    dc.forward(dct45)
                    xy.turnRight(xyt45)
            else:
                print("Red couldn't be detected!")
                break
            
    if count==2:
        dc.forward(dct45)
    elif count==1:
        dc.reverse(dct45)
    count=0
    a=0

    xy.reverse(3.5) # taking a good position for second target
    xy.turnLeft(2)
    speed=48
    
    while True:
        L=cl.detect_blu()
        if L[0]=="blue":

            if L[1]=="left":
                xy.turnLeft(xyt45)
            elif L[1]=="right":
                xy.turnRight(xyt45)

            while dist.measure()>35:
                d=dist.measure()
                t=d/speed
                xy.forward(t/2)
                K=cl.detect_blu()
                if K[0]!="blue":
                    break

            if K[0]=="blue" and dist.measure()<=35:
                l.measure()#Sensor
                count=0
                break

        else:
            dc.forward(dct45)
            L=cl.detect_blu()
            if L[0]=="blue":
                a=1
                count=1
            if a!=1:
                dc.reverse(2*dct45)
                L=cl.detect_blu()
                if L[0]=="blue":
                    a=1
                    count=2
            if a==1:
                if count==1:
                    dc.reverse(dct45)
                    xy.turnLeft(xyt45)
                else:
                    dc.forward(dct45)
                    xy.turnRight(xyt45)
            else:
                print("Blue couldn't be detected!")
                break
    if count==2:
        dc.forward(dct45)
    elif count==1:
        dc.reverse(dct45)
    a=0
    count=0
    
    xy.turnLeft(1.5)
    xy.forward(3)
    while True:
        L=cl.detect_grn()
        if L[0]=="green":

            if L[1]=="left":
                xy.turnLeft(xyt45)
            elif L[1]=="right":
                xy.turnRight(xyt45)

            while dist.measure()>35:
                d=dist.measure()
                t=d/speed
                xy.forward(t/2)
                K=cl.detect_grn()
                if K[0]!="green":
                    break

            if K[0]=="green" and dist.measure()<=35:
                z.change_duty_cycle(90/18+2)#Sensor
                n.measure()
                count=0
                break

        else:
            dc.forward(dct45)
            L=cl.detect_grn()
            if L[0]=="green":
                a=1
                count=1
            if a!=1:
                dc.reverse(2*dct45)
                L=cl.detect_grn()
                if L[0]=="green":
                    a=1
                    count=2
            if a==1:
                if count==1:
                    dc.reverse(dct45)
                    xy.turnLeft(xyt45)
                else:
                    dc.forward(dct45)
                    xy.turnRight(xyt45)
            else:
                print("Green  couldn't be detected!")
                break
    if count==2:
        dc.forward(dct45)
    elif count==1:
        dc.reverse(dct45)
