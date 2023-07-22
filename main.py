import cv2
from simple_facerec import SimpleFacerec
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

#Setup Motion Sensor
GPIO.setup(19,GPIO.IN)
GPIO.setup(26,GPIO.OUT)

#Setup LED 
GPIO.setup(27,GPIO.OUT)
GPIO.setup(17,GPIO.OUT)
GPIO.output(17, GPIO.LOW)
GPIO.output(27, GPIO.LOW)

#Setup Servo Motor
servoPIN = 22
GPIO.setup(servoPIN, GPIO.OUT)
p = GPIO.PWM(servoPIN, 50)
p.start(2.5) #start

#Setup Force Sensor
GPIO.setup(23,GPIO.IN)
prev_input = 0


# Load Camera
cap = cv2.VideoCapture(0)

#Setting size of frame
cap.set(3, 640)
cap.set(4, 480)

#Encode faces for images folder
sfr = SimpleFacerec()
sfr.load_encoding_images("images/")

#Setting flag for sensors
flag = 0
servoFlag = 0

while True:
    #waits for motion to be detected
    i = GPIO.input(19)
    if i == 0:
        GPIO.output(26,0)
        time.sleep(1)
        print("No Motion detected",i)
    else:
        print("Motion detected",i)
        GPIO.output(26,1)
        time.sleep(1)
        while True:
            #Motion is detected, waits for face to be detected
            ret, frame = cap.read()

            #Face Detected
            face_locations, face_names = sfr.detect_known_faces(frame)
            for face_loc, name in zip(face_locations, face_names):
                y1, x2, y2, x1 = face_loc[0], face_loc[1], face_loc[2], face_loc[3]
                
                cv2.putText(frame, name, (x1, y1), cv2.FONT_HERSHEY_DUPLEX, 1, (200,200,200), 2)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (200, 0, 0), 4)

                if name == 'Unknown':
                    print ("Unknown face detected (RED LED ON)")
                    GPIO.output(17, GPIO.HIGH)
                    time.sleep(.5)
                else:
                    while True:
                        input = GPIO.input(23)
                        flag = 1
                        if servoFlag == 0:
                            #Servo motor unlocked, green LED on
                            GPIO.output(17, GPIO.LOW)
                            print ("RED LED off")
                            GPIO.output(27,GPIO.HIGH)
                            print ("Green LED on")
                            p.ChangeDutyCycle(6)
                            servoFlag = 1
                            

                        if not prev_input and input:
                            #Pressure Detected, servo motor locked
                            print ("Pressure Detected")
                            p.ChangeDutyCycle(15)
                            break
                            
                        #update previous input
                        prev_input = input


            if servoFlag == 1:
                servoFlag = 0
                cv2.destroyAllWindows()
                break
            if flag == 1:
                GPIO.output(27,GPIO.LOW)
                print ("Green LED off")
                flag = 0

            GPIO.output(17, GPIO.LOW)
    



            cv2.imshow("Frame", frame)

            key = cv2.waitKey(1)
            if key == 27:
                break


cap.release()
