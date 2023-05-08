import cv2
import numpy as np
import time
import gtts
import pyttsx3
print("project started")
import RPi.GPIO as GPIO
#thres = 0.45 # Threshold to detect object
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
# for ultrasonic sensor use in front
GPIO_ECHO_US1 = 24

GPIO_TRIG_US1 = 18
GPIO_ECHO_US2 = 25

GPIO_TRIG_US2 = 23

GPIO_ECHO_US3 = 27

GPIO_TRIG_US3 = 22 

GPIO.setup(GPIO_ECHO_US1, GPIO.IN)

GPIO.setup(GPIO_TRIG_US1, GPIO.OUT)

GPIO.setup(GPIO_ECHO_US2, GPIO.IN)
GPIO.setup(GPIO_TRIG_US2, GPIO.OUT)

GPIO.setup(GPIO_ECHO_US3, GPIO.IN)

GPIO.setup(GPIO_TRIG_US3, GPIO.OUT)


classNames = []
classFile = "/home/pi/Desktop/Object_Detection_Files/coco.names"
with open(classFile,"rt") as f:
    classNames = f.read().rstrip("\n").split("\n")

configPath = "/home/pi/Desktop/Object_Detection_Files/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
weightsPath = "/home/pi/Desktop/Object_Detection_Files/frozen_inference_graph.pb"

net = cv2.dnn_DetectionModel(weightsPath,configPath)
net.setInputSize(320,320)
net.setInputScale(1.0/ 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)


def getObjects(img, thres, nms, draw=True, objects=[]):
    classIds, confs, bbox = net.detect(img,confThreshold=thres,nmsThreshold=nms)
    #print(classIds,bbox)
    if len(objects) == 0: objects = classNames
    objectInfo =[]
    if len(classIds) != 0:
        for classId, confidence,box in zip(classIds.flatten(),confs.flatten(),bbox):
            className = classNames[classId - 1]
            if className in objects:
                objectInfo.append([box,className])
                if (draw):
                    cv2.rectangle(img,box,color=(0,255,0),thickness=2)
                    cv2.putText(img,classNames[classId-1].upper(),(box[0]+10,box[1]+30),
                    cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)
                    cv2.putText(img,str(round(confidence*100,2)),(box[0]+200,box[1]+30),
                    cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)
                print(className)
                engine = pyttsx3.init()
                engine.say(className)
            # play the speech
                engine.runAndWait()
    return img,objectInfo


if __name__ == "__main__":

    cap = cv2.VideoCapture(0)
    cap.set(3,640)
    cap.set(4,480)
    #cap.set(10,70)


    while True:
        success, img = cap.read()
        result, objectInfo = getObjects(img,0.45,0.2)
        #print(objectInfo)
        cv2.imshow("Output",img)
        cv2.waitKey(1)
        
        
        
        GPIO.output(GPIO_TRIG_US1, GPIO.LOW)

        time.sleep(2)
        GPIO.output(GPIO_TRIG_US1, GPIO.HIGH)

        time.sleep(0.00001)

        GPIO.output(GPIO_TRIG_US1, GPIO.LOW)

        while GPIO.input(GPIO_ECHO_US1) == 0:
            start_time = time.time()

        while GPIO.input(GPIO_ECHO_US1) == 1:
            Bounce_back_time = time.time()

        pulse_duration = Bounce_back_time - start_time

        distance_front = round(pulse_duration * 17150, 2)

        print ("Obstacle distance in front : ",distance_front,"cm")
        if (distance_front < 50 ):
            print("alert object near in front")
            engine = pyttsx3.init()
            engine.setProperty('rate', 125) 
            engine.say("object in front at less than 50cm", )
            engine.runAndWait()
    # for ultrasonic sensor use in left
        GPIO.output(GPIO_TRIG_US2, GPIO.LOW)

        time.sleep(2)
        GPIO.output(GPIO_TRIG_US2, GPIO.HIGH)

        time.sleep(0.00001)

        GPIO.output(GPIO_TRIG_US2, GPIO.LOW)

        while GPIO.input(GPIO_ECHO_US2) == 0:
            start_time = time.time()

        while GPIO.input(GPIO_ECHO_US2) == 1:
            Bounce_back_time = time.time()

        pulse_duration = Bounce_back_time - start_time

        distance_left = round(pulse_duration * 17150, 2)

        print ("Obstacle distance in left : ",distance_left,"cm")
        if (distance_left < 50 ):
            print("alert object near in left")
            engine = pyttsx3.init()
            engine.setProperty('rate', 125) 
            engine.say("object in left at less than 50 centi-meter", )
            engine.runAndWait()
    
    # for ultrasonic sensor use in right
        GPIO.output(GPIO_TRIG_US3, GPIO.LOW)

        time.sleep(2)
        GPIO.output(GPIO_TRIG_US3, GPIO.HIGH)

        time.sleep(0.00001)

        GPIO.output(GPIO_TRIG_US3, GPIO.LOW)

        while GPIO.input(GPIO_ECHO_US3) == 0:
            start_time = time.time()

        while GPIO.input(GPIO_ECHO_US3) == 1:
            Bounce_back_time = time.time()

        pulse_duration = Bounce_back_time - start_time

        distance_right = round(pulse_duration * 17150, 2)

        print ("Obstacle distance in right : ",distance_right,"cm")
        if (distance_right < 50 ):
            print("alert object near in right")
            engine = pyttsx3.init()
            engine.setProperty('rate', 125) 
            engine.say("object in right at less than 50 centi-meter", )
            engine.runAndWait()
