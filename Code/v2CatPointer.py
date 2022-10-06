#****************************************************************
#   Coder: Grant Croft (gxc9140)
#   RIT 585 Principles of Robotics final project
#   This is a cat laser pointer project and can be found on: https://github.com/grantcroft/catLaserPointer/blob/main/README.md 
#
#	For slightly less lag, comment out the .imshow and .waitKey on lines 344 & 345
#****************************************************************
import cv2
import RPi.GPIO as GPIO                                         # Import RPi.GPIO module
import time

# GPIO Pins
pinLaser = 5                                                    # Pin 5 is GPIO 5
pinBase = 12
pinCam = 13

# Motor Control Variables
startAngleC = 70        #45 for wal mount, 90 for floor
camAngle = startAngleC                                          # Keeps track of camera servo angle
startAngleB = 90
baseAngle = startAngleB                                         # Keeps track of camera servo angle
camChange = 2                                                   # How big each sep it (degrees)
baseChange = 2                                                  # How big each sep it (degrees)
baseSearchChange = 5
camUpperBound = 100                                             # Upper bound on camera motor
moveWaitTime = 0.02                                             # How long program waits for servos to move

# Imave Control Varibales
imageW = 640
imageH = 480
camFPS = 30#15#
imageCapW = 160#320#177#
imageCapH = 120#240#144#

# searchforobject() Variables
noObjFound = False
leftSideFLAG = False
rightSideFLAG = False
waitForScan = 10                                                                                                # How long to wait before starting to scan the area for the object

classFile = "/home/pi/Projects/objDetection/coco.names"
weightsPath = "/home/pi/Projects/objDetection/frozen_inference_graph.pb"
configPath = "/home/pi/Projects/objDetection/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
classNames = []
timeCatLastSeen = 0.0

with open(classFile,"rt") as f:
    classNames = f.read().rstrip("\n").split("\n")


net = cv2.dnn_DetectionModel(weightsPath,configPath)
net.setInputSize(imageCapW,imageCapH)
net.setInputScale(1.0/ 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)








def GPIOSetup():
    global pinLaser, pinCam, pinBase

    print("Setting up the GPIO")
    GPIO.setmode(GPIO.BCM)                                      # Choose BCM (GPIO #) or BOARD  (# on board [DIFFERENT PER BOARD])
    GPIO.setwarnings(False)                                     # Disable GPIO Warnings
    GPIO.setup(pinLaser, GPIO.OUT)                              # Set a port/pin as an output
    GPIO.output(pinLaser, 1)                                    # Start with laser off

    print("Going to Starting point")
    GPIO.setup(pinCam,GPIO.OUT)
    servoCam = GPIO.PWM(pinCam, 50)
    servoCam.start(0)
    servoCam.ChangeDutyCycle(2+(startAngleC/18))                # Start camera at 45 degrees (parallel with mounting surface)
    time.sleep(0.15)                                            # Let motor move
    servoCam.ChangeDutyCycle(0)

    GPIO.setup(pinBase,GPIO.OUT)
    servoBase = GPIO.PWM(pinBase, 50)
    servoBase.start(0)
    servoBase.ChangeDutyCycle(2+(startAngleB/18))               # Start camera at 90 degrees
    time.sleep(0.15)                                            # Let motor move
    servoBase.ChangeDutyCycle(0)

    return servoCam, servoBase


def laserOnOff(onOff, pin):   # Turns laser on and off with '0' and '1' input
        if onOff == 0:
                GPIO.output(pin, 1)                             # Turn OFF
        elif onOff == 1:
                GPIO.output(pin, 0)                             # Turn ON
        return


def findQuad(objCenterX, objCenterY):
    global imageW, imageH

    x_half = (int(imageW/2))
    y_half = (int(imageH/2))
    if (objCenterX >= x_half):
        if (objCenterY <= y_half):                                      # quad #1
            quadrant = 1
            print("Animal is too high")
            print("Animal is too far RIGHT")
        elif(objCenterY > y_half):                                      # quad #4
            quadrant = 4
            print("Animal is too low")
            print("Animal is too far RIGHT")

    elif (objCenterX < x_half):
        if (objCenterY <= y_half):                                      # quad #2
            quadrant = 2
            print("Animal is too high")
            print("Animal is too far LEFT")
        elif (objCenterY > y_half):                                     # quad #3
            quadrant = 3
            print("Animal is too low")
            print("Animal is too far LEFT")
    else:
        print("ERROR, INVALID CENTER LOCATION")
        quadrant = 0

    return quadrant


def motorMove(Xc, Yc, servoC, servoB, quadSector, allowMotion=True):     # Moves the two motors
    global camAngle, baseAngle, camChange, baseChange, camUpperBound, moveWaitTime

    if allowMotion:
                # Cam servo contorl
        if Yc != imageH/2:
            if quadSector == 1 or quadSector == 2:
                camAngle -= camChange                           # decrease servo angle
                if camAngle <= 0:
                    camAngle = 1
                    print("Cam angle boundary: 0 deg")
                servoC.ChangeDutyCycle(2+(camAngle/18))         # Change cam angel
                time.sleep(moveWaitTime)                        # Let motor move
                servoC.ChangeDutyCycle(0)
            elif quadSector == 3 or quadSector == 4:
                camAngle += camChange                           # increase servo angle
                if camAngle >= camUpperBound:
                    camAngle = camUpperBound
                    print("Cam angle boundary: ", camUpperBound, " deg")
                servoC.ChangeDutyCycle(2+(camAngle/18))         # Change cam angel
                time.sleep(moveWaitTime)                        # Let motor move
                servoC.ChangeDutyCycle(0)


                # Base servo control
        if Xc != imageW/2:
            if quadSector == 1 or quadSector == 4:
                baseAngle -= baseChange                         # decrease servo angle
                if baseAngle <= 0:
                    baseAngle = 1
                    print("Base angle boundary: 0 deg")
                servoB.ChangeDutyCycle(2+(baseAngle/18))        # Change base angel
                time.sleep(moveWaitTime)                        # Let motor move
                servoB.ChangeDutyCycle(0)

            elif quadSector == 2 or quadSector == 3:
                baseAngle += baseChange                         # increase servo angle
                if baseAngle >= 180:
                    baseAngle = 179
                    print("Base angle boundary: 180 deg")
                servoB.ChangeDutyCycle(2+(baseAngle/18))        # Change base angel
                time.sleep(moveWaitTime)                        # Let motor move
                servoB.ChangeDutyCycle(0)

    else:
        print("Motion not allowed")
        servoB.ChangeDutyCycle(0)
        servoC.ChangeDutyCycle(0)
    return

def searchforobject(img, thres, nms, servoC, servoB, draw=True, objects=[]):
    global baseAngle, baseChange, moveWaitTime, timeCatLastSeen, leftSideFLAG, rightSideFLAG
        #global camAngle, camChange, camUpperBound,

    if baseAngle <= baseSearchChange:
        print("Left point reached")
        leftSideFLAG = True
    if baseAngle >= (180-baseSearchChange):
        print("Right point reached")
        rightSideFLAG = True

    if leftSideFLAG and rightSideFLAG:
        print("Area searched, no object found.")
        rightSideFLAG = False
        leftSideFLAG = False

    if leftSideFLAG:                                                                    # Search going right
        baseAngle += baseSearchChange                   # Increase servo angle
        servoB.ChangeDutyCycle(2+(baseAngle/18))        # Change base angel
        time.sleep(moveWaitTime)                        # Let motor move
        servoB.ChangeDutyCycle(0)
        print("Searching right for object. Base angle: {}".format(baseAngle))

    elif baseAngle > baseSearchChange:                                  # Search going left
        baseAngle -= baseSearchChange                   # decrease servo angle
        servoB.ChangeDutyCycle(2+(baseAngle/18))        # Change base angel
        time.sleep(moveWaitTime)                        # Let motor move
        servoB.ChangeDutyCycle(0)
        print("Searching left for object. Base angle: {}".format(baseAngle))

    return


def drawOnImg(lineORCenter, pic, classId, confidence, servoC, servoB, box=0):
    if lineORCenter == 'box':
        global classNames
        cv2.rectangle(pic,box,color=(0,255,0),thickness=2)
        cv2.putText(pic,classNames[classId-1].upper(),(box[0]+10,box[1]+30),
        cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)
        cv2.putText(pic,str(round(confidence*100,2)),(box[0]+200,box[1]+30),
        cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)

    if lineORCenter == 'line':                              # Draw grid on image
        global imageW, imageH

        x_half, x_L, x_R = int(imageW/2), 0, imageW
        y_half, y_bot, y_top = int(imageH/2), 0, imageH
        cv2.line(pic, (x_half, y_bot), (x_half, y_top), (0, 0, 200), thickness=1)
        cv2.line(pic, (x_L, y_half), (x_R, y_half), (0, 0, 200), thickness=1)

    if lineORCenter == 'center':                            # Finds center of object
        Xtl, Ytl = box[:2]
        Xcenter = Xtl + (box[2]/2)
        Ycenter = Ytl + (box[3]/2)
        center_coordinates = (int(Xcenter), int(Ycenter))
        cv2.circle(pic, center_coordinates, radius=5, color=(255, 255, 0), thickness=5)
        #print(Xcenter ," <<<X, Y>>> " , Ycenter)
        quad = findQuad(Xcenter, Ycenter)
        print(quad)
        motorMove(Xcenter, Ycenter, servoC, servoB, quad, True)        # Moves the motors
    return


def getObjects(img, thres, nms, motorY, motorX, draw=True, objects=[]):
    global imageW, imageH, timeCatLastSeen, noObjFound

    classIds, confs, bbox = net.detect(img,confThreshold=thres,nmsThreshold=nms)
    #print(classIds,bbox)

    if len(objects) == 0: objects = classNames
    objectInfo =[]
    if len(classIds) != 0:
        numOfObjDetected = 0
        for classId, confidence,box in zip(classIds.flatten(),confs.flatten(),bbox):
            className = classNames[classId - 1]
            if className in objects:
                objectInfo.append([box,className])
                noObjFound = False                                                      # Saves that it found an obj
                timeCatLastSeen = time.time()                   # Updates when the cat was last seen
                laserOnOff(1, pinLaser)                         # Turn ON

                numOfObjDetected += 1
                if numOfObjDetected == 2:                       # Makes it return only set num of objs
                    break       #WORKS, BUT MIGHT NOT BE THE OBJ YOU WANT

                if (draw):

                    drawOnImg('box', img, classId, confidence, 0, 0, box)       # Draw bounding box
                    drawOnImg('center', img, classId, confidence, motorY, motorX, box)    # Find center of obj
                    drawOnImg('line', img, classId, confidence, 0, 0)                     # Draw quad grid

    return img,objectInfo


def clean_up(servoC, servoB):                                                                   #Cleans up resources

    print('Shutting down')
    try:
        if cap:
            cap.release()                                   # Release software & hardware resources for 'cap'
            print("Camera released")

        laserOnOff(0, pinLaser)                             #Turn off laser
        print("Laser powered off")

        servoC.ChangeDutyCycle(2+(startAngleC/18))          # Start camera at 90 degrees (parallel with mounting surface)
        time.sleep(0.15)                                    # Let motor move
        servoC.ChangeDutyCycle(0)
        print("Cam position reset")

        servoB.ChangeDutyCycle(2+(startAngleB/18))          # Start camera at 90 degrees (parallel with mounting surface)
        time.sleep(0.15)                                    # Let motor move
        servoB.ChangeDutyCycle(0)
        print("Base motor powered off")

        GPIO.cleanup()
        print("GPIO clean up finished!!! Bye\n\n")

    except Exception:
        print('Error while shutting down')
    return



if __name__ == "__main__":
    #Setup:
    print("Hello, starting up the cat laser pointer!")
    sCamObj, sBaseObj = GPIOSetup()                             # Creating Camera Servo object and DC motor object

    print("Turning on camera")
    cap = cv2.VideoCapture(0)
    cap.set(3,imageCapW)                                        # Width of the frames in the video stream.
    cap.set(4,imageCapH)                                        # Height of the frames in the video stream.
    cap.set(5, camFPS)                                          # Frame rate.
    # https://docs.opencv.org/3.0-beta/modules/videoio/doc/reading_and_writing_video.html#videocapture-set      <<< VideoCapture doc

    # Info on USB probe camera being used:
    #$ v4l2-ctl -d /dev/video0 --list-formats-ext
    # OUTPUT: [0]: 'YUYV' (YUYV 4:2:2) Resolution: 640x480, 352x288, 320x240, 176x144, 160x120 All at 15 & 30 FPS

#Test Rsutls:   #using .imshow slows it down about a second
    # RES       LAG 30 FPS      LAG 15 FPS
    #640x480    33 sec          34 sec
    #352x288    13 sec          13.7 sec
    #320x240    8.3 sec         8.0
    #176x144    3.75            3.5                     TOO SMALL?
    #160x120    3.0             2.9                     TOO SMALL?

    #BEST: imageCapWH = 160x120, imageW = 640x480; FPS 30: 3.9sec, FPS 15: 3.9sec





    numObj = 0.99 #nms:  nms percentage value, increasing this will limit the amount of simultaneously identified objects
    thresh = 0.45 # Threshold to detect object

    print("Program read")
    while True:
        try:
            success, img = cap.read()
            img = cv2.resize(img, (imageW, imageH))
            result, objectInfo = getObjects(img, thresh, numObj, sCamObj, sBaseObj, objects=['cat', 'dog', 'person'])   # 'cat', 'dog', 'person'
            cv2.imshow("Output",img)
            cv2.waitKey(1)

            if (time.time() - waitForScan) > timeCatLastSeen:   # Checks if the cat was seen within "waitForScan" seconds
                noObjFound = True
                laserOnOff(0, pinLaser)                         # Turn off laser
                searchforobject(img, thresh, numObj, sCamObj, sBaseObj, objects=['cat', 'dog', 'person'])                               # 'cat', 'dog', 'person'
#                motorMove(0, 0, sCamObj, sBaseObj, 0, False)   # Stop Motors

            if noObjFound:
                print("Object not found for ", int(time.time()-timeCatLastSeen), " seconds.")                   # DOESNT WORK QUITE RIGHT... IF NEVER FOUND OBJECT PRINTS HUGE NUMBER


        except KeyboardInterrupt:
            print('Exiting...')
            clean_up(sCamObj, sBaseObj)

