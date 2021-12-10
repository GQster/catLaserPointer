#****************************************************************
#   Coder: Grant Croft (gxc9140)
#   RIT 585 Principles of Robotics final project
#   This is a cat laser pointer project and can be found on: https://github.com/grantcroft/catLaserPointer/blob/main/README.md 
#   *This is a stripped down version of the code meant to run on any computer as it only does the object detection. 
#****************************************************************

import cv2


# Imave Control Varibales
imageW = 640
imageH = 480
camFPS = 30#15#
imageCapW = 160#320#177#
imageCapH = 120#240#144#

classFile = "/Users/grant/Documents/objDetection/coco.names"
weightsPath = "/Users/grant/Documents/objDetection/frozen_inference_graph.pb"
configPath = "/Users/grant/Documents/objDetection/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
classNames = []
timeCatLastSeen = 0.0

with open(classFile,"rt") as f:
    classNames = f.read().rstrip("\n").split("\n")


net = cv2.dnn_DetectionModel(weightsPath,configPath)
net.setInputSize(imageCapW,imageCapH)
net.setInputScale(1.0/ 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)


def findQuad(objCenterX, objCenterY):
    global imageW, imageH

    x_half = (int(imageW/2))
    y_half = (int(imageH/2))
    if (objCenterX >= x_half):
        if (objCenterY <= y_half):                      # quad #1
            quadrant = 1
            print("Animal is too high")
            print("Animal is too far RIGHT")
        elif(objCenterY > y_half):                      # quad #4
            quadrant = 4
            print("Animal is too low")
            print("Animal is too far RIGHT")

    elif (objCenterX < x_half):
        if (objCenterY <= y_half):                      # quad #2
            quadrant = 2
            print("Animal is too high")
            print("Animal is too far LEFT")
        elif (objCenterY > y_half):                     # quad #3
            quadrant = 3
            print("Animal is too low")
            print("Animal is too far LEFT")
    else:
        print("ERROR, INVALID CENTER LOCATION")
        quadrant = 0

    return quadrant


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
    return


def getObjects(img, thres, nms, draw=True, objects=[]):
    global imageW, imageH, timeCatLastSeen

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

                numOfObjDetected += 1
                if numOfObjDetected == 2:                       # Makes it return only set num of objs
                    break       #WORKS, BUT MIGHT NOT BE THE OBJ YOU WANT

                if (draw):

                    drawOnImg('box', img, classId, confidence, 0, 0, box)       # Draw bounding box
                    drawOnImg('center', img, classId, confidence, 0, 0, box)    # Find center of obj
                    drawOnImg('line', img, classId, confidence, 0, 0)                     # Draw quad grid
    return img,objectInfo


def clean_up():
        """Cleans up resources"""
        print('Shutting down')
        try:
            if cap:
                cap.release()                                   # Release software & hardware resources for 'cap'
                print("Camera released")

        except Exception:
            print('Error while shutting down')




if __name__ == "__main__":
    #Setup:
    print("Hello, starting up the cat laser pointer!")

    print("Turning on camera")
    cap = cv2.VideoCapture(0)
    cap.set(3,imageCapW)                                           # Width of the frames in the video stream.
    cap.set(4,imageCapH)                                           # Height of the frames in the video stream.
    cap.set(5, camFPS)                                              # Frame rate.
    # https://docs.opencv.org/3.0-beta/modules/videoio/doc/reading_and_writing_video.html#videocapture-set      <<< VideoCapture doc


    numObj = 0.99 #nms:  nms percentage value, increasing this will limit the amount of simultaneously identified objects
    thresh = 0.45 # Threshold to detect object

    print("Program read")
    while True:
        try:
            success, img = cap.read()
            img = cv2.resize(img, (imageW, imageH))
            result, objectInfo = getObjects(img, thresh, numObj, objects=['cat', 'dog', 'person'])       # 'cat', 'dog', 'person'
            cv2.imshow("Output",img)
            cv2.waitKey(1)

        except KeyboardInterrupt:
            print('Exiting...')
            clean_up()
