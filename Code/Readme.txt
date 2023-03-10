Readme

Three versions: 1) v2CatPointer.py 
                2) v2CatPointerLAPTOPVERSION.py
                3) *full laptop version*

1) The first uses the GPIO of the raspberry pi 3b so will not run on a normal computer. 
It uses the following libraries:
    cv2
    RPi.GPIO as GPIO
    time

2) The second is a stripped version of the first with all GPIO references removed. 
This means it will not move the motors or control the laser. 
It will still output the captured video; displaying the detected object, its center, its quadrant, and what it is telling the motors to do.
This will run on a normal computer with the following  libraries:
    cv2
    
    
    





NOTE*: Reworked v2CatPointer.py to fully run on a laptop using a teensy to control the motors. Can be found here: https://github.com/grantcroft/AutoTracker/blob/main/autoTracker.py
