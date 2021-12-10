# catLaserPointer
This is a robot that tracks and points a laser for a cat.
It does this by using a raspberry pi 3B+ as the brain, 3d printed parts, two DF9GMS servos, a usb camera, and a laser.

It tracks the cat by using a python script with opencv and an SSD model. 
I followed this guide for the object detection: https://core-electronics.com.au/tutorials/object-identify-raspberry-pi.html 

The CAD files and code are here. 

Pins used on the Pi: 
pinLaser = 5    # Laser     
pinBase = 12    # Base servo
pinCam = 13     # Camera sevo

*NOTE: These are the GPIO.BCM pins
