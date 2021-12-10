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


Basically the code works by detecting the object, drawing a bounding box on the object, finding its center, then trying to center the object in the image by using the servos. 
Right when the object is detected it turns on the laser. 
After "waitForScan" seconds (10 right now) the robot turns off the laser and scanns the room for objects. 


With my current setup there is about a 3.5 second lag. This is down from ~33 seconds.

Improvments:
  -Decrease lag, 
  -Improve motor contorl to allow for bigger steps depending on how far off center the object is from the image
  -Different robot physical design to increase its workspace


*NOTE: These are the GPIO.BCM pins
**NOTE: Down around line 340- 350 of "v2CatPointer.py" you can set which objects the robot should detect in "objects= ['cat', 'dog', 'person'] of getObjects() and searchforobject(). There are many different options saved in the "coco.names" file. 
***NOTE: I measured and cadded all the models so they might not be exact. That said I'm fairly sure they are accurate as everything works when basing the measurments off them.
