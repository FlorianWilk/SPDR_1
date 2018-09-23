# SPDR_1

SPiDeR ONE - prototype of an autonomous robot which uses 
a combination of Hector-SLAM/OctoMapping and Kalman-Filters to navigate in unknown environments.  

![SPDR_ONE](IMG_20180616_164342800.jpg)

Hardware:
  - Freenove Quadruped Robot Kit including acrylic parts, servos and the Arduino Mega Servo-Board. 100€
  - a Raspberry PI Zero W - 30€
  - RPI fisheye cam - 20€
  - SD-Card - i use 32GB - ?
  - SSD1306 OLED Display - 8€
  - RPLIDAR A1 - 100€
  - MPU 9250 - gyro, accel, compass 
  - at least 2x (i use 4x+) 14500 750mAh Batteries - don't try to use standard batteries 
  - cables and some soldering tools
  
  - in my case i use cablebinder and rubberbands to attach the LIDAR and all the other parts on the chassis.
 
The Freenove Robot Kit is a perfect start for building up a small autonomous robot. We could make it much more easier by using a wheel based bot, but i love this creepy spider-style when it crawles around. 

![Freenove](http://www.freenove.com/images/logo2.png)


The Kit provides an Arduino Mega Controller Board which with Servo-Controllers onBoard. 
We will use this Board - as it is by default - as Controller-Board for all the body movement. 
We will modify/replace the Freenove Software to make our robot act much more natural and veeery creepy.

We will use diffenent types of software and programming languages when realizing all this:

  - the Arduino will be programmed using Processing and very simple C++ but with a lot of funny mathstuff
  - the Raspberry is programmed by some shell-skripts, python and ROS - ROS is a very very extensive "Toolset" which provides 
  solutions for quite complicate problems such as SLAM (simultanous locating and mapping) - in our case using a 360° LIDAR (light/laser based range detection) and a wide angle camera.
  
  
## Building it

First of all, build up the Freenove Robot Kit. Calibrate it and start playing with it. This will take you some time.. :)
Now the custom part:

<to be done>
