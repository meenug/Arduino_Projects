####Note: ####

You might need to use Arduino MEGA for this. As number of LEDs increased, it needed more memory and got flaky on UNO. Also, for NeoPix I used external power.

##NeoPix_Flexi.ino###

Code to subscribe Adafruit's Neopixel Flexi (8X32)
 * rosserial Subscriber  - NeoPixel Flexi Flash
 * Flash NeoPix Flexi on 1 second interval

###Usage: ###
####Run####

On terminal 1 
      
      $ roscore

On terminal 2 (specify your arduino port)
      
      $ rosrun rosserial_python serial_node.py /dev/ttyACM0

On terminal 3 
      
      $ rostopic pub neo_flexi nasa_robot/NeoFlexi \"ON\" 255 0 0 1 1 8 32 --once
          \"ON\" 255 0 0 3 5 6 7 - Flash(ON/OFF/PAUSE)  red green blue startx starty endx endy 
                   255 0 0  3 5 6 7 - red green blue startx starty endx endy 
          red, green,blue value is [0-255]
             startx/y, endx/y ( it's 8X32 matrix, 8 rows - x value, and 32 columns - y value)
