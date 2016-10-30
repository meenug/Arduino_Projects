# ROS_Arduino_Projects

###NeoPix.ino
This program will fill a rectangular area on Neo pixel tile (8X8) with the color specified.

**Usage:**

Run roscore

$ roscore

Run the rosserial client application. Use the correct serial port: 

$ rosrun rosserial_python serial_node.py /dev/ttyACM0

To publish message

rostopic pub neopix_led std_msgs/String "Color startX startY endX endY" --once

"Color startX startY endX endY" each param value is separated by a single space. The x,y co-ordinates can be from 1 to 8.
where color is

R or r for Red,
G or g for Green,
B or b for Blue,
Y or y for Yellow

Any other letter than above for color will off the LED pixels

e.g

$ rostopic pub neopix_led std_msgs/String "R 1 1 4 5" --once


###ScrollText_Flexi.ino
Arduino only program for the specified text to scroll in red, blue and green color on Neo flexi strip of 8X32

###ScrollText_Tile.ino
Arduino only program for the specified text to scroll in red, blue and green color on Neopixel tile of 8X8
