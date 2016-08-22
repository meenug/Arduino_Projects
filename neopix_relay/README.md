
The code has 2 subscribers - to Adafruit's Neopixel and to 8 channel Relay. 
It uses the custom message structures NeoPixel.msg and relay_channel.msg.

Subscriber1 - NeoPix LED Matrix to fill the area with color on callback

**Usage:** 
Run
 *    On terminal 1 
 
      $ roscore
 *    On terminal 2 (specify your arduino port)
 
      $ rosrun rosserial_python serial_node.py /dev/ttyACM0
 *    On terminal 3 
 
      $ rostopic pub neopix_led nasa_robot/NeoPixel 255 0 0 3 5 6 7 --once
 
      255 0 0  3 5 6 7 - red green blue startx starty endx endy 
 
      red, green,blue value is [0-255]
 
      startx/y, endx/y value is 1-10

Subscriber 2 - Relay Channels. Relay ON/OFF on callback
Run
 *    On terminal 1 
 
      $ roscore
 *    On terminal 2 (specify your arduino port)
 
      $ rosrun rosserial_python serial_node.py /dev/ttyACM0
 *    On terminal 3 
 
      $ rostopic pub relay_switch nasa_robot/relay_channel "<relayNum> <ON/OFF/FLASHON/FLASHOFF>" --once
 
      eg. $ rostopic pub relay_switch nasa_robot/relay_channel 1 \"ON\" --once
