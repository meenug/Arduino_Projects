/* 
 * rosserial Subscriber
 * NeoPix LED Matrix to fill the area with color on callback
 * Usage: Run
 *    On terminal 1 
 *     $ roscore
 *    On terminal 2 (specify your arduino port)
 *     $ rosrun rosserial_python serial_node.py /dev/ttyACM0
 *    On terminal 3 
 *     $ rostopic pub neopix_led std_msgs/String "y 3 5 6 7" --once
 *     y 3 5 6 7 - color startx starty endx endy separated by a single space
 *        Color supported are R/r- Red, G/g - Green, B/b - Blue, Y/y - Yellow and any other char will clear the area.
 */


#include <ros.h>
#include <std_msgs/String.h>

// AdaFruit stuff
#include <Adafruit_NeoPixel.h>

#ifndef PSTR
 #define PSTR // Make Arduino happy
#endif

#define PIN 6 // arduino PIN connected to Neopixel matrix
#define NUM_PIXELS 64

#define START_PIXELS_IN_ROW 1
#define START_PIXELS_IN_COL 1

#define END_PIXELS_IN_ROW 8
#define END_PIXELS_IN_COL 8

#define NUM_PIXELS_IN_ROW 8
#define NUM_PIXELS_IN_COL 8

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_PIXELS, PIN, NEO_GRB + NEO_KHZ800);

ros::NodeHandle  nh;

int pass = 0;
int  pixelPerChar = 6;

// callback function: Specify the color and begin and end co-ordinates and it will fill the area with
// the color
void messageCb( const std_msgs::String& led_msg){
  
  // check if msg is len is 9 char. Data: "Color startrow startcol endrow endcol" eg. "R 1 2 3 4": startrow starts from arduino pin connector (as top left) and goes across (top right)
  // rostopic pub neopix_led std_msgs/String "R 0 0 2 2" --once

  int len = strlen ((const char* ) led_msg.data);
  char color; // color to display
  
  int startX; // start co-ordinate
  int startY;
  int endX;   // end co-ordinate
  int endY;

  // check for the length of the string
  if (len == 9) {
    color = led_msg.data[0];
    
    // convert to integer
    startX = led_msg.data[2] - '0';
    startY = led_msg.data[4] - '0';   
    
    endX = led_msg.data[6] - '0';
    endY = led_msg.data[8] - '0';   

    if ( (startX >= START_PIXELS_IN_ROW && startX <= END_PIXELS_IN_COL) && 
       (startY >= START_PIXELS_IN_ROW && startY <= END_PIXELS_IN_COL) && 
       (endX >= START_PIXELS_IN_ROW   && endX <= END_PIXELS_IN_COL)   && 
       (endY >= START_PIXELS_IN_ROW   && endY <= END_PIXELS_IN_COL) ) { 

      uint32_t myColor;
      if (color == 'R' || color == 'r')
        myColor = strip.Color(255, 0, 0); // red
      else if (color == 'G' || color == 'g')
        myColor = strip.Color(0, 255, 0); // green
      else if (color == 'B'  || color == 'b')
        myColor = strip.Color(0, 0, 255); // blue
      else if (color == 'Y' || color == 'y')
        myColor = strip.Color(255, 255, 0); // yellow
      else // white
        myColor = strip.Color(0, 0, 0); // white
    
      int startrow = startX;
      int startcol = startY;
     
      int endrow = endX;
      int endcol = endY;
    
      for (int i=startrow-1; i <= endrow-1;i++){ // first NeoPixel is 0 and so on from arduino connector
        for (int j= (NUM_PIXELS_IN_ROW*i + startcol-1); j<= (NUM_PIXELS_IN_ROW*i + endcol-1);j++){
          strip.setPixelColor(j, myColor);
          strip.show();
          delay(50);
        } // for
      } // for
    }   // if ( (startX >= START_PIXELS_IN_ROW && startY <= END_PIXELS_IN_COL) && 
  } // if (len == 9)
  
} // void messageCb( const std_msgs::String& led_msg){

ros::Subscriber<std_msgs::String> sub("neopix_led", &messageCb );

void setup()
{ 
  // Ada fruit stuff
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  nh.initNode();
  nh.subscribe(sub);
}


void loop()
{  
  nh.spinOnce();
  delay(1);
}

