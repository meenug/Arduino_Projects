/* Code to subscribe Adafruit's Neopixel and Relay Channel
 *  Written by: Meenu Goswami
 *  
 * rosserial Subscriber 1 - NeoPixel LED Matrix
 * NeoPix LED Matrix to fill the area with color on callback
 * Usage: Run
 *    On terminal 1 
 *     $ roscore
 *    On terminal 2 (specify your arduino port)
 *     $ rosrun rosserial_python serial_node.py /dev/ttyACM0
 *    On terminal 3 
 *     $ rostopic pub neopix_led nasa_robot/NeoPixel 255 0 0 3 5 6 7 --once
 *     255 0 0  3 5 6 7 - red green blue startx starty endx endy 
 *     red, green,blue value is [0-255]
 *     startx/y, endx/y value is 1-8
 */


/* 
 * rosserial Subscriber 2 - Relay Channels
 * Relay ON/OFF on callback
 * Usage: Run
 *    On terminal 1 
 *     $ roscore
 *    On terminal 2 (specify your arduino port)
 *     $ rosrun rosserial_python serial_node.py /dev/ttyACM0
 *    On terminal 3 
 *     $ rostopic pub relay_switch nasa_robot/relay_channel "<relayNum> <ON/OFF/FLASHON/FLASHOFF>" --once
 *     eg. $ rostopic pub relay_switch nasa_robot/relay_channel 1 \"ON\" --once
 */

#include <ros.h>
#include <nasa_robot/NeoPixel.h>
#include <nasa_robot/relay_channel.h>

// AdaFruit stuff
#include <Adafruit_NeoPixel.h>

#ifndef PSTR
 #define PSTR // Make Arduino happy
#endif

#define PIN 5                 // arduino PIN connected to Neopixel matrix
#define NUM_PIXELS 64

#define START_PIXELS_IN_ROW 1
#define START_PIXELS_IN_COL 1
#define END_PIXELS_IN_ROW 8
#define END_PIXELS_IN_COL 8

#define NUM_PIXELS_IN_ROW 8
#define NUM_PIXELS_IN_COL 8

// NOTE: There is issue with startup of Arduino programs that controls these relays.
// All of 2,4,8 relay boards input control are Active LOW, means setting pin LOW turns them ON
#define RELAY_ON 0
#define RELAY_OFF 1

#define RELAY_CHANNELS 4      // total channels on relay board
#define RELAY_START_CHANNEL 0 // starting with 0 for 1 channel; 4 means starting from 5th channel.
                              // Useful if testing few channels at a time. To test channels 5 to 8
                              // set this to 4, change RELAY_CHANNELS to 4 and set array relayNum with
                              // 4 pins initialized

// Arduino Digital I/O pin number for relay channels
byte relayNum[RELAY_CHANNELS]={ 6, 7, 8, 9};
//byte relayNum[RELAY_CHANNELS]={ 6, 7, 8, 9, 10, 11, 12, 13};
byte relayPin;
boolean relayFlash = false;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_PIXELS, PIN, NEO_GRB + NEO_KHZ800);

ros::NodeHandle  nh; 

// NeoPixel callback function: Specify the color and begin and end co-ordinates and it will fill the area with
// the color
void messageCb( const nasa_robot::NeoPixel& msg ){
  
  // startrow starts from arduino pin connector (as top left) and goes across (top right)
  // rostopic pub neopix_led custom_param_ros/NeoPixelStr R 0 0 2 2 --once

  byte red;   // red color value 0-255
  byte green; // green
  byte blue;  // blue

  byte startX; // start co-ordinate
  byte startY;
  byte endX;   // end co-ordinate
  byte endY;
  
  //color  = msg.color[0];   

  red    = msg.red;
  green  = msg.green;
  blue   = msg.blue;
  
  startX = msg.startX;
  startY = msg.startY;

  endX   = msg.endX;
  endY   = msg.endY;

  if ( (startX >= START_PIXELS_IN_ROW && startX <= END_PIXELS_IN_COL) && 
       (startY >= START_PIXELS_IN_ROW && startY <= END_PIXELS_IN_COL) && 
       (endX >= START_PIXELS_IN_ROW   && endX <= END_PIXELS_IN_COL)   && 
       (endY >= START_PIXELS_IN_ROW   && endY <= END_PIXELS_IN_COL) ) { 
      
      uint32_t myColor;
      myColor = strip.Color(red, green, blue);
          
      byte startrow = startX;
      byte startcol = startY;

      byte endrow = endX;
      byte endcol = endY;
      
      for (byte i=startrow-1; i <= endrow-1;i++){ // first NeoPixel is 0 and so on from arduino connector
      
        for (byte j= (NUM_PIXELS_IN_ROW*i + startcol-1); j<= (NUM_PIXELS_IN_ROW*i + endcol-1);j++){
          strip.setPixelColor(j, myColor);
        } // for
      } // for
      strip.show();
   }   // if ( (startX >= START_PIXELS_IN_ROW && startY <= END_PIXELS_IN_COL) && 
      
} // void messageCb( const nasa_robot::NeoPixel& msg ){

ros::Subscriber<nasa_robot::NeoPixel> sub1("neopix_led", &messageCb );


// Relay Callback function
// format <relay number> <ON/OFF/FLASHON/FLASHOFF> eg. 1 \"ON\" or 1 \"OFF\"
void relaySwitchCb( const nasa_robot::relay_channel& relay_msg){

  relayPin = relayNum[relay_msg.channel - (RELAY_START_CHANNEL + 1)];

  if ( strcasecmp((const char* ) relay_msg.mode, "ON") == 0){ 
      relayFlash = false;
      digitalWrite(relayPin, RELAY_ON);// set the Relay ON

  }
  else if ( strcasecmp((const char* ) relay_msg.mode, "OFF") == 0){
      relayFlash = false;
      digitalWrite(relayPin, RELAY_OFF);// set the Relay OFF

  }
  else if ( strcasecmp((const char* ) relay_msg.mode, "FLASHON") == 0){
      relayFlash = true;
  }
  else if ( strcasecmp((const char* ) relay_msg.mode, "FLASHOFF") == 0){      
      relayFlash = false;
      digitalWrite(relayPin, RELAY_OFF);// set the Relay flashing OFF

  }
  
} // void relaySwitchCb( const nasa_robot::relay_channel& relay_msg){

ros::Subscriber<nasa_robot::relay_channel> sub2("relay_switch", &relaySwitchCb );

void setup()
{    

  // Ada fruit stuff
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  nh.initNode();       

  nh.subscribe(sub1); // NeoPixel
  nh.subscribe(sub2); // Relay

  // set relay to inactive at init. This board is active to begin with.
  // Initialize the Arduino data pins for OUTPUT
  byte i=0;
  while (i < RELAY_CHANNELS) {

    digitalWrite(relayNum[i], RELAY_OFF); // HIGH
    pinMode(relayNum[i], OUTPUT);  
    i++;
  }

} // void setup()
 
unsigned long last_change_time = 0; 
void loop()
{
  nh.spinOnce(); 
  // handle relay flashing here
    if (relayFlash){
      if (millis() - last_change_time > 500) {
          last_change_time = millis();
          flash_on = !flash_on;
          if (flash_on) {
            digitalWrite(relayPin, RELAY_ON);
          } else {
            digitalWrite(relayPin, RELAY_OFF);
          }
      }
  }
  delay(10);
 
} // void loop()


