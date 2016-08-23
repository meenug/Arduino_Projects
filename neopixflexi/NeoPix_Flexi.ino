/* Code to subscribe Adafruit's Neopixel Flexi (8X32)
 *  Written by: Meenu Goswami
 *  
 * rosserial Subscriber  - NeoPixel Flexi Flash
 * NeoPix Flexi to flash on 1 second interval
 * Usage: Run
 *    On terminal 1 
 *     $ roscore
 *    On terminal 2 (specify your arduino port)
 *     $ rosrun rosserial_python serial_node.py /dev/ttyACM0
 *    On terminal 3 
 *     $ rostopic pub neo_flexi nasa_robot/NeoFlexi \"ON\" 255 0 0 1 1 8 32 --once
 *     \"ON\" 255 0 0 3 5 6 7 - Flash(ON/OFF/PAUSE)  red green blue startx starty endx endy 
 *     *     255 0 0  3 5 6 7 - red green blue startx starty endx endy 
 *     red, green,blue value is [0-255]
 *     startx/y, endx/y ( it's 8X32 matrix, 8 rows - x value, and 32 columns - y value)
 *     
 */

#include <ros.h>
#include <nasa_robot/NeoFlexi.h>

// AdaFruit stuff
#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>

#ifndef PSTR
 #define PSTR // Make Arduino happy
#endif

// NeoPixel Flexi settings
#define FLEXI_PIN 4

#define MIN_PIXELS_IN_ROW 1
#define MIN_PIXELS_IN_COL 1
#define MAX_PIXELS_IN_ROW 8
#define MAX_PIXELS_IN_COL 32

Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(32, 8, FLEXI_PIN,
  NEO_MATRIX_TOP     + NEO_MATRIX_LEFT +
  NEO_MATRIX_COLUMNS + NEO_MATRIX_ZIGZAG,
  NEO_GRB            + NEO_KHZ800);

ros::NodeHandle  nh; 


// Neopixel Flexi
byte red;   // red color value 0-255
byte green; // green
byte blue;  // blue
byte startX; // start co-ordinate
byte startY;
byte endX;   // end co-ordinate
byte endY;

boolean bFlash = false;
boolean bColor = false;


// NeoPixel Flexi Flash callback function: 
void neoFlexiCb( const nasa_robot::NeoFlexi& msg ){
  red    = msg.red;
  green  = msg.green;
  blue   = msg.blue;

  startX = msg.start_x;
  startY = msg.start_y;

  endX   = msg.end_x;
  endY   = msg.end_y;
  
  if ( strcasecmp((const char* ) msg.flash_switch, "ON") == 0){ 
    bFlash = true;
  }
  else if ( strcasecmp((const char* ) msg.flash_switch, "OFF") == 0){ 
    bFlash = false;
    flash(0, 0 );
  }
  else if ( strcasecmp((const char* ) msg.flash_switch, "PAUSE") == 0){ 
    bFlash = false;
    flash(matrix.Color(red, green, blue), 2 );
  }

} // void neoFlexiCb( const nasa_robot::NeoFlexi& msg ){

ros::Subscriber<nasa_robot::NeoFlexi> sub("neo_flexi", &neoFlexiCb );

void setup()
{    

  // neopixel flexi
  matrix.begin();

  nh.initNode();       

  nh.subscribe(sub); // NeoPixel Flexi

} // void setup()

unsigned long last_change_flexi_time = 0; 

void loop()
{
  nh.spinOnce(); 

  // Neo Flexi - handle flash here
  if (bFlash){ // start flash 
    if (millis() - last_change_flexi_time > 1000) { // delay for 1 sec
      last_change_flexi_time = millis();
      flash(matrix.Color(red, green, blue), 1);
    } 
  }    
  delay(10);

} // void loop()

static void flash(uint32_t c, byte flashStatus) {
  if ( (startX >= MIN_PIXELS_IN_ROW && startX <= MAX_PIXELS_IN_ROW) && 
       (startY >= MIN_PIXELS_IN_COL   && startY <= MAX_PIXELS_IN_COL)   && 
       (endX >= MIN_PIXELS_IN_ROW && endX <= MAX_PIXELS_IN_ROW) && 
       (endY >= MIN_PIXELS_IN_COL   && endY <= MAX_PIXELS_IN_COL) ) { 

    if (flashStatus == 2){ // pause
      for (byte i=startX-1; i <= endX-1;i++){ // first NeoPixel is 0 and so on from arduino connector
        for (byte j= startY-1; j<= endY-1;j++){
          matrix.drawPixel(j, i, c);
        } // for
      } // for
      
    }
    else if (flashStatus == 1){ // flash on
      if (bColor == false){ // display color
        bColor = true;
        for (byte i=startX-1; i <= endX-1;i++){ 
          for (byte j= startY-1; j<= endY-1;j++){
            // col, row, color
            matrix.drawPixel(j, i, c); 
          } // for
        } // for
      } // if
      else{ // display no color, to give flash effect
        bColor = false;
        for (byte i=startX-1; i <= endX-1;i++){ 
          for (byte j= startY-1; j<= endY-1;j++){
            // col, row, color
            matrix.drawPixel(j, i, 0);
          } // for
        } // for
      }
    } 
    else if (flashStatus == 0){ // flash off
      for (byte i=startX-1; i <= endX-1;i++){ 
        for (byte j= startY-1; j<= endY-1;j++){
          // col, row, color
          matrix.drawPixel(j, i, 0);
        } // for
      } // for
      
    }
    matrix.show();
  } // if ( (startX >= MIN_PIXELS_IN_ROW && startX <= MAX_PIXELS_IN_ROW) && 
} // static void flash(uint32_t c, byte flashStatus) {


