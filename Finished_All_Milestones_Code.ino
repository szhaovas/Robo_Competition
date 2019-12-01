/*
   COGS 220 - Robot Competition
   Robot Name: Renegade
   Team: Steven, Shihan, & Wenxuan
*/

//Import libraries
#include <Servo.h>
#include <SPI.h>
#include <Pixy.h>
#include <Wire.h>
#include <SoftwareWire.h>
#include "Adafruit_TCS34725softi2c.h"

//Create the servo objects
Servo LeftWheel;
Servo RightWheel;

//Create camera object
Pixy camera;

//Create RGB1 object (center RGB)
Adafruit_TCS34725softi2c tcs = Adafruit_TCS34725softi2c(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X, 20, 21);

//Create RGB2 object (left RGB)
//Establish digital pins 22 & 24 as SDA and SCL pins, respectively
#define SDApin2 22
#define SCLpin2 24
Adafruit_TCS34725softi2c tcs2 = Adafruit_TCS34725softi2c(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X, SDApin2, SCLpin2);

//Create RGB3 object (right RGB)
//Establish digital pins 30 & 31 as SDA and SCL pins, respectively
#define SDApin3 30
#define SCLpin3 31
Adafruit_TCS34725softi2c tcs3 = Adafruit_TCS34725softi2c(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X, SDApin3, SCLpin3);

//Create the servo objects for the two arms
Servo LeftArm;
Servo RightArm;

//Arduino Input Pins
const int leftServoPin = 5;             //Wheel Pins
const int rightServoPin = 7;

const int larmPin = 8;                  //Arm Pins
const int rarmPin = 9;

const int LEDR = A7;                    //LED Pins
const int LEDG = A6;
const int LEDB = A5;
const int LEDR2 = A4;
const int LEDG2 = A3;
const int LEDB2 = A2;

//////////////////////GLOBAL VARIABLES///////////////////////////////////////////////////////

//Saves the current color the center RGB sees (Second Tape color)
int current = 0;

//Saves the previous color the center RGB sees (First Tape color)
int previous = 0;

//Variable that determines whether the block is in grasping range (0 = no, 1 = yes)
int hold = 0;

//Variable that determines which side of the robot hit the white boundary (left side = RGB2 & right side = RGB3)
int leftHit = 0;
int rightHit = 0;

//State Function for Homing (Value refers to color)
int regionA = 0;   //Home
int regionB = 0;   //Adjacent Left (Left hand side of A)
int regionC = 0;   //Adjacent Right (Right hand side of A)
int regionD = 0;   //Diagonal (Diagonal of A)

//Variable for whether the internal map has malfunctioned (0 = no, 1 = yes)
int internalError = 0;

//Has current been updated? (0 =
int current_updated = 0;

//Has previous been updated?
int previous_updated = 0;

//EscapeBack Activated (0 = no, 1 = yes)
int escapeBackActivated = 0;

//////////////////////////SETUP////////////////////////////////////////////////////////////////
void setup() {
  //initialize camera
  camera.init();

  //Initialize the RGB sensors
  tcs.begin();
  tcs2.begin();
  tcs3.begin();

  //Set the initial wheel servo positions
  LeftWheel.writeMicroseconds(1500);
  RightWheel.writeMicroseconds(1500);

  //Set the initial LED configurations;
  setColor(0, 0, 0);
  setColor2(0, 0, 0);

  //Set the Arduino Input & Output pins
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  pinMode(LEDR2, OUTPUT);
  pinMode(LEDG2, OUTPUT);
  pinMode(LEDB2, OUTPUT);

  //Attach the servo objects to the servos on the respective pins
  LeftWheel.attach(leftServoPin);
  RightWheel.attach(rightServoPin);
  LeftArm.attach(larmPin);
  RightArm.attach(rarmPin);
}

//////////////////////////////LOOP////////////////////////////////////////////////////////////////
void loop() {
  if (current != regionA) {
    dt(regionA);
  }
  else {
    Drive(50, 50, 1);
    armControl(1);
  }
}

/////////////////////////////////RGB HOMING SYSTEM///////////////////////////////////////////
void RGB_homing() {
  float turn_goal;

  current_updated = 0;
  previous_updated = 0;

  while ((!current_updated) || (!previous_updated)) {
    Drive(50, 50, 1);
  }

  if (current == regionB) {
    if (previous == regionA) {
      //turn 180
      Drive(50, -50, 7);
    } else if (previous == regionC) {
      //turn 90 L
      Drive(-50, 50, 3.5);
    } else if (previous == regionD) {
      //turn 60 L
      Drive(-50, 50, 2.5);
    }
  } else if (current == regionC) {
    if (previous == regionA) {
      //turn 180
      Drive(50, -50, 7);
    } else if (previous == regionB) {
      //turn 90 R
      Drive(50, -50, 3.5);
    } else if (previous == regionD) {
      //turn 60 R
      Drive(50, -50, 2.5);
    }
  } else if (current == regionD) {
    if (previous == regionA) {
      //turn 180
      Drive(50, -50, 7);
    } else if (previous == regionB) {
      //turn 120 R
      Drive(50, -50, 4.5);
    } else if (previous == regionC) {
      //turn 120 L
      Drive(-50, 50, 4.5);
    }
  }

  while (current != regionA) {
    RGB_homing();
  }
}



/////////////////////////////FUNCTION FOR CAMERA//////////////////////////////////////////////////////
//Input: one of the ints 1, 2, 3, or 4, representing colors red, green, blue, and yellow respectively
//Output: drive towards the specified color
void dt(int cl) {

  uint16_t blocks;
  int targetIndex;
  int targetX;
  int direct;
  float w;
  float h;
  float wh;
  int siz;

  //Close the arms
  armControl(1);

  //Fetch all color blocks for further processing
  blocks = camera.getBlocks();

  //The largest block will be the target
  float sizes[blocks];

  //Use x-coordinate to guide
  float x[blocks];

  //Initialize both sizes and x to be all zeroes
  for (int i = 0; i < blocks; i++) {
    sizes[i] = 0;
    x[i] = 0;
  }

  //If Robot does not see any blocks and is not holding anything
  if ((!blocks) && (hold == 0)) {
    randomDrive();
    setColor(0, 0, 0);
  }
  //If the robot is holding something
  else if (hold == 1) {
    RGB_homing();
    Drive(50, 50, 10);
    armControl(0);
    EscapeBack();
    hold = 0;
  }
  //If the robot sees blocks
  else {
    //get all blocks of the specified color
    for (int i = 0; i < blocks; i++) {
      w = (float)camera.blocks[i].width;
      h = (float)camera.blocks[i].height;
      siz = (camera.blocks[i].width * camera.blocks[i].height);
      wh = w / h;
      if ((camera.blocks[i].signature == cl) && (wh < 1.3) && (siz < 8000)) {
        sizes[i] = siz;
        x[i] = camera.blocks[i].x;
      }
    }

    if (sizes[getMax(sizes)] >= 1800) {
      hold = 1;
      armControl(0);
      Drive(50, 50, 10);
      armControl(1);
    }

    targetIndex = getMax(sizes);
    targetX = x[targetIndex];
    if (targetX) {
      //x ranges between 0 and 319, so midpoint is 159
      //store targetX-159 as reference for Drive()
      direct = targetX - 159;
      if (direct > 0 && (abs(direct) > 10)) {
        Drive((direct * 50 / 159), 0, 1);
      } else if (direct < 0 && (abs(direct) > 10)) {
        Drive(0, -direct * 50 / 159, 1);
      } else {
        Drive(30, 30, 1);
      }

      //Set LED color
      switch (cl) {
        case 0:
          setColor(0, 0, 0);
          break;
        case 1:
          setColor(255, 0, 0);
          break;
        case 2:
          setColor(0, 255, 0);
          break;
        case 3:
          setColor(0, 0, 255);
          break;
        case 4:
          setColor(0, 255, 255);
          break;
      }
    }
    else {
      randomDrive();
    }
  }
  delay(20);
}

//Helper function used to get the minimum index from a list
int getMax(float* array)
{
  int maxIndex = 0;
  int size = sizeof(array);
  int maximum = array[0];
  for (int i = 0; i < size; i++)
  {
    if (array[i] > maximum) {
      maximum = array[i];
      maxIndex = i;
    }
  }
  return maxIndex;
}
////////////////////////////////MAIN DRIVE FUNCTION/////////////////////////////////////////////
void Drive(float ls, float rs, float d) {
  /*
     Drive Key:
     -50 < ls & rs < 50
     d is in microseconds
  */
  //----------------------------------------------------------------------------------------
  //For RGB123
  //----------------------------------------------------------------------------------------

  uint16_t clear, red, green, blue, clear2, red2, green2, blue2, clear3, red3, green3, blue3;

  tcs.getRawData(&red, &green, &blue, &clear);

  //If RGB Sees Red
  if (red > 1700 && green < 1500 && blue < 1500) {
    if (current != 1) {
      previous = current;
      current = 1;
      current_updated = 1;
      previous_updated = 1;
    }
    if (regionA == 0) {
      regionA = 1;
      regionB = 2;
      regionC = 3;
      regionD = 4;
      amIHome = 0;
    }
  }
  //If RGB Sees Green
  else if (green > 3500 && red < 4000 && blue < 4000) {
    if (current != 2) {
      previous = current;
      current = 2;
      current_updated = 1;
      previous_updated = 1;
    }
    if (regionA == 0) {
      regionA = 2;
      regionB = 4;
      regionC = 1;
      regionD = 3;
      amIHome = 0;
    }
  }
  //If RGB Sees Blue
  else if (blue > 3300 && red < 2800 && green < 7600) {
    if (current != 3) {
      previous = current;
      current = 3;
      current_updated = 1;
      previous_updated = 1;
    }
    if (regionA == 0) {
      regionA = 3;
      regionB = 1;
      regionC = 4;
      regionD = 2;
      amIHome = 0;
    }
  }
  If RGB Sees Yellow
  else if (red > 4000 && green < 12000 && blue < 4600) {
    if (current != 4) {
      previous = current;
      current = 4;
      current_updated = 1;
      previous_updated = 1;
    }
    if (regionA == 0) {
      regionA = 4;
      regionB = 3;
      regionC = 2;
      regionD = 1;
      amIHome = 0;
    }
  }

  //Side RGBs
  tcs2.getRawData(&red2, &green2, &blue2, &clear2);

  if (red2 > 2000 && green2 > 2500 && blue2 > 2000) {
    leftHit = 1;
    if (!escapeBackActivated) {
      EscapeBack();
    }
  }

  tcs3.getRawData(&red3, &green3, &blue3, &clear3);

  if (red3 > 2000 && green3 > 2500 && blue3 > 2000) {
    rightHit = 1;
    if (!escapeBackActivated) {
      EscapeBack();
    }
  }

  escapeBackActivated = 0;

  //---------------------------------------------------------------------------------------
  //For disp
  //---------------------------------------------------------------------------------------

  if (current == 1) {
    setColor2(255, 0, 0);
  } else if (current == 2) {
    setColor2(0, 255, 0);
  } else if (current == 3) {
    setColor2(0, 0, 255);
  } else if (current == 4) {
    setColor2(255, 255, 0);
  } else {
    setColor2(0, 0, 0);
  }

  //---------------------------------------------------------------------------------------
  //For the actual "drive"
  //---------------------------------------------------------------------------------------
  ls = -ls;
  rs = rs;

  LeftWheel.writeMicroseconds(1500 + 6 * ls);
  RightWheel.writeMicroseconds(1500 + 5.6 * rs);

  if (d == 1) {
    delay(d);
  } else {
    delay(d * 160);
  }
}

//////////////////////////////////OTHER DRIVE FUNCTIONS///////////////////////////////////////
void randomDrive() {
  Drive(random(30, 50), random(30, 50), 1);
}

void EscapeBack() {
  escapeBackActivated = 1;
  setColor2(255, 255, 255);
  Drive(-50, -50, 10);
  //Turn a random degree to avoid hitting into the edge again
  int rand = 50;
  if (!rightHit) {
    Drive(-rand, rand, 5);
  } else {
    Drive(rand, -rand, 5);
  }
  leftHit = 0;
  rightHit = 0;
}

///////////////////////////////FUNCTIONS FOR LEDS/////////////////////////////////////////////////
/*
   LED Key:
   red = 255, 0 , 0
   green = 0, 255, 0
   blue = 0, 0, 255
   yellow = 255, 255, 0
*/

//Set and display the color of the home quadrant
void setColor(int red, int green, int blue) {
  red = 255 - red;
  green = 255 - green;
  blue = 255 - blue;
  analogWrite(LEDR, red);
  analogWrite(LEDG, green);
  analogWrite(LEDB, blue);
}

//Set and display the color of the quadrant it is currently in
void setColor2(int red, int green, int blue) {
  red = 255 - red;
  green = 255 - green;
  blue = 255 - blue;
  analogWrite(LEDR2, red);
  analogWrite(LEDG2, green);
  analogWrite(LEDB2, blue);
}

////////////////////////////////////ARM CONTROL///////////////////////////////////////////////
//Input 0 for open and 1 for close
void armControl(int mode) {
  //Open arms
  if (mode == 0) {
    LeftArm.write(60);
    RightArm.write(120);
  }
  //Close arms
  else if (mode == 1) {
    LeftArm.write(125);
    RightArm.write(55);
  }
}




