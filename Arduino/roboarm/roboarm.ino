#include <G15.h>    // include the library

#define LED_BOARD 13
#define SPEED_SLOW 25
#define SPEED_MEDIUM 200
#define SPEED_FAST 500

#define LINK1 105 // 105 mm
#define LINK2 75  // 75 mm

// Servo angle limits
#define OFF_THETA1 -270 // -270 degrees
#define MIN_THETA1 210 // 210 degrees
#define MAX_THETA1 330 // 330 degrees
#define OFF_THETA2 -360 // -360 degrees
#define MIN_THETA2 -90 // -90 degrees
#define MAX_THETA2 90 // 90 degrees

// Canvas offsets (X and Y offsets form the origin of our coordinate system)
#define X_OFFSET 80 // 80 mm
#define Y_OFFSET -75 // -75 mm
#define X_MIN 80 // 80 mm
#define Y_MIN -75 // -75 mm
#define X_MAX 160 // 160 mm
#define Y_MAX 75 // 75 mm

/*Return Error definitions & Mask
=====Higher 8 bits of Word=========
packet length error :       0x0100
packet header error:        0x0200
ID mismatch error:          0x0400
packet checksum mismatch :  0x0800
====Lower 8 bits of word==========
Error status return by G15:
INST			0x0040		
OVERLOAD		0x0020
CHECKSUM		0x0010
RANGE			0x0008
OVERHEAT		0x0004
ANGLELIMIT 	        0x0002
VOLTAGE		        0x0001
*/


//declaration of variables & object
word ERROR=0;
byte DATA[10]; 
word STATUS;

//declare G15 Class Object
//servo1 ID=1
G15 servo1(1);
G15 servo2(2);
G15 servo3(3);

//--------------------------------------------------------------------------------------
// Arduino setup.
//--------------------------------------------------------------------------------------
void setup()
{  
  word stat;
  
  Serial.begin(9600);
  
  //initialize the arduino main board's serial/UART and Control Pins
  G15ShieldInit(19200,3,8); 
  
  //call the init function to init the servo objs
  servo1.init();
  servo2.init();
  servo3.init();

  stat = servo1.SetTorqueOnOff(OFF,iWRITE_DATA);   //turned off servo1's torque immediately. 
  if(STATUS != 0)
  {
    Serial.print("ERROR TURNING OFF TORQUE");
  }

  //init LED indicator as output
  pinMode(LED_BOARD,OUTPUT);  
  digitalWrite(LED_BOARD, LOW); 
  
  delay(500); 
  digitalWrite(LED_BOARD, HIGH);
  
  arminit();
  penup();
  delay(5000);
}

//--------------------------------------------------------------------------------------
// Main loop.
//--------------------------------------------------------------------------------------
void loop()
{    
  byte data[4];
  word stat;
  
  STATUS = servo1.SetLED(ON,iWRITE_DATA);
  STATUS = servo2.SetLED(ON,iWRITE_DATA);
  STATUS = servo3.SetLED(ON,iWRITE_DATA);
  
  // Go to origin
  moveto(0,0);
  //delay(5000);
  //goto(0,40);
  arminit();
  //servo1.SetPosSpeed(ConvertAngle2Pos(240), SPEED_SLOW, iWRITE_DATA);
  //servo1.SetPosSpeed(ConvertAngle2Pos(300), SPEED_SLOW, iWRITE_DATA);
  delay(5000);
  moveto(100,75);
  delay(5000);
}

//--------------------------------------------------------------------------------------
// Bring the pen down to paper.
//--------------------------------------------------------------------------------------
void pendown()
{
  servo3.SetPosSpeed(ConvertAngle2Pos(75), SPEED_SLOW, iWRITE_DATA);
  delay(1000);
}

//--------------------------------------------------------------------------------------
// Bring the pen up from the paper.
//--------------------------------------------------------------------------------------
void penup()
{
  servo3.SetPosSpeed(ConvertAngle2Pos(0), SPEED_SLOW, iWRITE_DATA);
  delay(1000);
}

//--------------------------------------------------------------------------------------
// Initialize the arm for drawing.
//--------------------------------------------------------------------------------------
void arminit() {
  word angle;
  word stat,pos=0;
  byte data[4];

  stat = servo1.GetPos(data); //get the current position from servo1
  pos = data[0];
  pos|=word(data[1])<<8;
  angle = ConvertPos2Angle(pos);
  Serial.print("Servo 1 Angle: ");
  Serial.print(angle);
  Serial.print("\n");
  
  stat = servo2.GetPos(data); //get the current position from servo2
  pos = data[0];
  pos|=word(data[1])<<8;
  angle = ConvertPos2Angle(pos);
  Serial.print("Servo 2 Angle: ");
  Serial.print(angle);
  Serial.print("\n");
}

//--------------------------------------------------------------------------------------
// Given X and Y coordinates, performs inverse kinematics and moves arm to location.
// X and Y should be based on a (0,0) origin.
//--------------------------------------------------------------------------------------
void moveto(float x, float y) {
  // Adjust X and Y values for offsets
  x = x + X_OFFSET;
  y = y + Y_OFFSET;
  
  // Threshold X and Y values
  if(x > X_MAX) {
    x = X_MAX;
  }
  if(x < X_MIN) {
    x = X_MIN;
  }
  if(y > Y_MAX) {
    y = Y_MAX;
  }
  if(y < Y_MIN) {
    y = Y_MIN;
  }
  
  float B2 = (x*x) + (y*y);
  float q1 = atan2(y,x);
  float q2 = acos(((LINK1*LINK1)-(LINK2*LINK2)+B2)/(2*LINK1*sqrt(B2))); 
  float theta1 = q1 + q2;
  float theta2 = acos(((LINK1*LINK1)+(LINK2*LINK2)-B2)/(2*LINK1*LINK2));
  
  theta1 *= 180/3.14159;
  theta2 *= 180/3.14159;
  //theta2 -= 180;
  
  movearm(theta1, theta2);
}

//--------------------------------------------------------------------------------------
// Move robot arm based on given angles (keeping within limits)
//--------------------------------------------------------------------------------------
void movearm(int theta1, int theta2) {
  // Convert thetas to our orientation
  theta1 = (theta1 + OFF_THETA1) * (-1);
  theta2 = (theta2 + OFF_THETA2) * (-1);
 
  // Max threshold theta1
  if(theta1 > MAX_THETA1)
  {
    theta1 = MAX_THETA1;
  }
  // Min threshold theta1
  if(theta1 < MIN_THETA1)
  {
    theta1 = MIN_THETA1;
  }
  // Max threshold theta2
  if(theta2 > MAX_THETA2)
  {
    theta2 = MAX_THETA2;
  }
  // Min threshold theta2
  if(theta2 < MIN_THETA2)
  {
    theta2 = MIN_THETA2;
  }
  
  servo1.SetPosSpeed(ConvertAngle2Pos(theta1), SPEED_SLOW, iREG_WRITE);
  servo2.SetPosSpeed(ConvertAngle2Pos(theta2), SPEED_SLOW, iREG_WRITE);
  delay(1000);
  G15::SetAction();
}

