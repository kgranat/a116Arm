/***********************************************************************************
 *  }--\     InterbotiX Robotic Arm            /--{
 *      |       Playback Code                 |
 *   __/                                       \__
 *  |__|                                       |__|
 *
 *
 *  The following sketch will continously playback the sequence defned in 
 *  armSequence.h . This sequence can be generated from the Arm Link Software
 *  or written by hand.
 *
 *
 *  WIRING
 *
 *    Digital Inputs
 *      Digital 2 - Button 1
 *
 *=============================================================================
 * Based upon Kurt's PX Reactor arm code.
 * https://github.com/KurtE
 * This code provides serial control of the Interbotix line of robotic arms, which are sold by Trossen Robotics:
 * http://www.trossenrobotics.com/robotic-arms.aspx
 * http://learn.trossenrobotics.com/interbotix/robot-arms
 *=============================================================================
 * 
 *   This code is a Work In Progress and is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
 ****************************************************************************************************************/

//=============================================================================
// Define Options
//=============================================================================

#define PINCHER 1
#define REACTOR 2
#define WIDOWX 3

//uncomment one of the following lines depending on which arm you want to use
//#define ARMTYPE PINCHER
#define ARMTYPE REACTOR
//#define ARMTYPE WIDOWX

#if !defined(ARMTYPE) 
   #error YOU HAVE TO SELECT THE ARM YOU ARE USING! Uncomment the correct line above for your arm
#endif

#define SOUND_PIN    8      // Tell system we have added speaker to IO pin 1
#define MAX_SERVO_DELTA_PERSEC 512
//#define DEBUG             // Enable Debug mode via serial

//=============================================================================
// Global Include files
//=============================================================================
//DYNAMIXEL Control libraries
//a1-16 library for communicating with servos
#include <A1_16.h>
//BOLIDE_Player interpolates movements
#include <BOLIDE_Player.h>

//input control file - local
#include "Kinematics.h"

//armSequence
#include "armSequence.h"

#include <ax12.h>

 //include the Wire/I2C Library
#include <Wire.h>      
//include the WiiClassy Libary
#include <WiiClassy.h>  
#include <Servo.h>  


#define buzzerPin   8
#define songLength  18
#define tempo       150

char notes[] = "cdfda ag cdfdg gf ";                  // This is the note you want to play
int  beats[] = {1,1,1,1,1,1,4,4,2,1,1,1,1,1,1,4,4,2}; // How long to play the note in array

Servo myservo;  // create servo object to control a servo
WiiClassy classy = WiiClassy(); //start an instance of the WiiClassy Library



//=============================================================================
// Global Objects
//=============================================================================

BOLIDE_Player bioloid ; //start bioloid controller at 1mbps





const int WII_JOYSTICK_MAX = 63; //max value that a joystick fromt he wiimote will send back
//generic deadband limits - not all joystics will center at 32, so these limits remove 'drift' from joysticks that are off-center.
const int DEADBANDLOW = 30;   //decrease this value if drift occurs, increase it to increase sensitivity around the center position
const int DEADBANDHIGH = 34;  //increase this value if drift occurs, decrease it to increase sensitivity around the center position

const int BUZZER_PIN = 8;

int servo5Value = 90; //servo position


Servo servo5;   //servo object

int spdgrip = 5; 


int mod[5];

int mapChange;



//last read values of analog sensors (Native values, 0-WII_JOYSTICK_MAX)
int joyXVal = 0;     //present value of the base rotation knob (analog 0)
int joyYVal = 0; //present value of the shoulder joystick (analog 1)
int joyZVal = 0;    //present value of the elbow joystick (analog 2)
int joyGAVal = 0;    //present value of the wrist joystick (analog 3)
int joyGripperVal = 0;  //present value of the gripper rotation knob (analog 4)

//last calculated values of analog sensors (Mapped values)
float joyXMapped = 0;      //base joystick value, mapped from 1-WII_JOYSTICK_MAX to BASE_MIN-BASE_MAX
float joyYMapped = 0;      //shoulder joystick value, mapped from 1-WII_JOYSTICK_MAX to -spd to spd
float joyZMapped = 0;      //elbow joystick value, mapped from 1-WII_JOYSTICK_MAX to -spd to spd
float joyGAMapped = 0;     //wrist joystick value, mapped from 1-WII_JOYSTICK_MAX to -spd to spd
float joyGripperMapped = 0;   //gripper knob  value, mapped from 1-WII_JOYSTICK_MAX to GRIPPER_MIN-GRIPPER_MAX



float spd = 1.00;  //speed modififer, increase this to increase the speed of the movement

//use these for variable speed only
float spdMod = .1;
float maxSpd = 5;
float minSpd = .2;
unsigned long lastArmSpeedUpdate;
int gripSpd = 10;
int gripSpdMod = 5;
int maxGripSpd = 100;
int minGripSpd = 1;
unsigned long lastGripperSpeedUpdate;
int speedUpdateInterval = 100; //10hz
bool lastLButtonState;
bool lastRButtonState;
int delayTime = 5; //milliseocnds to delay in each processAnalog function - reduce this to get full speed from the arm
//end variable speed


#include <ax12.h>


//===================================================================================================
// Setup 
//====================================================================================================
void setup() {


  
  Serial2.begin(57600);
  ax12Init(57600, &Serial2);

  

     pinMode(buzzerPin, OUTPUT);   //don't forget to put the pin as an ouput


  delay(100);
  classy.init();  //start classy library
  delay(100);
  classy.update();  //read data from the classic controller


  servo5.attach(6); //attach the servo on pin 10

  //initialize the Serial Port
  Serial.begin(115200);  
  
  Serial.println("A1-16 Robot Arm Online.");

  Serial1.begin(115200);//start serial communication on serial1 / A!-16 servo
  A1_16_Ini(115200);//initiate a1-16 library
  bioloid.setup(115200, 6);//intiate bioloid interpolation 

//  // Next initialize the Bioloid
//  bioloid.poseSize = CNT_SERVOS;
//
//  // Read in the current positions...
//  bioloid.readPose();
  delay(100);
  
  // Start off to put arm to sleep...
  //PutArmToSleep();
  
  //startup sound
  MSound(3, 60, 2000, 80, 2250, 100, 2500);
//
//while(1)
//{
//
//   for(int i = 1; i< 7; i++)
//   {
//    
//   Serial.print(i);
//   Serial.print(":");
//   int test = A1_16_ReadData(i,CMD_RAM_READ,RAM_Joint_Position,2);
//   Serial.println(test);
//   delay(10);
//   }
//    delay(1000);
////
////   delay(1000);
////
////
////    int tempi = (-0.0615*900)+1049.2;
////    SetPositionI_JOG(2, 0, tempi - 900);
////    SetPositionI_JOG(3, 0, 900);
////  delay(10000);
////
////
////    
////
////
////    
////  for(int i = 0; i < 1023; i ++)
////{
////    int tempi = (-0.0615*i)+1049.2;
////  
////    SetPositionI_JOG(2, 0, i);
////    SetPositionI_JOG(3, 0, tempi - i);
////    delay(25);
////}
////
////    
////  for(int i = 1023; i > 0; i --)
////{
////  
////    int tempi = (-0.0615*i)+1049.2;
////    SetPositionI_JOG(2, 0,  i);
////    SetPositionI_JOG(3, 0, tempi - i);
////    delay(25);
////}
//
//    
//}


//while(1)
//{
//  bioloid.poseSize = 1;//2 servos, so the pose size will be 2
//  bioloid.readPose();//find where the servos are currently
//  bioloid.setNextPose(1,512);//prepare the PAN servo to the centered position, pan/2048
//
//  bioloid.interpolateSetup(5000);//setup for interpolation from the current position to the positions set in setNextPose, over 2000ms
//  while(bioloid.interpolating > 0)  //until we have reached the positions set in setNextPose, execute the instructions in this loop
//  {
//    bioloid.interpolateStep();//move servos 1 'step
//    delay(3);
//  }
//  
//
//  bioloid.poseSize = 1;//2 servos, so the pose size will be 2
//  bioloid.readPose();//find where the servos are currently
//  bioloid.setNextPose(1,300);//prepare the PAN servo to the centered position, pan/2048
//
//  bioloid.interpolateSetup(5000);//setup for interpolation from the current position to the positions set in setNextPose, over 2000ms
//  while(bioloid.interpolating > 0)  //until we have reached the positions set in setNextPose, execute the instructions in this loop
//  {
//    bioloid.interpolateStep();//move servos 1 'step
//    delay(3);
//  }
//
//  bioloid.poseSize = 1;//2 servos, so the pose size will be 2
//  bioloid.readPose();//find where the servos are currently
//  bioloid.setNextPose(1,800);//prepare the PAN servo to the centered position, pan/2048
//
//  bioloid.interpolateSetup(5000);//setup for interpolation from the current position to the positions set in setNextPose, over 2000ms
//  while(bioloid.interpolating > 0)  //until we have reached the positions set in setNextPose, execute the instructions in this loop
//  {
//    bioloid.interpolateStep();//move servos 1 'step
//    delay(3);
//  }
//  
//
//}





  bioloid.poseSize = 6;//2 servos, so the pose size will be 2
  bioloid.readPose();//find where the servos are currently
  bioloid.setNextPose(1,512);//prepare the PAN servo to the centered position, pan/2048
  bioloid.setNextPose(2,512);//preprare the tilt servo to the centered position, tilt/2048
  bioloid.setNextPose(3,512);//prepare the PAN servo to the centered position, pan/2048
  bioloid.setNextPose(4,512);//preprare the tilt servo to the centered position, tilt/2048
  bioloid.setNextPose(5,512);//prepare the PAN servo to the centered position, pan/2048
  bioloid.setNextPose(6,512);//preprare the tilt servo to the centered position, tilt/2048
  bioloid.interpolateSetup(2500);//setup for interpolation from the current position to the positions set in setNextPose, over 2000ms
  while(bioloid.interpolating > 0)  //until we have reached the positions set in setNextPose, execute the instructions in this loop
  {
    bioloid.interpolateStep();//move servos 1 'step
    delay(3);
  }



  
// MoveArmTo(512, 473, 281, 743, 512, 512, 200, true) ;

MoveArmToHome();
delay(5000);
  
}//end setup


//===================================================================================================
// loop: Our main Loop!
//===================================================================================================
void loop() 
{
  

   updateControls();//update the controls from the wii controller


   doArmIK(false , g_sBase,  g_sIKY,  g_sIKZ,  g_sIKGA);  //update serovs based on y/z/angle ik value
   MoveArmTo(sBase, sShoulder, sElbow, sWrist, sWristRot, sGrip, 10, true);  //move arm to nw positions

 // playSequence();
  

  
  // if (bioloid.interpolating > 0) 
  // {
  //   bioloid.interpolateStep();
  // }
} //end Main







//===================================================================================================
// functions
//===================================================================================================



//tie together all the wii classic work
void updateControls()
{
    static bool lastYPressed = false;//check the last button pressed, this is needed for togglien buttons

    
    classy.update();  //read data from the classic controller
  
    readClassicJoysticks(); //read joystick values


    if(classy.yPressed )
    {
      //code here to do something when pressed
      lastYPressed = true;
    }
    else
    {
      lastYPressed = false; //resets button state when you let it go
    }

  
    if(classy.homePressed == true)
    {
      //do something, don't need the state button unless it is a very short action (short actions may repeat)
    }
    
    if(classy.selectPressed == true)
    {


      //do something, don't need the state button unless it is a very short action (short actions may repeat)
    }
    
    if(classy.startPressed == true)
    {
      
  
      //do something, don't need the state button unless it is a very short action (short actions may repeat)
    }
    
  
  if (classy.aPressed) {
    rickRoll();
    }
  if (classy.bPressed) {
    leftStack();
    }



     //only update the base joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
     if(joyXVal > DEADBANDHIGH || joyXVal < DEADBANDLOW)
     {
       joyXMapped = mapfloat(joyXVal, WII_JOYSTICK_MAX, 0,  -spd * 5 , spd * 5); //Map analog value from native joystick value (0 to WII_JOYSTICK_MAX) to incremental change (-spd to spd)
       g_sBase = g_sBase + joyXMapped;

     }
  
     //only update the shoulder joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
     if(joyYVal > DEADBANDHIGH || joyYVal < DEADBANDLOW)
     {
       joyYMapped = mapfloat(joyYVal, WII_JOYSTICK_MAX, 0, -2*spd, 2*spd); //Map analog value from native joystick value (0 to WII_JOYSTICK_MAX) to incremental change (-spd to spd)
       g_sIKY = g_sIKY + joyYMapped;
     }
  
     //only update the elbow joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
     if(joyZVal > DEADBANDHIGH || joyZVal < DEADBANDLOW)
     {
      
  
       joyZMapped = mapfloat(joyZVal, WII_JOYSTICK_MAX, 0, spd*2, -spd*2); //Map analog value from native joystick value (0 to WII_JOYSTICK_MAX) to incremental change (-spd to spd)
       g_sIKZ = g_sIKZ + joyZMapped;
     }
     
     //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
     if(joyGAVal > DEADBANDHIGH || joyGAVal < DEADBANDLOW)
     {

        Serial.println(joyGAVal);
       joyGAMapped = mapfloat(joyGAVal, WII_JOYSTICK_MAX, 0, -spd*1.1 , spd*1.1 ); //Map analog value from native joystick value (0 to WII_JOYSTICK_MAX) to incremental change (-spd to spd)
       g_sIKGA = g_sIKGA + joyGAMapped;
     }

     

  
   if(classy.lzPressed)
   {
    
     // joint[GRIPPER] = 0;
   servo5Value = servo5Value + spdgrip;
     //sGrip = 176;
   }

   
   if(classy.rzPressed)
   {
     // joint[GRIPPER] = 40;
 
  servo5Value = servo5Value - spdgrip;
     //sGrip = 176;

   }
    

  
   if(classy.leftShoulderPressed)
   {
    
     // joint[WRIST_ROTATE] = 802;

    sWristRot = 826;
   }

   
   if(classy.rightShoulderPressed)
   {
     // joint[WRIST_ROTATE] = 222;

   

     sWristRot = 218;



   }


servo5Value = constrain(servo5Value, 0, 110);  //constrain the servo value to keep in between 0 and 110 for the gripper

  servo5.write(servo5Value);


    int interpolate = 10;




}




//read the wii classic controller joysticks
void readClassicJoysticks()
{  
  
   //read analog values from analog sensors
   joyXVal = classy.leftStickX;                       //read standard left stick data (0-63)
   joyYVal = classy.leftStickY;                       //read standard left stick  data (0-63)
   joyZVal = 2 * (classy.rightStickY + 1);                  //read standard right stick data (0-31),add one then,  then double it to get to 6-bit range (0-64), 
   
   joyGAVal = 2 * (classy.rightStickX + 1);                 //read standard right stick data (0-31), add one then double it to get to 6-bit range (0-64)
   joyGripperVal = 2 * (classy.rightShoulderPressure + 1);  //read standard shoulder data (0-31),add one then double it to get to 6-bit range (0-64)
   delay(delayTime);  //slow down the readings - remove
}


//float version of map() to convert data from one range to another
  float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
  {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }




void IKSequencingControl(float X, float Y, float Z, float GA, float WR, int grip, int interpolate, int pause, int enable)
{
  //only run the sequence code if the enable is set
  if(enable == 1)
  {
    //if the arm is in Cartesian mode, do the arm IK in cartesian mode. This will set the arm parameters
    if(g_bIKMode == IKM_IK3D_CARTESIAN || g_bIKMode == IKM_IK3D_CARTESIAN_90)
    {
      doArmIK(true, X, Y, Z, GA); 
      
    }
    
    //if the arm is in cylindrical mode, do the arm IK in cylindrical mode. This will set the arm parameters
    else if(g_bIKMode == IKM_CYLINDRICAL || g_bIKMode ==IKM_CYLINDRICAL_90)
    {  
    //  sBase = X;
      doArmIK(false, X, Y, Z, GA); 
      
    }
    
    //otherweise the arm is in backhoe mode, so set the arm parameters directly
    else if(g_bIKMode == IKM_BACKHOE)
    {
      sBase = X;
      sShoulder = Y;
      sElbow = Z;
      sWrist = GA;
      
    }
    
    
    //set the wrist rotate and gripper parameters (as these are not part of the IK)
    sWristRot = WR;
    sGrip = grip;
    
    //move arm to position
    MoveArmToPose(sBase, sShoulder, sElbow, sWrist, sWristRot, sGrip, interpolate, true);  
    
    //wait for pause milliseconds
    delay(pause);
  }
}










// BUGBUG:: Move to some library...
//==============================================================================
//    SoundNoTimer - Quick and dirty tone function to try to output a frequency
//            to a speaker for some simple sounds.
//==============================================================================
#ifdef SOUND_PIN
void SoundNoTimer(unsigned long duration,  unsigned int frequency)
{
#ifdef __AVR__
  volatile uint8_t *pin_port;
  volatile uint8_t pin_mask;
#else
  volatile uint32_t *pin_port;
  volatile uint16_t pin_mask;
#endif
  long toggle_count = 0;
  long lusDelayPerHalfCycle;

  // Set the pinMode as OUTPUT
  pinMode(SOUND_PIN, OUTPUT);

  pin_port = portOutputRegister(digitalPinToPort(SOUND_PIN));
  pin_mask = digitalPinToBitMask(SOUND_PIN);

  toggle_count = 2 * frequency * duration / 1000;
  lusDelayPerHalfCycle = 1000000L/(frequency * 2);

  // if we are using an 8 bit timer, scan through prescalars to find the best fit
  while (toggle_count--) {
    // toggle the pin
    *pin_port ^= pin_mask;

    // delay a half cycle
    delayMicroseconds(lusDelayPerHalfCycle);
  }    
  *pin_port &= ~(pin_mask);  // keep pin low after stop

}

void MSound(byte cNotes, ...)
{
  va_list ap;
  unsigned int uDur;
  unsigned int uFreq;
  va_start(ap, cNotes);

  while (cNotes > 0) {
    uDur = va_arg(ap, unsigned int);
    uFreq = va_arg(ap, unsigned int);
    SoundNoTimer(uDur, uFreq);
    cNotes--;
  }
  va_end(ap);
}
#else
void MSound(byte cNotes, ...)
{
};
#endif



void rickRoll(){                    // this function is what actually plays the song
  int i, duration;
  for (i = 0; i < songLength; i++){ // step through the song arrays
    duration = beats[i] * tempo;    // length of note/rest in ms
    if (notes[i] == ' '){
      delay(duration);              // then pause for a moment
    }else{
      tone(buzzerPin, frequency(notes[i]), duration);
      delay(duration);              // wait for tone to finish
    }
    delay(tempo/10);                // brief pause between notes
  }
}
//-------------frequency-----------
int frequency(char note){           // This function is called by rickRoll()
  int r;
  const int numNotes = 8;           // number of notes we're storing
  char names[] = { 'c', 'd', 'e', 'f', 'g', 'a', 'b', 'C' };    // letter notes
  int frequencies[] = {262, 294, 330, 349, 392, 440, 494, 523}; // frequencies
  for (r = 0; r < numNotes; r++){
    if (names[r] == note){
      return(frequencies[r]);       // Yes! Return the frequency
    }
  }
  return(0);
}
//============================= you need these functions =======================================

