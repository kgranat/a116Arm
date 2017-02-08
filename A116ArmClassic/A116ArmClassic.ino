//a1-16 library for communicating with servos
#include <A1_16.h>
//BOLIDE_Player interpolates movements
#include <BOLIDE_Player.h>

 //include the Wire/I2C Library
#include <Wire.h>      
//include the WiiClassy Libary
#include <WiiClassy.h>  


#include <Servo.h>




BOLIDE_Player bioloid ; //start bioloid controller at 1mbps

Servo myservo;  // create servo object to control a servo
WiiClassy classy = WiiClassy(); //start an instance of the WiiClassy Library



const int WII_JOYSTICK_MAX = 63; //max value that a joystick fromt he wiimote will send back
//generic deadband limits - not all joystics will center at 32, so these limits remove 'drift' from joysticks that are off-center.
const int DEADBANDLOW = 30;   //decrease this value if drift occurs, increase it to increase sensitivity around the center position
const int DEADBANDHIGH = 34;  //increase this value if drift occurs, decrease it to increase sensitivity around the center position

const int BUZZER_PIN = 4;


//array references - NOT SERVO IDS 
const int BASE = 0;
const int SHOULDER = 1;
const int ELBOW = 2;
const int WRIST_ANGLE = 3;
const int WRIST_ROTATE = 4;
const int GRIPPER = 5;


//servo IDs for each joint, skips any double joints and handles them later
int servoId[5] = {1,2,4,5,6};
//holds positions for each joint -base, shoulder, elbow, wrist, wrist rotate. Also holds the gripper value at the end, which is a non a1-16 servo
int joint[6] = {512, 512, 512, 512, 512, 40};

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

void setup() 
{


  delay(100);
  classy.init();  //start classy library
  delay(100);
  classy.update();  //read data from the classic controller

  
  myservo.attach(6);  // attaches the servo on pin 9 to the servo object
  
  Serial.begin(115200); //start serial communication on serial0 / usb
  Serial.println("start");
  
  Serial1.begin(115200);//start serial communication on serial1 / A!-16 servo
  A1_16_Ini(115200);//initiate a1-16 library

  bioloid.setup(115200, 6);//intiate bioloid interpolation 


  pinMode(BUZZER_PIN, OUTPUT);  //set buzzer to output



  tone(BUZZER_PIN, 1000, 100);
  delay(100);
  noTone(BUZZER_PIN);


  bioloid.poseSize = 6;//2 servos, so the pose size will be 2
  bioloid.readPose();//find where the servos are currently
  bioloid.setNextPose(1,512);//prepare the rotate servo centered / 512
  bioloid.setNextPose(2,512);//preprare the shoulder joint centered / 512
  bioloid.setNextPose(3,512);//prepare the shoulder servo centered / 512
  bioloid.setNextPose(4,750);//preprare the elbow servo centered / 512
  bioloid.setNextPose(5,512);//prepare the wrist servo centered / 512
  bioloid.setNextPose(6,512);//preprare the wrist rotate servo centered / 512
  bioloid.interpolateSetup(5000);//setup for interpolation from the current position to the positions set in setNextPose, over 5000
  
  while(bioloid.interpolating > 0)  //until we have reached the positions set in setNextPose, execute the instructions in this loop
  {
    bioloid.interpolateStep();//move servos 1 'step
    delay(3);
  }


  /*commented out for later

 // int test = A1_16_ReadData(1,CMD_RAM_READ,18,2);

  A1_16_WriteData(2,CMD_RAM_WRITE,RAM_LED_Control,1);
  delay(500);
  
  A1_16_WriteData(2,CMD_RAM_WRITE,RAM_LED_Control,1<<1);
  delay(500);
  
  A1_16_WriteData(2,CMD_RAM_WRITE,RAM_LED_Control,1<<2);
  
  delay(500);
  
  A1_16_WriteData(2,CMD_RAM_WRITE,RAM_LED_Control,1<<3);
  
  delay(500);
  
  A1_16_WriteData(2,CMD_RAM_WRITE,RAM_LED_Control,0);
  
  delay(500);




  
  A1_16_WriteData2(1,CMD_RAM_WRITE,68,0);
  
  delay(500);
  
  A1_16_WriteData(2,CMD_RAM_WRITE,RAM_LED_Control,0);

//  A1_16_TorqueOff(2);
  //A1_16_TorqueOff(3);

  
  int test = ReadPosition(1);
  Serial.println(test);

  A1_16_WriteData2(1,CMD_RAM_WRITE,RAM_Overload_PWM,1024);
  delay(10);
  A1_16_WriteData2(2,CMD_RAM_WRITE,RAM_Overload_PWM,255);
  delay(10);
  A1_16_WriteData2(3,CMD_RAM_WRITE,RAM_Overload_PWM,255);
  delay(10);
  
  for(int i = 1; i< 7; i++)
  {
    
  Serial.print(i);
  Serial.print(":");
  test = A1_16_ReadData(i,CMD_RAM_READ,RAM_Overload_PWM,2);
  Serial.println(test);
  delay(500);
  }

  
  Serial.print(1);
  Serial.print(":");
  test = A1_16_ReadData(1,CMD_RAM_READ,RAM_Overload_PWM,2);
  Serial.println(test);
  delay(500);
  */
}




void loop() {

  
  updateControls();//update the controls from the wii controller

  moveServos(); //move servos based on control changes

  delay(20);  //short delay to move servos, maybe adjust this

   

}


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
    
  
     //only update the base joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
     if(joyXVal > DEADBANDHIGH || joyXVal < DEADBANDLOW)
     {
       joyXMapped = mapfloat(joyXVal, WII_JOYSTICK_MAX, 0,  -spd * 10 , spd * 10); //Map analog value from native joystick value (0 to WII_JOYSTICK_MAX) to incremental change (-spd to spd)
       joint[BASE] = joint[BASE] + joyXMapped;

     }
  
     //only update the shoulder joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
     if(joyYVal > DEADBANDHIGH || joyYVal < DEADBANDLOW)
     {
       joyYMapped = mapfloat(joyYVal, WII_JOYSTICK_MAX, 0, -spd, spd); //Map analog value from native joystick value (0 to WII_JOYSTICK_MAX) to incremental change (-spd to spd)
       joint[SHOULDER] = joint[SHOULDER] + joyYMapped;
     }
  
     //only update the elbow joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
     if(joyZVal > DEADBANDHIGH || joyZVal < DEADBANDLOW)
     {
      
  
       joyZMapped = mapfloat(joyZVal, WII_JOYSTICK_MAX, 0, -spd, spd); //Map analog value from native joystick value (0 to WII_JOYSTICK_MAX) to incremental change (-spd to spd)
       joint[ELBOW] = joint[ELBOW] + joyZMapped;
     }
     
     //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
     if(joyGAVal > DEADBANDHIGH || joyGAVal < DEADBANDLOW)
     {

        Serial.println(joyGAVal);
       joyGAMapped = mapfloat(joyGAVal, WII_JOYSTICK_MAX, 0, -spd * 5, spd * 5); //Map analog value from native joystick value (0 to WII_JOYSTICK_MAX) to incremental change (-spd to spd)
       joint[WRIST_ANGLE] = joint[WRIST_ANGLE] + joyGAMapped;
     }

     

  
   if(classy.lzPressed)
   {
    
      joint[GRIPPER] = 0;
   }

   
   if(classy.rzPressed)
   {
      joint[GRIPPER] = 40;

   }
    

  
   if(classy.leftShoulderPressed)
   {
    
      joint[WRIST_ROTATE] = 802;
   }

   
   if(classy.rightShoulderPressed)
   {
      joint[WRIST_ROTATE] = 222;

   }








    int interpolate = 10;


    joint[1] = min(joint[1], 790);
    joint[1] = max(joint[1],243);

    joint[2] = min(joint[2], 790);
    joint[2] = max(joint[2],243);

    
    joint[3] = min(joint[3], 790);
    joint[3] = max(joint[3],243);

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


  void moveServos()
  {
    
    SetPositionI_JOG(1, 0, joint[0]);
    
    
    SetPositionI_JOG(2, 0, joint[1]);
    SetPositionI_JOG(3, 0, 1023-joint[1]);

    
    SetPositionI_JOG(4, 0, joint[2]);
    SetPositionI_JOG(5, 0, joint[3]);
    SetPositionI_JOG(6, 0, joint[4]);
  }

//later we'll try synchronus writes
void moveServosSync()
{

   bioloid.poseSize = 6;//2 servos, so the pose size will be 2
  bioloid.readPose();//find where the servos are currently
  bioloid.setNextPose(1,joint[0]);//prepare the rotate servo centered / 512
  bioloid.setNextPose(2,joint[1]);//preprare the shoulder joint centered / 512
  bioloid.setNextPose(3,1023-joint[1]);//prepare the shoulder servo centered / 512
  bioloid.setNextPose(4,joint[2]);//preprare the elbow servo centered / 512
  bioloid.setNextPose(5,joint[3]);//prepare the wrist servo centered / 512
  bioloid.setNextPose(6,joint[4]);//preprare the wrist rotate servo centered / 512

  
  bioloid.writePose();
}

