//unicating with servos
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



// Servo Angles
float ServoS_1_Angle = 90;
float ServoS_2_Angle = 90;

// Define arm Constants
const float a = 23;      // lower joint length (cm)
const float b = 19.5;      // upper joint length (cm)

// Correction factors to align servo values with their respective axis
const float S_1_CorrectionFactor = 1.309;     // Align arm "a" with the horizontal when at 0 degrees
const float S_2_CorrectionFactor = 1.509;     // Align arm "b" with arm "a" when at 0 degrees

// Correction factor to shift origin out to edge of the mount
const float X_CorrectionFactor = 0;       // X direction correction factor (cm)
const float Y_CorrectionFactor = 0;       // Y direction correction factor (cm)

// Angle Variables
float A;            //Angle oppposite side a (between b and c)
float B;            //Angle oppposite side b
float C;            //Angle oppposite side c
float theta;        //Angle formed between line from origin to (x,y) and the horizontal

// Distance variables
float x;            // x position (cm)
float y;            // y position (cm)
float c;            // Hypotenuse legngth in cm
const float pi = M_PI;  //Store pi in a less annoying format







void setup() 
{
  Serial.begin(115200);
  Serial.print ("Start");

  pinMode(8, OUTPUT);


  Serial1.begin(115200);//start serial communication on serial1 / A!-16 servo
  A1_16_Ini(115200);//initiate a1-16 library

  bioloid.setup(115200, 6);//intiate bioloid interpolation 




}

void loop()
{


  // int tempX  = 15 ;

  //  for(int i=0; i< 30; i++)
  //  {
  //   int tempY = i;
  //   FixCoordinates(tempX,tempY );
  //   CalculateServoAngles();
  //   Serial.print("R1:");
  //   Serial.print(ServoS_1_Angle);
  //   Serial.print(" R2:");
  //   Serial.println(ServoS_2_Angle);


  //   Serial.print(" SV1:");
  //   Serial.print(radToA1Servo(ServoS_1_Angle));
  //   Serial.print(" SV2:");
  //   Serial.println(radToA1Servo(ServoS_2_Angle));

  //   digitalWrite(8, HIGH);
  //   MoveArm();
  //   //delay(500);
  //   digitalWrite(8, LOW);

  //     joint[1] = radToA1Servo(ServoS_1_Angle);
  //     joint[2] = radToA1Servo(ServoS_2_Angle);


  //   delay(100);


  // }
  //  for(int i=30; i>0; i--)
  //  {
  //   int tempY = i;
  //   FixCoordinates(tempX,tempY );
  //   CalculateServoAngles();
  //   Serial.print("R1:");
  //   Serial.print(ServoS_1_Angle);
  //   Serial.print(" R2:");
  //   Serial.println(ServoS_2_Angle);


  //   Serial.print(" SV1:");
  //   Serial.print(radToA1Servo(ServoS_1_Angle));
  //   Serial.print(" SV2:");
  //   Serial.println(radToA1Servo(ServoS_2_Angle));

  //   digitalWrite(8, HIGH);
  //   MoveArm();
  //   //delay(500);
  //   digitalWrite(8, LOW);

  //     joint[1] = radToA1Servo(ServoS_1_Angle);
  //     joint[2] = radToA1Servo(ServoS_2_Angle);


  //   delay(100);


  // }


   int tempY  = 15 ;

   for(int i=0; i< 30; i++)
   {
    int tempX = i;
    FixCoordinates(tempX,tempY );
    CalculateServoAngles();
    Serial.print("R1:");
    Serial.print(ServoS_1_Angle);
    Serial.print(" R2:");
    Serial.println(ServoS_2_Angle);


    Serial.print(" SV1:");
    Serial.print(radToA1Servo(ServoS_1_Angle));
    Serial.print(" SV2:");
    Serial.println(radToA1Servo(ServoS_2_Angle));

    digitalWrite(8, HIGH);
    MoveArm();
    //delay(500);
    digitalWrite(8, LOW);

      joint[1] = radToA1Servo(ServoS_1_Angle);
      joint[2] = radToA1Servo(ServoS_2_Angle);


    delay(100);


  }
   for(int i=30; i>0; i--)
   {
    int tempX = i;
    FixCoordinates(tempX,tempY );
    CalculateServoAngles();
    Serial.print("R1:");
    Serial.print(ServoS_1_Angle);
    Serial.print(" R2:");
    Serial.println(ServoS_2_Angle);


    Serial.print(" SV1:");
    Serial.print(radToA1Servo(ServoS_1_Angle));
    Serial.print(" SV2:");
    Serial.println(radToA1Servo(ServoS_2_Angle));

    digitalWrite(8, HIGH);
    MoveArm();
    //delay(500);
    digitalWrite(8, LOW);

      joint[1] = radToA1Servo(ServoS_1_Angle);
      joint[2] = radToA1Servo(ServoS_2_Angle);


    delay(100);


  }



}







//===================================================================================================
// Convert radians to AX (10 bit goal address) servo position offset. 
//===================================================================================================
int radToA1Servo(float rads){
  float val = rads * 177.617;
  return (int) val;
}



// Get x and y measured from the bottom of the base. Function corrects for offset
void FixCoordinates(float x_input, float y_input)
{
 x = x_input + X_CorrectionFactor;
 y = y_input + Y_CorrectionFactor;
}

// Calculate necessary servo angles to move arm to desired points
void CalculateServoAngles()
{
  c = sqrt( sq(x) + sq(y) );                                            // pythagorean theorem
  B = (acos( (sq(b) - sq(a) - sq(c))/(-2*a*c) )) ;            // Law of cosines: Angle opposite upper arm section
  C = (acos( (sq(c) - sq(b) - sq(a))/(-2*a*b) )) ;            // Law of cosines: Angle opposite hypotenuse
  theta = (asin( y / c )) ;                                   // Solve for theta to correct for lower joint's impact on upper joint's angle
  ServoS_1_Angle = B + theta + S_1_CorrectionFactor;                    // Find necessary angle. Add Correction
  ServoS_2_Angle = C + S_2_CorrectionFactor;                            // Find neceesary angle. Add Correction

}

// Update the servos
void MoveArm()
{
  // ServoS_1.write(ServoS_1_Angle);              // Move joint to desired position
  // ServoS_2.write(ServoS_2_Angle);              // Move joint to desired position

    SetPositionI_JOG(1, 0, joint[0]);
    
    
    SetPositionI_JOG(2, 0, joint[1]);
    SetPositionI_JOG(3, 0, 1023-joint[1]);

    
    SetPositionI_JOG(4, 0, joint[2]);
    SetPositionI_JOG(5, 0, joint[3]);
    SetPositionI_JOG(6, 0, joint[4]);



}



