
#include <A1_16.h>
#include <BOLIDE_Player.h>

#include <EEPROM.h>


#include <Servo.h>
BOLIDE_Player bioloid ; //start bioloid controller at 1mbps



Servo myservo;  // create servo object to control a servo

int potpin = 5;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin





int base = 512;
int shoulder = 512;
int elbow = 512;
int wrist = 512;
int wristRot = 512;
int gripper = 512;

int servoId[5] = {1,2,4,5,6};
int joint[5] = {512, 512, 512, 512, 450};
int mod[5];

const int BUZZER_PIN = 4;
const int NEWID = 6;
void setup() {
  // put your setup code here, to run once:
  myservo.attach(6);  // attaches the servo on pin 9 to the servo object

  Serial.begin(115200);
  Serial.println("start");
  Serial1.begin(115200);
  A1_16_Ini(115200);

  bioloid.setup(115200, 6);


  pinMode(BUZZER_PIN, OUTPUT);



  tone(BUZZER_PIN, 1000, 100);
  delay(100);
  noTone(BUZZER_PIN);


  bioloid.poseSize = 6;//2 servos, so the pose size will be 2
  bioloid.readPose();//find where the servos are currently
  bioloid.setNextPose(1,512);//prepare the PAN servo to the centered position, pan/2048
  bioloid.setNextPose(2,512);//preprare the tilt servo to the centered position, tilt/2048
  bioloid.setNextPose(3,512);//prepare the PAN servo to the centered position, pan/2048
  bioloid.setNextPose(4,512);//preprare the tilt servo to the centered position, tilt/2048
  bioloid.setNextPose(5,312);//prepare the PAN servo to the centered position, pan/2048
  bioloid.setNextPose(6,512);//preprare the tilt servo to the centered position, tilt/2048
  bioloid.interpolateSetup(5000);//setup for interpolation from the current position to the positions set in setNextPose, over 2000ms
  while(bioloid.interpolating > 0)  //until we have reached the positions set in setNextPose, execute the instructions in this loop
  {
    bioloid.interpolateStep();//move servos 1 'step
    delay(3);
  }


  tone(BUZZER_PIN, 1000, 100);
  delay(100);
  noTone(BUZZER_PIN);
  delay(100);
  tone(BUZZER_PIN, 1200, 100);
  delay(100);
  noTone(BUZZER_PIN);
  delay(100);
  tone(BUZZER_PIN, 1400, 100);
  delay(100);
  noTone(BUZZER_PIN);
  delay(100);

//   SetPositionI_JOG(1, 1000, 512);
//   SetPositionI_JOG(2, 1000, 512);
//   SetPositionI_JOG(3, 1000, 512);
//   SetPositionI_JOG(4, 1000, 512);
//   SetPositionI_JOG(5, 1000, 512);
//   SetPositionI_JOG(6, 1000, 512);
  

//  // int test = A1_16_ReadData(1,CMD_RAM_READ,18,2);

//   A1_16_WriteData(2,CMD_RAM_WRITE,RAM_LED_Control,1);
//   delay(500);
  
//   A1_16_WriteData(2,CMD_RAM_WRITE,RAM_LED_Control,1<<1);
//   delay(500);
  
//   A1_16_WriteData(2,CMD_RAM_WRITE,RAM_LED_Control,1<<2);
  
//   delay(500);
  
//   A1_16_WriteData(2,CMD_RAM_WRITE,RAM_LED_Control,1<<3);
  
//   delay(500);
  
//   A1_16_WriteData(2,CMD_RAM_WRITE,RAM_LED_Control,0);
  
//   delay(500);




  
//   A1_16_WriteData2(1,CMD_RAM_WRITE,68,0);
  
//   delay(500);
  
//   A1_16_WriteData(2,CMD_RAM_WRITE,RAM_LED_Control,0);

// //  A1_16_TorqueOff(2);
//   //A1_16_TorqueOff(3);

  
//   int test = ReadPosition(1);
//   Serial.println(test);

//   A1_16_WriteData2(1,CMD_RAM_WRITE,RAM_Overload_PWM,1024);
//   delay(10);
//   A1_16_WriteData2(2,CMD_RAM_WRITE,RAM_Overload_PWM,255);
//   delay(10);
//   A1_16_WriteData2(3,CMD_RAM_WRITE,RAM_Overload_PWM,255);
//   delay(10);
  
//   for(int i = 1; i< 7; i++)
//   {
    
//   Serial.print(i);
//   Serial.print(":");
//   test = A1_16_ReadData(i,CMD_RAM_READ,RAM_Overload_PWM,2);
//   Serial.println(test);
//   delay(500);
//   }

  
//   Serial.print(1);
//   Serial.print(":");
//   test = A1_16_ReadData(1,CMD_RAM_READ,RAM_Overload_PWM,2);
//   Serial.println(test);
//   delay(500);
  
}




int mapChange;
void loop() {

  



}
