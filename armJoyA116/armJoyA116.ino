
#include <A1_16.h>

#include <EEPROM.h>

int base = 512;
int shoulder = 512;
int elbow = 512;
int wrist = 512;
int wristRot = 512;
int gripper = 512;

int servoId[5] = {1,2,4,5,6};
int joint[5] = {512, 512, 512, 512, 512};
int mod[5];

const int NEWID = 6;
void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  Serial.println("start");
  Serial1.begin(115200);
  A1_16_Ini(115200);
  SetPositionI_JOG(1, 1000, 512);
  SetPositionI_JOG(2, 1000, 512);
  SetPositionI_JOG(3, 1000, 512);
  SetPositionI_JOG(4, 1000, 512);
  SetPositionI_JOG(5, 1000, 512);
  SetPositionI_JOG(6, 1000, 512);
  

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
  
}




int mapChange;
void loop() {

  
  
  for(int i = 0; i < 5; i++)
  {
    int joyVal =   analogRead(i); //read joystick value
  //check if value is outiside deadband
  if(joyVal > 612 || joyVal < 412)
  {
    mod[i] = map(joyVal, 0, 1023, -10, 10);
    joint[i] = joint[i] + mod[i];
    
  }
    
  }


  delay(20);

    joint[1] = min(joint[1], 790);
    joint[1] = max(joint[1],243);

    joint[2] = min(joint[2], 790);
    joint[2] = max(joint[2],243);

    
    joint[3] = min(joint[3], 790);
    joint[3] = max(joint[3],243);

    
   // joint[4] = min(joint[4], 790);
    //joint[4] = max(joint[4],243);

    

    SetPositionI_JOG(1, 0, joint[0]);
    
    
    SetPositionI_JOG(2, 0, joint[1]);
    SetPositionI_JOG(3, 0, 1023-joint[1]);

    
    SetPositionI_JOG(4, 0, joint[2]);
    SetPositionI_JOG(5, 0, joint[3]);
    SetPositionI_JOG(6, 0, joint[4]);

}
