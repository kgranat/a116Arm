#include "Kinematics.h"
#include "GlobalArm.h"
extern void IKSequencingControl(float X, float Y, float Z, float GA, float WR, int grip, int interpolate, int pause, int enable);
// We need to declare the data exchange
// variable to be volatile - the value is
// read from memory.
volatile int playState = 0; // 0 = stopped 1 = playing

void playSequence()
{
  delay(500);
  Serial.println("Sequencing Mode Active."); 

  playState = 1;  //set playState to 1 as the sequence is now playing


    g_bIKMode = IKM_CYLINDRICAL;
  delay(500);
  Serial.println("IKSequencingControl"); 

    IKSequencingControl(512 , 150 , 250 , 0 , 512 , 0 , 2000 , 2000, playState);
    IKSequencingControl(512 , 200 , 250 , 0 , 512 , 0 , 2000 , 2000, playState);
  Serial.println("IKSequencingControl end"); 

  delay(1000);

    //IKSequencingControl(512 , 150 , 150 , 0 , 512 , 0 , 2000 , 1000, playState);



 delay(100);
 Serial.println("Sequence Complete."); 
 delay(500);
 
 //uncomment this to  put the arm in sleep position after a sequence
 //PutArmToSleep();
    
}



void leftStack()
{
  
    g_bIKMode = IKM_CYLINDRICAL;
  playState = 1;

  Serial.println("start IK sequence");
    
    IKSequencingControl(527 , 430 , 237 , 0 , 218 , 256 , 5000 , 0, playState);
    
  Serial.println("end IK sequence");
}


void rightStack()
{
  
    g_bIKMode = IKM_CYLINDRICAL;
  playState = 1;


  Serial.println("start IK sequence");
    
    IKSequencingControl(352 , 430 , 227 , 0 , 218 , 256 , 5000 , 0, playState);
    
  Serial.println("end IK sequence");
}

void homePos()
{
  
    g_bIKMode = IKM_CYLINDRICAL;
  playState = 1;
//    A1_16_SetSpeed(1,1,20);
//    A1_16_SetSpeed(2,1,20);
//    A1_16_SetSpeed(3,1,20);
//    A1_16_SetSpeed(4,1,20);
//    A1_16_SetSpeed(5,1,20);
//    A1_16_SetSpeed(6,1,20);

  Serial.println("start IK sequence");
    
    IKSequencingControl(507 , 415 , 463 , 0 , 512 , 256 , 500 , 0, playState);
    
  Serial.println("end IK sequence");
}

void topStack()
{
  
    g_bIKMode = IKM_CYLINDRICAL;
  playState = 1;


  Serial.println("start IK sequence");
    
    IKSequencingControl(448 , 548 , 142 , 0 , 218 , 256 , 5000 , 0, playState);
    
  Serial.println("end IK sequence");
}


void bottomStack()
{
  
    g_bIKMode = IKM_CYLINDRICAL_90;
    MoveArmTo90Home();
  playState = 1;

  Serial.println("start IK sequence");
    
    IKSequencingControl(519 , 99 , 59 , -71 , 512 , 256 , 1000 , 0, playState);
    
  Serial.println("end IK sequence");
}

void sideStack()
{
  
    g_bIKMode = IKM_CYLINDRICAL;
  playState = 1;

  Serial.println("start IK sequence");
    
    IKSequencingControl(787 , 418 , 235 , 0 , 218 , 256 , 5000 , 0, playState);
    
  Serial.println("end IK sequence");
}


