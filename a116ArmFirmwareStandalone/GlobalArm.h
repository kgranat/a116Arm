#ifndef GLOBALARM_H
#define GLOBALARM_H



//=============================================================================
// Reactor Global Constraints & Work Area Definition
//=============================================================================
#if ARMTYPE == REACTOR

#define ARMID       2
#define CNT_SERVOS  8 //(sizeof(pgm_axdIDs)/sizeof(pgm_axdIDs[0]))
#define OPT_WRISTROT

/* Servo IDs */
enum {
  SID_BASE=1, SID_RSHOULDER, SID_LSHOULDER, SID_RELBOW,SID_WRISTROT, SID_WRIST,  SID_GRIP};

// Normal Work Area


#define IK_MAX_X  300
#define IK_MIN_X  -300

#define IK_MAX_Y  350
#define IK_MIN_Y  50

#define IK_MAX_Z  350
#define IK_MIN_Z  20

#define IK_MAX_GA  90
#define IK_MIN_GA   -90

// 90 mode Work Area

#define IK_MAX_X_90  300
#define IK_MIN_X_90  -300

#define IK_MAX_Y_90  225
#define IK_MIN_Y_90  20

#define IK_MAX_Z_90  225
#define IK_MIN_Z_90  10

#define IK_MAX_GA_90  -45
#define IK_MIN_GA_90  -90

// offsets 
#define GA_OFFSET  90 //subtract this from GA to give us -90 - +90 angle
#define X_OFFSET  512 //offset value for 3D Cart mode on X axis

// Define Ranges for the different servos...

#define BASE_N      512
#define BASE_MIN    0
#define BASE_MAX    1023

#define SHOULDER_N    512 
#define SHOULDER_MIN  205 
#define SHOULDER_MAX  810

#define ELBOW_N      819
#define ELBOW_MIN    210
#define ELBOW_MAX    900

#define WRIST_N      512
#define WRIST_MIN    200
#define WRIST_MAX    830

#define WROT_N       512
#define WROT_MIN     0
#define WROT_MAX     1023

#define GRIP_N       256
#define GRIP_MIN     0
#define GRIP_MAX     512

// Define some lengths and offsets used by the arm
#define BaseHeight          165L   // (L0)
#define ShoulderLength      230L   // (L1)
#define ElbowLength         195L   //(L2)Length of the Arm from Elbow Joint to Wrist Joint
#define WristLength         50L   // (L3)Wrist length including Wrist rotate

#endif



#endif