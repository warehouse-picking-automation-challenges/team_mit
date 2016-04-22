#include <ros/ros.h>
#include <matVec/matVec.h>
#include <ncurses.h>
#include <string.h>
#include <cstdio>

#include <robot_comm/robot_comm.h>

#include <string>
using namespace std;

#include "teleop_console.hpp"

//#define DEG2RAD 3.1415 / 180.0

int linStep;
int angStep;

// Zone (max accuracy)
#define ROBOT_ZONE 1

// Robot speed
#define ROBOT_SPEED_TCP 30
#define ROBOT_SPEED_ORI 10

// Steps (mm, deg)
#define LIN_STEP_3            100 
#define ANG_STEP_3            30 
#define LIN_STEP_2            10
#define ANG_STEP_2            3 
#define LIN_STEP_1            1 
#define ANG_STEP_1            1 

// Rate to run the teleoperator interface (Hz)
#define TELEOP_RATE 25.0

bool initalize();
void updateCartesian(const robot_comm::robot_CartesianLogConstPtr& msg);
void updateJoints(const robot_comm::robot_JointsLogConstPtr& msg);
void updateOther();
void updateState(); //Makes the updated parameters the current ones.
void executeUserCommand();

RobotComm robot;
ros::NodeHandle *nodePtr;
ros::Subscriber robot_cartesian_sub;
ros::Subscriber robot_joints_sub;


double curCartesian[7];
double curJoints[6];
double curWorkObj[7];
double curTool[7];
double curInertia[7];
double curSpeed[2];

double newCartesian[7];
double newJoints[6];
double newWorkObj[7];
double newTool[7];
double newInertia[7];
double newSpeed[2];

//Booleans to signal updated values 
bool updatedCartesian, updatedJoints, updatedOther;

//Booleans to signal that the global values are being read.
bool readingCartesian, readingJoints, readingOther;

// Control of the teleoperator
bool exitProgram;
bool enabled;
KEY_CMDS last_key;

//Set of 6 predefined orientations
#define NUM_ALIGN_DIRS 6
const Vec align_vecs[NUM_ALIGN_DIRS] = 
{
  Vec("1    0   0", 3),
  Vec("0    1   0", 3),
  Vec("0    0   1", 3),
  Vec("-1   0   0", 3),
  Vec("0    -1  0", 3),
  Vec("0    0   -1",3)
};

const Quaternion align_quats[NUM_ALIGN_DIRS] =
{
  Quaternion("0.7071  0       0.7071  0"),
  Quaternion("0.7071 -0.7071  0       0"),
  Quaternion("1       0       0       0"),
  Quaternion("0.7071  0       -0.7071 0"),
  Quaternion("0.7071  0.7071  0       0"),
  Quaternion("-1      0       0       0")
};
