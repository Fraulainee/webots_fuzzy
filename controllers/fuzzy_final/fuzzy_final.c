#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/lidar.h>
#include <stdio.h>
#include <math.h>
#include <webots/supervisor.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>



#define TIME_STEP 64
#define MAX_SPEED 40  // Khepera4 max speed

#define goalX 0 //0.630298
#define goalY 0  //0.644297

// #define startX -1.62617
// #define startY -1.61453

#define GOAL_THRESHOLD 0.1  // Distance threshold to consider goal reached

// Fuzzy Logic Definitions
#define MAX_RULES 288

double getGoalAngle(double robotPositionX, double robotPositionY){ // calculate object angle
  double goalAngle;
  double computedAngle;
  
  if (goalX == robotPositionX){
   computedAngle = 0;
  }
  else{
    double angleRadians = atan((robotPositionY - goalY) / (robotPositionX - goalX));
    computedAngle = angleRadians * (180.0 / M_PI);  
  }

  if (goalX > robotPositionX){
    goalAngle = computedAngle;
  }
  else{
    goalAngle = 180 + computedAngle;
  }

  if (robotPositionX == goalX){
    if (robotPositionY < goalY){
      goalAngle = 270;
    }
    else{
      goalAngle = 90;
    }
  }
  
goalAngle = fmod(goalAngle + 360.0, 360.0);
return goalAngle;
}

double convertAngle(double inputAngle) { // Convert full 360-degree input angle to the desired -180 to 180 range
    double adjustedAngle;
    
    // Shift the reference frame: Make 90 -> 0
    adjustedAngle = inputAngle - 90;

    // Normalize the angle to the range [-180, 180]
    if (adjustedAngle > 180) {
        adjustedAngle -= 360;
    } else if (adjustedAngle < -180) {
        adjustedAngle += 360;
    }
    
    return adjustedAngle;
}


double getDistance(double x1, double y1, double x2, double y2) { // calculate distance to goal
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

double getRobotHeading(double angleDeg, double up_y){ // calculate robot's heading
   if (angleDeg < 0){ // reading from the webots, and converting 0-180 and 180-0
      angleDeg = angleDeg *-1; //since naay negative 180-0
    }
    else{
      angleDeg=angleDeg;
    }
    
    if (up_y > 0 ){ // converting 180-0 range of degree when robot is facing below
     angleDeg = (180-angleDeg)+180;
    }
   return angleDeg;
}


typedef enum { NEAR, MID, FAR } DistanceTerm;
typedef enum { LEFT, STRAIGHT, RIGHT } AngleTerm;
typedef enum { HIGHFORW, MEDFORW, LOWFORW, LOWREV, MEDREV } MotorTerm;
typedef enum { LIDAR_NEAR, LIDAR_FAR } LidarTerm;

typedef struct {
  LidarTerm left, frontLeft, front, frontRight, right;
  DistanceTerm distance;
  AngleTerm angle;
  MotorTerm leftMotor;
  MotorTerm rightMotor;
} Rule;

Rule rules[MAX_RULES];
int rule_count = 0;

Rule makeRule(LidarTerm l, LidarTerm fl, LidarTerm f, LidarTerm fr, LidarTerm r,
              DistanceTerm d, AngleTerm a, MotorTerm lm, MotorTerm rm) {
  Rule rule = {l, fl, f, fr, r, d, a, lm, rm};
  return rule;
}


//For Lidar Input
static double mf_lidar_near(double x) {
  if (x <= 0.0)
    return 1.0;
  if (x >= 0.8)
    return 0.0;
  // linear drop from 1 to 0 as x goes from 0..0.8
  return (0.8 - x) / 0.8;
}


// far: 0 at x<=0.7, linearly up to 1 at x>=2.0
static double mf_lidar_far(double x) {
  if (x <= 0.7)
    return 0.0;
  if (x >= 1.5)
    return 1.0;
  // ramp up from 0..1 for x in [0.7..2.0]
  return (x - 0.7) / (1.5 - 0.7);
}



// For Distance Input
// Far: percentage 0.0 to 0.4
static double distance_far(float x) {
    if (x <= 0.0) return 1.0;
    if (x >= 0.4) return 0.0;
    return (0.4 - x) / 0.4;
}

// Middle: percentage 0.2 to 0.8
static double distance_middle(float x) {
    if (x <= 0.2) return 0.0;
    if (x >= 0.8) return 1.0;
    return (x - 0.2) / 0.6;
}

// Near: percentage 0.6 to 1.0
static double distance_near(float x) {
    if (x <= 0.6) return 0.0;
    if (x >= 1.0) return 1.0;
    return (x - 0.6) / 0.4;
}



// For Angle input
static double angle_right(float x) {
    if (x <= -179.0)
        return 0.0;
    else if (x >= -1.0)
        return 1.0;
    else
        return (x + 179.0) / (178.0); // (x - (-179)) / (178)
}

// Straight MF: full strength only at 0
/*static double angle_straight(float x) {
    return (x == 0.0) ? 1.0 : 0.0;
}*/
static double angle_straight(double x) {
    if (x <= -60.0)
        return 0.0;
    else if (x >= 60.0)
        return 0.0;
    else
        return (60.0 - fabs(x)) / 60.0;  // Peak at 0
}


// Left MF: ramp from 1 to 179 (decreasing)
static double angle_left(float x) {
    if (x <= 0.0)
        return 0.0;
    else if (x >= 179.0)
        return 0.0;
    else
        return (179.0 - x) / 179.0;
}

// Match Membership
double match_lidar(LidarTerm term, double value) {
  return term == LIDAR_NEAR ? mf_lidar_near(value) : mf_lidar_far(value);
}

double match_distance(DistanceTerm term, double value) {
  if (term == NEAR) return distance_near(value);
  if (term == MID) return distance_middle(value);
  return distance_far(value);
}

double match_angle(AngleTerm term, double value) {
  if (term == LEFT) return angle_left(value);
  if (term == RIGHT) return angle_right(value);
  return angle_straight(value);
}


// FOR MOTOR OUTPUT
double motor_value(MotorTerm term) {
  switch (term) {
    case HIGHFORW: return 1.0;
    case MEDFORW: return 0.5;
    case LOWFORW: return 0.25;
    case LOWREV: return -0.25;
    case MEDREV: return -0.5;
    default: return 0.0;
  }
}



static void init_rules() { //(L, FL, F, FR, R, D, A, LM, RM)
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,NEAR,NEAR,NEAR,LEFT, MEDREV, MEDREV); // Rule No. 1
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,NEAR,NEAR,NEAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 2
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,NEAR,NEAR,NEAR,LEFT, LOWFORW, MEDFORW); // Rule No. 3
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,NEAR,NEAR,NEAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 4
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,NEAR,NEAR,NEAR,LEFT, LOWFORW, LOWFORW); // Rule No. 5
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,NEAR,NEAR,NEAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 6
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,NEAR,NEAR,NEAR,LEFT, LOWFORW, MEDFORW); // Rule No. 7
  rules[rule_count++] = makeRule(FAR,FAR,FAR,NEAR,NEAR,NEAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 8
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,FAR,NEAR,NEAR,LEFT, MEDFORW, LOWFORW); // Rule No. 9
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,FAR,NEAR,NEAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 10
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,FAR,NEAR,NEAR,LEFT, LOWFORW, MEDFORW); // Rule No. 11
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,FAR,NEAR,NEAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 12
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,FAR,NEAR,NEAR,LEFT, LOWFORW, LOWFORW); // Rule No. 13
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,FAR,NEAR,NEAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 14
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,FAR,NEAR,NEAR,LEFT, LOWFORW, MEDFORW); // Rule No. 15
  rules[rule_count++] = makeRule(FAR,FAR,FAR,FAR,FAR,NEAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 16
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,NEAR,FAR,NEAR,LEFT, HIGHFORW, LOWFORW); // Rule No. 17
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,NEAR,FAR,NEAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 18
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,NEAR,FAR,NEAR,LEFT, LOWFORW, MEDFORW); // Rule No. 19
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,NEAR,FAR,NEAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 20
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,NEAR,FAR,NEAR,LEFT, LOWFORW, LOWFORW); // Rule No. 21
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,NEAR,FAR,NEAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 22
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,NEAR,FAR,NEAR,LEFT, LOWFORW, MEDFORW); // Rule No. 23
  rules[rule_count++] = makeRule(FAR,FAR,FAR,NEAR,FAR,NEAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 24
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,FAR,FAR,NEAR,LEFT, MEDFORW, LOWFORW); // Rule No. 25
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,FAR,FAR,NEAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 26
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,FAR,FAR,NEAR,LEFT, LOWFORW, MEDFORW); // Rule No. 27
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,FAR,FAR,NEAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 28
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,FAR,FAR,NEAR,LEFT, LOWFORW, LOWFORW); // Rule No. 29
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,FAR,FAR,NEAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 30
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,FAR,FAR,NEAR,LEFT, LOWFORW, MEDFORW); // Rule No. 31
  rules[rule_count++] = makeRule(FAR,FAR,FAR,FAR,FAR,NEAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 32
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,NEAR,NEAR,MID,LEFT, MEDREV, MEDREV); // Rule No. 33
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,NEAR,NEAR,MID,LEFT, LOWFORW, HIGHFORW); // Rule No. 34
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,NEAR,NEAR,MID,LEFT, LOWFORW, MEDFORW); // Rule No. 35
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,NEAR,NEAR,MID,LEFT, LOWFORW, HIGHFORW); // Rule No. 36
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,NEAR,NEAR,MID,LEFT, MEDFORW, MEDFORW); // Rule No. 37
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,NEAR,NEAR,MID,LEFT, LOWFORW, HIGHFORW); // Rule No. 38
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,NEAR,NEAR,MID,LEFT, LOWFORW, MEDFORW); // Rule No. 39
  rules[rule_count++] = makeRule(FAR,FAR,FAR,NEAR,NEAR,MID,LEFT, LOWFORW, HIGHFORW); // Rule No. 40
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,FAR,NEAR,MID,LEFT, MEDFORW, LOWFORW); // Rule No. 41
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,FAR,NEAR,MID,LEFT, LOWFORW, HIGHFORW); // Rule No. 42
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,FAR,NEAR,MID,LEFT, LOWFORW, MEDFORW); // Rule No. 43
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,FAR,NEAR,MID,LEFT, LOWFORW, HIGHFORW); // Rule No. 44
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,FAR,NEAR,MID,LEFT, MEDFORW, MEDFORW); // Rule No. 45
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,FAR,NEAR,MID,LEFT, LOWFORW, HIGHFORW); // Rule No. 46
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,FAR,NEAR,MID,LEFT, LOWFORW, MEDFORW); // Rule No. 47
  rules[rule_count++] = makeRule(FAR,FAR,FAR,FAR,NEAR,MID,LEFT, LOWFORW, HIGHFORW); // Rule No. 48
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,NEAR,FAR,MID,LEFT, HIGHFORW, LOWFORW); // Rule No. 49
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,NEAR,FAR,MID,LEFT, LOWFORW, HIGHFORW); // Rule No. 50
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,NEAR,FAR,MID,LEFT, MEDFORW, HIGHFORW); // Rule No. 51
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,NEAR,FAR,MID,LEFT, LOWFORW, HIGHFORW); // Rule No. 52
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,NEAR,FAR,MID,LEFT, MEDFORW, MEDFORW); // Rule No. 53
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,NEAR,FAR,MID,LEFT, LOWFORW, HIGHFORW); // Rule No. 54
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,NEAR,FAR,MID,LEFT, MEDFORW, HIGHFORW); // Rule No. 55
  rules[rule_count++] = makeRule(FAR,FAR,FAR,NEAR,FAR,MID,LEFT, LOWFORW, HIGHFORW); // Rule No. 56
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,FAR,FAR,MID,LEFT, HIGHFORW, MEDFORW); // Rule No. 57
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,FAR,FAR,MID,LEFT, LOWFORW, HIGHFORW); // Rule No. 58
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,FAR,FAR,MID,LEFT, MEDFORW, HIGHFORW); // Rule No. 59
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,FAR,FAR,MID,LEFT, LOWFORW, HIGHFORW); // Rule No. 60
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,FAR,FAR,MID,LEFT, MEDFORW, MEDFORW); // Rule No. 61
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,FAR,FAR,MID,LEFT, LOWFORW, HIGHFORW); // Rule No. 62
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,FAR,FAR,MID,LEFT, MEDFORW, HIGHFORW); // Rule No. 63
  rules[rule_count++] = makeRule(FAR,FAR,FAR,FAR,FAR,MID,LEFT, LOWFORW, HIGHFORW); // Rule No. 64
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,NEAR,NEAR,FAR,LEFT, MEDREV, MEDREV); // Rule No. 65
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,NEAR,NEAR,FAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 66
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,NEAR,NEAR,FAR,LEFT, MEDFORW, HIGHFORW); // Rule No. 67
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,NEAR,NEAR,FAR,LEFT, MEDFORW, HIGHFORW); // Rule No. 68
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,NEAR,NEAR,FAR,LEFT, HIGHFORW, HIGHFORW); // Rule No. 69
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,NEAR,NEAR,FAR,LEFT, HIGHFORW, HIGHFORW); // Rule No. 70
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,NEAR,NEAR,FAR,LEFT, MEDFORW, HIGHFORW); // Rule No. 71
  rules[rule_count++] = makeRule(FAR,FAR,FAR,NEAR,NEAR,FAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 72  
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,FAR,NEAR,FAR,LEFT, HIGHFORW, MEDFORW); // Rule No. 73
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,FAR,NEAR,FAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 74
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,FAR,NEAR,FAR,LEFT, MEDFORW, HIGHFORW); // Rule No. 75
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,FAR,NEAR,FAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 76
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,FAR,NEAR,FAR,LEFT, HIGHFORW, HIGHFORW); // Rule No. 77
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,FAR,NEAR,FAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 78
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,FAR,NEAR,FAR,LEFT, MEDFORW, HIGHFORW); // Rule No. 79
  rules[rule_count++] = makeRule(FAR,FAR,FAR,FAR,NEAR,FAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 80
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,NEAR,FAR,FAR,LEFT, HIGHFORW, LOWFORW); // Rule No. 81
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,NEAR,FAR,FAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 82
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,NEAR,FAR,FAR,LEFT, MEDFORW, HIGHFORW); // Rule No. 83
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,NEAR,FAR,FAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 84
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,NEAR,FAR,FAR,LEFT, HIGHFORW, HIGHFORW); // Rule No. 85
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,NEAR,FAR,FAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 86
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,NEAR,FAR,FAR,LEFT, MEDFORW, HIGHFORW); // Rule No. 87
  rules[rule_count++] = makeRule(FAR,FAR,FAR,NEAR,FAR,FAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 88
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,FAR,FAR,FAR,LEFT, HIGHFORW, MEDFORW); // Rule No. 89
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,FAR,FAR,FAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 90
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,FAR,FAR,FAR,LEFT, MEDFORW, HIGHFORW); // Rule No. 91
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,FAR,FAR,FAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 92
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,FAR,FAR,FAR,LEFT, HIGHFORW, HIGHFORW); // Rule No. 93
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,FAR,FAR,FAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 94
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,FAR,FAR,FAR,LEFT, MEDFORW, HIGHFORW); // Rule No. 95
  rules[rule_count++] = makeRule(FAR,FAR,FAR,FAR,FAR,FAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 96
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,NEAR,NEAR,NEAR,STRAIGHT, MEDREV, MEDREV); // Rule No. 97
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,NEAR,NEAR,NEAR,STRAIGHT, LOWFORW, HIGHFORW); // Rule No. 98
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,NEAR,NEAR,NEAR,STRAIGHT, MEDFORW, HIGHFORW); // Rule No. 99
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,NEAR,NEAR,NEAR,STRAIGHT, MEDFORW, HIGHFORW); // Rule No. 100
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,NEAR,NEAR,NEAR,STRAIGHT, LOWFORW, LOWFORW); // Rule No. 101
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,NEAR,NEAR,NEAR,STRAIGHT, LOWFORW, LOWFORW); // Rule No. 102
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,NEAR,NEAR,NEAR,STRAIGHT, LOWFORW, LOWFORW); // Rule No. 103
  rules[rule_count++] = makeRule(FAR,FAR,FAR,NEAR,NEAR,NEAR,STRAIGHT, LOWFORW, LOWFORW); // Rule No. 104
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,FAR,NEAR,NEAR,STRAIGHT, HIGHFORW, MEDFORW); // Rule No. 105
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,FAR,NEAR,NEAR,STRAIGHT, HIGHFORW, MEDFORW); // Rule No. 106
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,FAR,NEAR,NEAR,STRAIGHT, MEDFORW, HIGHFORW); // Rule No. 107
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,FAR,NEAR,NEAR,STRAIGHT, MEDFORW, HIGHFORW); // Rule No. 108
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,FAR,FAR,NEAR,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 109
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,FAR,NEAR,NEAR,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 110
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,FAR,NEAR,NEAR,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 111
  rules[rule_count++] = makeRule(FAR,FAR,FAR,FAR,NEAR,NEAR,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 112
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,NEAR,FAR,NEAR,STRAIGHT, HIGHFORW, LOWFORW); // Rule No. 113
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,NEAR,FAR,NEAR,STRAIGHT, LOWFORW, HIGHFORW); // Rule No. 114
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,NEAR,FAR,NEAR,STRAIGHT, MEDFORW, HIGHFORW); // Rule No. 115
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,NEAR,FAR,NEAR,STRAIGHT, MEDFORW, HIGHFORW); // Rule No. 116
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,NEAR,FAR,NEAR,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 117
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,NEAR,FAR,NEAR,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 118
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,NEAR,FAR,NEAR,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 119
  rules[rule_count++] = makeRule(FAR,FAR,FAR,NEAR,FAR,NEAR,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 120
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,FAR,FAR,NEAR,STRAIGHT, HIGHFORW, MEDFORW); // Rule No. 121
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,FAR,FAR,NEAR,STRAIGHT, HIGHFORW, MEDFORW); // Rule No. 122
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,FAR,FAR,NEAR,STRAIGHT, HIGHFORW, MEDFORW); // Rule No. 123
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,FAR,FAR,NEAR,STRAIGHT, HIGHFORW, MEDFORW); // Rule No. 124
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,FAR,FAR,NEAR,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 125
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,FAR,FAR,NEAR,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 126
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,FAR,FAR,NEAR,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 127
  rules[rule_count++] = makeRule(FAR,FAR,FAR,FAR,FAR,NEAR,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 128
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,NEAR,NEAR,MID,STRAIGHT, MEDREV, MEDREV); // Rule No. 129
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,NEAR,NEAR,MID,STRAIGHT, MEDREV, HIGHFORW); // Rule No. 130
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,NEAR,NEAR,MID,STRAIGHT, LOWFORW, MEDFORW); // Rule No. 131
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,NEAR,NEAR,MID,STRAIGHT, LOWFORW, MEDFORW); // Rule No. 132
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,NEAR,NEAR,MID,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 133
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,NEAR,NEAR,MID,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 134
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,NEAR,NEAR,MID,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 135
  rules[rule_count++] = makeRule(FAR,FAR,FAR,NEAR,NEAR,MID,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 136
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,FAR,NEAR,MID,STRAIGHT, HIGHFORW, MEDFORW); // Rule No. 137
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,FAR,NEAR,MID,STRAIGHT, HIGHFORW, MEDFORW); // Rule No. 138
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,FAR,NEAR,MID,STRAIGHT, MEDFORW, HIGHFORW); // Rule No. 139
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,FAR,NEAR,MID,STRAIGHT, MEDFORW, HIGHFORW); // Rule No. 140
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,FAR,NEAR,MID,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 141
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,FAR,NEAR,MID,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 142
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,FAR,NEAR,MID,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 143
  rules[rule_count++] = makeRule(FAR,FAR,FAR,FAR,NEAR,MID,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 144
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,NEAR,FAR,MID,STRAIGHT, HIGHFORW, LOWFORW); // Rule No. 145
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,NEAR,FAR,MID,STRAIGHT, LOWFORW, HIGHFORW); // Rule No. 146
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,NEAR,FAR,MID,STRAIGHT, MEDFORW, HIGHFORW); // Rule No. 147
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,NEAR,FAR,MID,STRAIGHT, MEDFORW, HIGHFORW); // Rule No. 148
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,NEAR,FAR,MID,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 149
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,NEAR,FAR,MID,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 150
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,NEAR,FAR,MID,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 151
  rules[rule_count++] = makeRule(FAR,FAR,FAR,NEAR,FAR,MID,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 152
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,FAR,FAR,MID,STRAIGHT, HIGHFORW, MEDFORW); // Rule No. 153
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,FAR,FAR,MID,STRAIGHT, HIGHFORW, MEDFORW); // Rule No. 154
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,FAR,FAR,MID,STRAIGHT, MEDFORW, HIGHFORW); // Rule No. 155
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,FAR,FAR,MID,STRAIGHT, MEDFORW, HIGHFORW); // Rule No. 156
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,FAR,FAR,MID,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 157
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,FAR,FAR,MID,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 158
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,FAR,FAR,MID,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 159
  rules[rule_count++] = makeRule(FAR,FAR,FAR,FAR,FAR,MID,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 160
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,NEAR,NEAR,FAR,STRAIGHT, MEDREV, MEDREV); // Rule No. 161
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,NEAR,NEAR,FAR,STRAIGHT, LOWFORW, HIGHFORW); // Rule No. 162
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,NEAR,NEAR,FAR,STRAIGHT, LOWFORW, MEDFORW); // Rule No. 163
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,NEAR,NEAR,FAR,STRAIGHT, LOWFORW, HIGHFORW); // Rule No. 164
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,NEAR,NEAR,FAR,STRAIGHT, HIGHFORW, HIGHFORW); // Rule No. 165
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,NEAR,NEAR,FAR,STRAIGHT, HIGHFORW, HIGHFORW); // Rule No. 166
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,NEAR,NEAR,FAR,STRAIGHT, HIGHFORW, HIGHFORW); // Rule No. 167
  rules[rule_count++] = makeRule(FAR,FAR,FAR,NEAR,NEAR,FAR,STRAIGHT, HIGHFORW, HIGHFORW); // Rule No. 168
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,FAR,NEAR,FAR,STRAIGHT, HIGHFORW, MEDFORW); // Rule No. 169
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,FAR,NEAR,FAR,STRAIGHT, HIGHFORW, MEDFORW); // Rule No. 170
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,FAR,NEAR,FAR,STRAIGHT, HIGHFORW, MEDFORW); // Rule No. 171
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,FAR,NEAR,FAR,STRAIGHT, MEDFORW, HIGHFORW); // Rule No. 172
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,FAR,NEAR,FAR,STRAIGHT, HIGHFORW, HIGHFORW); // Rule No. 173
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,FAR,NEAR,FAR,STRAIGHT, HIGHFORW, HIGHFORW); // Rule No. 174
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,FAR,NEAR,FAR,STRAIGHT, HIGHFORW, HIGHFORW); // Rule No. 175
  rules[rule_count++] = makeRule(FAR,FAR,FAR,FAR,NEAR,FAR,STRAIGHT, HIGHFORW, HIGHFORW); // Rule No. 176
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,NEAR,FAR,FAR,STRAIGHT, HIGHFORW, MEDREV); // Rule No. 177
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,NEAR,FAR,FAR,STRAIGHT, HIGHFORW, MEDREV); // Rule No. 178
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,NEAR,FAR,FAR,STRAIGHT, MEDFORW, HIGHFORW); // Rule No. 179
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,NEAR,FAR,FAR,STRAIGHT, HIGHFORW, HIGHFORW); // Rule No. 180
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,NEAR,FAR,FAR,STRAIGHT, HIGHFORW, HIGHFORW); // Rule No. 181
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,NEAR,FAR,FAR,STRAIGHT, HIGHFORW, HIGHFORW); // Rule No. 182
  rules[rule_count++] = makeRule(FAR,FAR,FAR,NEAR,FAR,FAR,STRAIGHT, HIGHFORW, HIGHFORW); // Rule No. 183
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,FAR,FAR,FAR,STRAIGHT, HIGHFORW, MEDFORW); // Rule No. 184
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,FAR,FAR,FAR,STRAIGHT, HIGHFORW, MEDFORW); // Rule No. 185
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,FAR,FAR,FAR,STRAIGHT, MEDFORW, HIGHFORW); // Rule No. 186
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,FAR,FAR,FAR,STRAIGHT, MEDFORW, HIGHFORW); // Rule No. 187
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,FAR,FAR,FAR,STRAIGHT, HIGHFORW, HIGHFORW); // Rule No. 188
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,FAR,FAR,FAR,STRAIGHT, HIGHFORW, HIGHFORW); // Rule No. 189
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,FAR,FAR,FAR,STRAIGHT, HIGHFORW, HIGHFORW); // Rule No. 190
  rules[rule_count++] = makeRule(FAR,FAR,FAR,FAR,FAR,FAR,STRAIGHT, HIGHFORW, HIGHFORW); // Rule No. 191
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,NEAR,NEAR,NEAR,RIGHT, MEDREV, MEDREV); // Rule No. 192
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,NEAR,NEAR,NEAR,RIGHT, MEDREV, HIGHFORW); // Rule No. 193
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,NEAR,NEAR,NEAR,RIGHT, LOWREV, MEDFORW); // Rule No. 194
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,NEAR,NEAR,NEAR,RIGHT, LOWREV, MEDFORW); // Rule No. 195
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,NEAR,NEAR,NEAR,RIGHT, MEDFORW, MEDFORW); // Rule No. 196
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,NEAR,NEAR,NEAR,RIGHT, MEDFORW, MEDFORW); // Rule No. 197
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,NEAR,NEAR,NEAR,RIGHT, MEDFORW, MEDFORW); // Rule No. 198
  rules[rule_count++] = makeRule(FAR,FAR,FAR,NEAR,NEAR,NEAR,RIGHT, MEDFORW, MEDFORW); // Rule No. 199
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,FAR,NEAR,NEAR,RIGHT, MEDFORW, LOWFORW); // Rule No. 200
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,FAR,NEAR,NEAR,RIGHT, MEDFORW, LOWFORW); // Rule No. 201
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,FAR,NEAR,NEAR,RIGHT, MEDFORW, LOWFORW); // Rule No. 202
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,FAR,NEAR,NEAR,RIGHT, MEDFORW, LOWFORW); // Rule No. 203
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,FAR,NEAR,NEAR,RIGHT, MEDFORW, LOWFORW); // Rule No. 204
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,FAR,NEAR,NEAR,RIGHT, MEDFORW, LOWFORW); // Rule No. 205
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,FAR,NEAR,NEAR,RIGHT, MEDFORW, LOWFORW); // Rule No. 206
  rules[rule_count++] = makeRule(FAR,FAR,FAR,FAR,NEAR,NEAR,RIGHT, LOWFORW, LOWFORW); // Rule No. 207
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,NEAR,FAR,NEAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 208
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,NEAR,FAR,NEAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 209
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,NEAR,FAR,NEAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 210
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,NEAR,FAR,NEAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 211
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,NEAR,FAR,NEAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 212
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,NEAR,FAR,NEAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 213
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,NEAR,FAR,NEAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 214
  rules[rule_count++] = makeRule(FAR,FAR,FAR,NEAR,FAR,NEAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 215
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,FAR,FAR,NEAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 216
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,FAR,FAR,NEAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 217
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,FAR,FAR,NEAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 218
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,FAR,FAR,NEAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 219
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,FAR,FAR,NEAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 220
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,FAR,FAR,NEAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 221
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,FAR,FAR,NEAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 222
  rules[rule_count++] = makeRule(FAR,FAR,FAR,FAR,FAR,NEAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 223
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,NEAR,NEAR,MID,RIGHT, MEDREV, MEDREV); // Rule No. 224
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,NEAR,NEAR,MID,RIGHT, LOWFORW, HIGHFORW); // Rule No. 225
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,NEAR,NEAR,MID,RIGHT, LOWFORW, MEDFORW); // Rule No. 226
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,NEAR,NEAR,MID,RIGHT, LOWFORW, MEDFORW); // Rule No. 227
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,NEAR,NEAR,MID,RIGHT, MEDFORW, MEDFORW); // Rule No. 228
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,NEAR,NEAR,MID,RIGHT, MEDFORW, MEDFORW); // Rule No. 229
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,NEAR,NEAR,MID,RIGHT, MEDFORW, MEDFORW); // Rule No. 230
  rules[rule_count++] = makeRule(FAR,FAR,FAR,NEAR,NEAR,MID,RIGHT, MEDFORW, MEDFORW); // Rule No. 231
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,FAR,NEAR,MID,RIGHT, MEDFORW, LOWFORW); // Rule No. 232
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,FAR,NEAR,MID,RIGHT, MEDFORW, LOWFORW); // Rule No. 233
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,FAR,NEAR,MID,RIGHT, MEDFORW, LOWFORW); // Rule No. 234
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,FAR,NEAR,MID,RIGHT, MEDFORW, LOWFORW); // Rule No. 235
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,FAR,NEAR,MID,RIGHT, MEDFORW, LOWFORW); // Rule No. 236
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,FAR,NEAR,MID,RIGHT, MEDFORW, LOWFORW); // Rule No. 237
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,FAR,NEAR,MID,RIGHT, MEDFORW, LOWFORW); // Rule No. 238
  rules[rule_count++] = makeRule(FAR,FAR,FAR,FAR,NEAR,MID,RIGHT, MEDFORW, LOWFORW); // Rule No. 239
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,NEAR,FAR,MID,RIGHT, HIGHFORW, LOWFORW); // Rule No. 240
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,NEAR,FAR,MID,RIGHT, HIGHFORW, LOWFORW); // Rule No. 241
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,NEAR,FAR,MID,RIGHT, HIGHFORW, LOWFORW); // Rule No. 242
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,NEAR,FAR,MID,RIGHT, HIGHFORW, LOWFORW); // Rule No. 243
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,NEAR,FAR,MID,RIGHT, HIGHFORW, LOWFORW); // Rule No. 244
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,NEAR,FAR,MID,RIGHT, HIGHFORW, LOWFORW); // Rule No. 245
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,NEAR,FAR,MID,RIGHT, HIGHFORW, LOWFORW); // Rule No. 246
  rules[rule_count++] = makeRule(FAR,FAR,FAR,NEAR,FAR,MID,RIGHT, HIGHFORW, LOWFORW); // Rule No. 247
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,FAR,FAR,MID,RIGHT, HIGHFORW, LOWFORW); // Rule No. 248
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,FAR,FAR,MID,RIGHT, HIGHFORW, LOWFORW); // Rule No. 249
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,FAR,FAR,MID,RIGHT, HIGHFORW, LOWFORW); // Rule No. 250
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,FAR,FAR,MID,RIGHT, HIGHFORW, LOWFORW); // Rule No. 251
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,FAR,FAR,MID,RIGHT, HIGHFORW, LOWFORW); // Rule No. 252
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,FAR,FAR,MID,RIGHT, HIGHFORW, LOWFORW); // Rule No. 253
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,FAR,FAR,MID,RIGHT, HIGHFORW, LOWFORW); // Rule No. 254
  rules[rule_count++] = makeRule(FAR,FAR,FAR,FAR,FAR,MID,RIGHT, HIGHFORW, LOWFORW); // Rule No. 255
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,NEAR,NEAR,FAR,RIGHT, MEDREV, MEDREV); // Rule No. 256
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,NEAR,NEAR,FAR,RIGHT, LOWFORW, HIGHFORW); // Rule No. 257
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,NEAR,NEAR,FAR,RIGHT, LOWFORW, MEDFORW); // Rule No. 258
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,NEAR,NEAR,FAR,RIGHT, LOWFORW, MEDFORW); // Rule No. 259
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,NEAR,NEAR,FAR,RIGHT, HIGHFORW, HIGHFORW); // Rule No. 260
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,NEAR,NEAR,FAR,RIGHT, HIGHFORW, HIGHFORW); // Rule No. 261
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,NEAR,NEAR,FAR,RIGHT, HIGHFORW, HIGHFORW); // Rule No. 262
  rules[rule_count++] = makeRule(FAR,FAR,FAR,NEAR,NEAR,FAR,RIGHT, HIGHFORW, HIGHFORW); // Rule No. 263
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,FAR,NEAR,FAR,RIGHT, HIGHFORW, MEDFORW); // Rule No. 264
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,FAR,NEAR,FAR,RIGHT, HIGHFORW, MEDFORW); // Rule No. 265
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,FAR,NEAR,FAR,RIGHT, HIGHFORW, MEDFORW); // Rule No. 266
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,FAR,NEAR,FAR,RIGHT, HIGHFORW, MEDFORW); // Rule No. 267
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,FAR,NEAR,FAR,RIGHT, HIGHFORW, MEDFORW); // Rule No. 268
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,FAR,NEAR,FAR,RIGHT, HIGHFORW, MEDFORW); // Rule No. 269
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,FAR,NEAR,FAR,RIGHT, HIGHFORW, MEDFORW); // Rule No. 270
  rules[rule_count++] = makeRule(FAR,FAR,FAR,FAR,NEAR,FAR,RIGHT, HIGHFORW, MEDFORW); // Rule No. 271
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,NEAR,FAR,FAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 272
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,NEAR,FAR,FAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 273
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,NEAR,FAR,FAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 274
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,NEAR,FAR,FAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 275
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,NEAR,FAR,FAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 276
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,NEAR,FAR,FAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 277
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,NEAR,FAR,FAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 278
  rules[rule_count++] = makeRule(FAR,FAR,FAR,NEAR,FAR,FAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 279
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,FAR,FAR,FAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 280
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,FAR,FAR,FAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 281
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,FAR,FAR,FAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 282
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,FAR,FAR,FAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 283
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,FAR,FAR,FAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 284
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,FAR,FAR,FAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 285
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,FAR,FAR,FAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 286
  rules[rule_count++] = makeRule(FAR,FAR,FAR,FAR,FAR,FAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 287
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,NEAR,FAR,FAR,STRAIGHT, MEDFORW, HIGHFORW); // Rule No. 288
}

int file_exists_and_not_empty(const char *filename) {
  struct stat st;
  return (stat(filename, &st) == 0 && st.st_size > 0);
}

FILE *open_file_with_header(const char *filename, const char *header) {
  FILE *f = fopen(filename, "a");
  if (!file_exists_and_not_empty(filename)) {
    fprintf(f, "%s\n", header);
  }
  return f;
}

int main() {
    FILE *log_file = open_file_with_header(
      "/home/lab/Documents/fuzzy_output_log.csv",
      "Left,FrontLeft,Front,FrontRight,Right,DistancePerc,ObjectAngle,GoalAngle");
    
    FILE *left_file = open_file_with_header(
      "/home/lab/Documents/left_motor_output.csv",
      "LeftMotor");
    
    FILE *right_file = open_file_with_header(
      "/home/lab/Documents/right_motor_output.csv",
      "RightMotor");

    wb_robot_init();
    init_rules();
 
    int initialDistance = 0;
    
    
    WbDeviceTag lidar = wb_robot_get_device("lidar");
    wb_lidar_enable(lidar, TIME_STEP);

    WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
    WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");


    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_velocity(left_motor, 0);
    wb_motor_set_velocity(right_motor, 0);

    // double leftSpeed, rightSpeed;
    
    WbNodeRef robot = wb_supervisor_node_get_from_def("Khepera4"); //read robot position and orientation
    WbFieldRef rotationField = wb_supervisor_node_get_field(robot, "rotation");
    WbNodeRef robot_node = wb_supervisor_node_get_from_def("Khepera4");
    
    
    initialDistance = getDistance(-1.62617, -1.61453, goalX, goalY);
    
    
    
    while (wb_robot_step(TIME_STEP) != -1) {
      const double *rot = wb_supervisor_field_get_sf_rotation(rotationField);
      const double *orient = wb_supervisor_node_get_orientation(robot_node);
      double angleRad = rot[3];
      double angleDeg = angleRad * (180.0 / M_PI);
      const double *position = wb_supervisor_node_get_position(robot);
      angleDeg = getRobotHeading(angleDeg, orient[1]); // calculate robot face angle
      double objectAngle = convertAngle(getGoalAngle(position[0], position[1])); // calculate goal angle
      double distanceToGoal = getDistance(position[0], position[1], goalX, goalY); //calculate distance
      double angleDegConverted = convertAngle(angleDeg);
      
      
      
      double goalAngle = objectAngle - angleDegConverted;
      
      if (goalAngle > 180) {
        goalAngle -= 360;
      } else if (goalAngle < -180) {
          goalAngle += 360;
      }
      
      //dont use this for sensor Reading
      //if (initialDistance == 0){ // calculate initial distance for percentage
      //  initialDistance = getDistance(position[0], position[1], goalX, goalY);
      //}
      

      double distPerc =  (1 - (distanceToGoal / initialDistance));
      
      // *** Stopping function: check if the goal is reached ***
      if (distanceToGoal < GOAL_THRESHOLD) {
        wb_motor_set_velocity(left_motor, 0);
        wb_motor_set_velocity(right_motor, 0);
        printf("Goal reached! Distance to goal: %f\n", distanceToGoal);
        break;  // Exit the control loop
      }
      
      const float *range_image = wb_lidar_get_range_image(lidar); // for lidar
      double leftSensor = range_image[0]; //divide lidar regions 25 to compress into 5 regions only.
      for(int i=1; i<5; i++)
        if(range_image[i]<leftSensor) leftSensor=range_image[i];
  
      double frontLeftSensor= range_image[5];
      for(int i=6; i<10; i++)
        if(range_image[i]<frontLeftSensor) frontLeftSensor=range_image[i];
  
      double frontSensor= range_image[10];
      for(int i=11; i<15; i++)
        if(range_image[i]<frontSensor) frontSensor=range_image[i];
  
      double frontRightSensor= range_image[15];
      for(int i=16; i<20; i++)
        if(range_image[i]<frontRightSensor) frontRightSensor=range_image[i];
  
      double rightSensor= range_image[20];
      for(int i=21; i<24; i++)
        if(range_image[i]<rightSensor) rightSensor=range_image[i];
 
    // Fuzzy Evaluation
    double total_left = 0, total_right = 0, total_activation = 0;

     for (int i = 0; i < rule_count; i++) {
      Rule r = rules[i];
      double act = fmin(fmin(fmin(fmin(
        match_lidar(r.left, leftSensor),
        match_lidar(r.frontLeft, frontLeftSensor)),
        match_lidar(r.front, frontSensor)),
        match_lidar(r.frontRight, frontRightSensor)),
        match_lidar(r.right, rightSensor));

      act = fmin(act,
            fmin(match_distance(r.distance, distPerc),
                 match_angle(r.angle, goalAngle)));

      total_left += act * motor_value(r.leftMotor);
      total_right += act * motor_value(r.rightMotor);
      total_activation += act;
    }
    
    double leftSpeed = (total_activation > 0) ? (total_left / total_activation) * MAX_SPEED : 0;
    double rightSpeed = (total_activation > 0) ? (total_right / total_activation) * MAX_SPEED : 0;
    
    
    wb_motor_set_velocity(left_motor, leftSpeed);
    wb_motor_set_velocity(right_motor, rightSpeed);
    
    printf("L: %f, LF: %f, F: %f, FR: %f, R: %f, distPerc: %f, objectAngle: %f, GoalAngle: %f, LM: %f, RM: %f\n",
      leftSensor, frontLeftSensor, frontSensor, frontRightSensor, rightSensor,
      distPerc, objectAngle, goalAngle, leftSpeed, rightSpeed);

    fprintf(log_file, "%f,%f,%f,%f,%f,%f,%f,%f\n",
      leftSensor, frontLeftSensor, frontSensor, frontRightSensor, rightSensor, distPerc, objectAngle, goalAngle);

    fprintf(left_file, "%f\n", leftSpeed);

    fprintf(right_file, "%f\n", rightSpeed);
  
  
  }   
    fclose(log_file);
    wb_robot_cleanup();
    return 0;

}