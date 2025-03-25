#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/lidar.h>
#include <stdio.h>
#include <math.h>
#include <webots/supervisor.h>
#include <stdlib.h>
#include <string.h>


#define TIME_STEP 64
#define MAX_SPEED 40  // Khepera4 max speed

#define goalX 0 
#define goalY 0  

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


typedef enum { NEAR, FAR } DistanceTerm;
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
// Near: trimf [0.0, 0.5, 1.0]
static double mf_lidar_near(float x) {
  if (x <= 0.0 || x >= 1.0) return 0.0;
  if (x <= 0.5) return (x - 0.0) / (0.5 - 0.0);  // rising edge
  return (1.0 - x) / (1.0 - 0.5);                // falling edge
}

// Far: trimf [0.6, 1.1, 2.0]
static double mf_lidar_far(float x) {
  if (x <= 0.6 || x >= 2.0) return 0.0;
  if (x <= 1.1) return (x - 0.6) / (1.1 - 0.6);  // rising edge
  return (2.0 - x) / (2.0 - 1.1);                // falling edge
}


// For Distance Input
// Distance: Far [0.4, 0.7, 1.0]
static double distance_far(float x) {
  if (x <= 0.4 || x >= 1.0) return 0.0;
  if (x <= 0.7) return (x - 0.4) / (0.7 - 0.4);
  return (1.0 - x) / (1.0 - 0.7);
}

// Distance: Near [0.0, 0.2, 0.4]
static double distance_near(float x) {
  if (x <= 0.0 || x >= 0.4) return 0.0;
  if (x <= 0.2) return (x - 0.0) / (0.2 - 0.0);
  return (0.4 - x) / (0.4 - 0.2);
}

// For Angle input
// Angle: Right [-179, -90, -1]
static double angle_right(float x) {
  if (x <= -179.0 || x >= -1.0) return 0.0;
  if (x < -90.0) return (x + 179.0) / (89.0);     // rising: -179 to -90
  return (-1.0 - x) / (89.0);                     // falling: -90 to -1
}

// Angle: Straight [-15, 0, 15]
static double angle_straight(float x) {
  if (x <= -15.0 || x >= 15.0) return 0.0;
  if (x < 0.0) return (x + 15.0) / 15.0;          // rising: -15 to 0
  return (15.0 - x) / 15.0;                       // falling: 0 to 15
}

// Angle: Left [1, 90, 179]
static double angle_left(float x) {
  if (x <= 1.0 || x >= 179.0) return 0.0;
  if (x < 90.0) return (x - 1.0) / (89.0);         // rising: 1 to 90
  return (179.0 - x) / (89.0);                    // falling: 90 to 179
}


// Match Membership
double match_lidar(LidarTerm term, double value) {
  return term == LIDAR_NEAR ? mf_lidar_near(value) : mf_lidar_far(value);
}

double match_distance(DistanceTerm term, double value) {
  if (term == NEAR) return distance_near(value);
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
  rules[rule_count++] = makeRule(FAR,FAR,FAR,FAR,FAR,FAR,STRAIGHT, HIGHFORW, HIGHFORW); // Rule No. 1
  rules[rule_count++] = makeRule(FAR,FAR,FAR,FAR,FAR,FAR,RIGHT, HIGHFORW, MEDREV); // Rule No. 2
  rules[rule_count++] = makeRule(FAR,FAR,FAR,FAR,FAR,FAR,LEFT, MEDREV, HIGHFORW); // Rule No. 3
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,FAR,FAR,FAR,STRAIGHT, HIGHFORW, MEDFORW); // Rule No. 4
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,FAR,FAR,FAR,STRAIGHT, HIGHFORW, LOWFORW); // Rule No. 5
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,FAR,FAR,FAR,STRAIGHT, HIGHFORW, MEDREV); // Rule No. 6
  rules[rule_count++] = makeRule(FAR,FAR,FAR,NEAR,FAR,FAR,STRAIGHT, LOWFORW, HIGHFORW); // Rule No. 7
  rules[rule_count++] = makeRule(FAR,FAR,FAR,FAR,NEAR,FAR,STRAIGHT, MEDFORW, HIGHFORW); // Rule No. 8
  rules[rule_count++] = makeRule(FAR,FAR,FAR,FAR,FAR,NEAR,STRAIGHT, LOWFORW, LOWFORW); // Rule No. 9
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,FAR,FAR,FAR,RIGHT, HIGHFORW, MEDFORW); // Rule No. 10
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,FAR,FAR,FAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 11
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,FAR,FAR,FAR,RIGHT, HIGHFORW, MEDREV); // Rule No. 12
  rules[rule_count++] = makeRule(FAR,FAR,FAR,NEAR,FAR,FAR,RIGHT, LOWFORW, HIGHFORW); // Rule No. 13
  rules[rule_count++] = makeRule(FAR,FAR,FAR,FAR,NEAR,FAR,RIGHT, MEDFORW, HIGHFORW); // Rule No. 14
  rules[rule_count++] = makeRule(FAR,FAR,FAR,FAR,FAR,NEAR,RIGHT, MEDFORW, LOWREV); // Rule No. 15
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,FAR,FAR,FAR,LEFT, HIGHFORW, MEDFORW); // Rule No. 16
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,FAR,FAR,FAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 17
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,FAR,FAR,FAR,LEFT, MEDREV, HIGHFORW); // Rule No. 18
  rules[rule_count++] = makeRule(FAR,FAR,FAR,NEAR,FAR,FAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 19
  rules[rule_count++] = makeRule(FAR,FAR,FAR,FAR,NEAR,FAR,LEFT, MEDFORW, HIGHFORW); // Rule No. 20
  rules[rule_count++] = makeRule(FAR,FAR,FAR,FAR,FAR,NEAR,LEFT, LOWREV, MEDFORW); // Rule No. 21
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,FAR,FAR,FAR,STRAIGHT, MEDFORW, LOWFORW); // Rule No. 22
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,FAR,FAR,FAR,STRAIGHT, HIGHFORW, MEDREV); // Rule No. 23
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,NEAR,FAR,FAR,STRAIGHT, MEDFORW, HIGHFORW); // Rule No. 24
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,FAR,NEAR,FAR,STRAIGHT, HIGHFORW, HIGHFORW); // Rule No. 25
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,FAR,FAR,NEAR,STRAIGHT, MEDFORW, LOWFORW); // Rule No. 26
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,FAR,FAR,FAR,STRAIGHT, HIGHFORW, MEDREV); // Rule No. 27
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,NEAR,FAR,FAR,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 28
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,FAR,NEAR,FAR,STRAIGHT, MEDFORW, LOWFORW); // Rule No. 29
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,FAR,FAR,NEAR,STRAIGHT, MEDFORW, LOWFORW); // Rule No. 30
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,NEAR,FAR,FAR,STRAIGHT, HIGHFORW, MEDREV); // Rule No. 31
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,FAR,NEAR,FAR,STRAIGHT, MEDFORW, LOWFORW); // Rule No. 32
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,FAR,FAR,NEAR,STRAIGHT, MEDFORW, LOWREV); // Rule No. 33
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,NEAR,NEAR,FAR,STRAIGHT, LOWREV, MEDREV); // Rule No. 34
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,NEAR,FAR,NEAR,STRAIGHT, MEDFORW, MEDREV); // Rule No. 35
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,NEAR,NEAR,NEAR,STRAIGHT, LOWREV, MEDREV); // Rule No. 36
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,FAR,FAR,FAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 37
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,FAR,FAR,FAR,RIGHT, HIGHFORW, MEDREV); // Rule No. 38
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,NEAR,FAR,FAR,RIGHT, LOWFORW, HIGHFORW); // Rule No. 39
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,FAR,NEAR,FAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 40
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,FAR,FAR,NEAR,RIGHT, MEDFORW, LOWREV); // Rule No. 41
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,FAR,FAR,FAR,RIGHT, MEDFORW, MEDREV); // Rule No. 42
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,NEAR,FAR,FAR,RIGHT, MEDFORW, MEDFORW); // Rule No. 43
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,FAR,NEAR,FAR,RIGHT, HIGHFORW, LOWFORW); // Rule No. 44
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,FAR,FAR,NEAR,RIGHT, MEDFORW, LOWREV); // Rule No. 45
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,NEAR,FAR,FAR,RIGHT, HIGHFORW, MEDREV); // Rule No. 46
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,FAR,NEAR,FAR,RIGHT, MEDREV, LOWREV); // Rule No. 47
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,FAR,FAR,NEAR,RIGHT, MEDFORW, LOWREV); // Rule No. 48
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,NEAR,NEAR,FAR,RIGHT, LOWREV, MEDREV); // Rule No. 49
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,NEAR,FAR,NEAR,RIGHT, HIGHFORW, MEDREV); // Rule No. 50
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,NEAR,NEAR,NEAR,RIGHT, LOWREV, MEDREV); // Rule No. 51
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,FAR,FAR,FAR,LEFT, MEDFORW, LOWFORW); // Rule No. 52
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,FAR,FAR,FAR,LEFT, MEDREV, MEDFORW); // Rule No. 53
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,NEAR,FAR,FAR,LEFT, MEDREV, MEDREV); // Rule No. 54
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,FAR,NEAR,FAR,LEFT, LOWREV, HIGHFORW); // Rule No. 55
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,FAR,FAR,NEAR,LEFT, HIGHFORW, MEDFORW); // Rule No. 56
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,FAR,FAR,FAR,LEFT, MEDREV, MEDFORW); // Rule No. 57
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,NEAR,FAR,FAR,LEFT, LOWFORW, MEDFORW); // Rule No. 58
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,FAR,NEAR,FAR,LEFT, HIGHFORW, HIGHFORW); // Rule No. 59
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,FAR,FAR,NEAR,LEFT, LOWFORW, MEDFORW); // Rule No. 60
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,NEAR,FAR,FAR,LEFT, MEDREV, HIGHFORW); // Rule No. 61
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,FAR,NEAR,FAR,LEFT, MEDFORW, LOWREV); // Rule No. 62
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,FAR,FAR,NEAR,LEFT, HIGHFORW, MEDREV); // Rule No. 63
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,NEAR,NEAR,FAR,LEFT, MEDREV, LOWREV); // Rule No. 64
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,NEAR,FAR,NEAR,LEFT, MEDREV, LOWREV); // Rule No. 65
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,NEAR,NEAR,NEAR,LEFT, MEDREV, LOWREV); // Rule No. 66
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,FAR,FAR,FAR,STRAIGHT, HIGHFORW, LOWFORW); // Rule No. 67
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,NEAR,FAR,FAR,STRAIGHT, HIGHFORW, HIGHFORW); // Rule No. 68
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,FAR,NEAR,FAR,STRAIGHT, HIGHFORW, MEDFORW); // Rule No. 69
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,FAR,FAR,NEAR,STRAIGHT, LOWFORW, LOWREV); // Rule No. 70
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,NEAR,FAR,FAR,STRAIGHT, HIGHFORW, MEDREV); // Rule No. 71
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,FAR,NEAR,FAR,STRAIGHT, HIGHFORW, LOWFORW); // Rule No. 72
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,FAR,FAR,NEAR,STRAIGHT, MEDFORW, LOWREV); // Rule No. 73
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,NEAR,NEAR,FAR,STRAIGHT, MEDREV, HIGHFORW); // Rule No. 74
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,NEAR,FAR,NEAR,STRAIGHT, HIGHFORW, MEDREV); // Rule No. 75
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,NEAR,NEAR,NEAR,STRAIGHT, MEDREV, LOWREV); // Rule No. 76
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,FAR,FAR,FAR,RIGHT, HIGHFORW, MEDREV); // Rule No. 77
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,NEAR,FAR,FAR,RIGHT, HIGHFORW, HIGHFORW); // Rule No. 78
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,FAR,NEAR,FAR,RIGHT, HIGHFORW, MEDREV); // Rule No. 79
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,FAR,FAR,NEAR,RIGHT, MEDFORW, LOWREV); // Rule No. 80
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,NEAR,FAR,FAR,RIGHT, HIGHFORW, MEDREV); // Rule No. 81
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,FAR,NEAR,FAR,RIGHT, MEDFORW, LOWREV); // Rule No. 82
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,FAR,FAR,NEAR,RIGHT, MEDFORW, LOWFORW); // Rule No. 83
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,NEAR,NEAR,FAR,RIGHT, MEDREV, MEDFORW); // Rule No. 84
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,NEAR,FAR,NEAR,RIGHT, MEDFORW, LOWREV); // Rule No. 85
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,NEAR,NEAR,NEAR,RIGHT, MEDREV, MEDFORW); // Rule No. 86
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,FAR,FAR,FAR,LEFT, MEDFORW, MEDREV); // Rule No. 87
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,NEAR,FAR,FAR,LEFT, MEDFORW, LOWFORW); // Rule No. 88
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,FAR,NEAR,FAR,LEFT, MEDFORW, MEDFORW); // Rule No. 89
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,FAR,FAR,NEAR,LEFT, MEDFORW, LOWFORW); // Rule No. 90
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,NEAR,FAR,FAR,LEFT, MEDREV, MEDFORW); // Rule No. 91
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,FAR,NEAR,FAR,LEFT, MEDREV, HIGHFORW); // Rule No. 92
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,FAR,FAR,NEAR,LEFT, MEDREV, MEDFORW); // Rule No. 93
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,NEAR,NEAR,FAR,LEFT, MEDREV, MEDFORW); // Rule No. 94
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,NEAR,FAR,NEAR,LEFT, MEDREV, MEDFORW); // Rule No. 95
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,NEAR,NEAR,NEAR,LEFT, MEDREV, MEDFORW); // Rule No. 96
  rules[rule_count++] = makeRule(FAR,FAR,FAR,FAR,NEAR,NEAR,RIGHT, MEDFORW, LOWFORW); // Rule No. 97
  rules[rule_count++] = makeRule(FAR,FAR,FAR,FAR,NEAR,NEAR,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 98
  rules[rule_count++] = makeRule(FAR,FAR,FAR,FAR,NEAR,NEAR,LEFT, LOWFORW, MEDFORW); // Rule No. 99
  rules[rule_count++] = makeRule(FAR,FAR,FAR,NEAR,FAR,NEAR,RIGHT, MEDFORW, LOWREV); // Rule No. 100
  rules[rule_count++] = makeRule(FAR,FAR,FAR,NEAR,FAR,NEAR,STRAIGHT, LOWREV, MEDFORW); // Rule No. 101
  rules[rule_count++] = makeRule(FAR,FAR,FAR,NEAR,FAR,NEAR,LEFT, LOWREV, MEDFORW); // Rule No. 102
  rules[rule_count++] = makeRule(FAR,FAR,FAR,NEAR,NEAR,FAR,RIGHT, LOWFORW, HIGHFORW); // Rule No. 103
  rules[rule_count++] = makeRule(FAR,FAR,FAR,NEAR,NEAR,FAR,STRAIGHT, LOWFORW, HIGHFORW); // Rule No. 104
  rules[rule_count++] = makeRule(FAR,FAR,FAR,NEAR,NEAR,FAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 105
  rules[rule_count++] = makeRule(FAR,FAR,FAR,NEAR,NEAR,NEAR,RIGHT, LOWFORW, MEDFORW); // Rule No. 106
  rules[rule_count++] = makeRule(FAR,FAR,FAR,NEAR,NEAR,NEAR,STRAIGHT, LOWFORW, MEDFORW); // Rule No. 107
  rules[rule_count++] = makeRule(FAR,FAR,FAR,NEAR,NEAR,NEAR,LEFT, LOWFORW, MEDFORW); // Rule No. 108
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,FAR,FAR,NEAR,RIGHT, MEDFORW, LOWFORW); // Rule No. 109
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,FAR,FAR,NEAR,STRAIGHT, MEDFORW, LOWREV); // Rule No. 110
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,FAR,FAR,NEAR,LEFT, LOWFORW, MEDFORW); // Rule No. 111
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,FAR,NEAR,FAR,RIGHT, MEDFORW, LOWFORW); // Rule No. 112
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,FAR,NEAR,FAR,STRAIGHT, MEDFORW, LOWFORW); // Rule No. 113
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,FAR,NEAR,FAR,LEFT, LOWFORW, MEDFORW); // Rule No. 114
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,FAR,NEAR,NEAR,RIGHT, HIGHFORW, HIGHFORW); // Rule No. 115
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,FAR,NEAR,NEAR,STRAIGHT, HIGHFORW, HIGHFORW); // Rule No. 116
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,FAR,NEAR,NEAR,LEFT, LOWFORW, MEDFORW); // Rule No. 117
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,NEAR,FAR,FAR,RIGHT, MEDFORW, MEDREV); // Rule No. 118
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,NEAR,FAR,FAR,STRAIGHT, LOWFORW, HIGHFORW); // Rule No. 119
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,NEAR,FAR,FAR,LEFT, LOWREV, HIGHFORW); // Rule No. 120
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,NEAR,FAR,NEAR,RIGHT, MEDFORW, LOWREV); // Rule No. 121
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,NEAR,FAR,NEAR,STRAIGHT, LOWREV, MEDFORW); // Rule No. 122
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,NEAR,FAR,NEAR,LEFT, LOWREV, MEDFORW); // Rule No. 123
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,NEAR,NEAR,FAR,RIGHT, LOWREV, HIGHFORW); // Rule No. 124
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,NEAR,NEAR,FAR,STRAIGHT, LOWFORW, HIGHFORW); // Rule No. 125
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,NEAR,NEAR,FAR,LEFT, LOWREV, HIGHFORW); // Rule No. 126
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,NEAR,NEAR,NEAR,RIGHT, LOWREV, MEDFORW); // Rule No. 127
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,NEAR,NEAR,NEAR,STRAIGHT, LOWREV, MEDFORW); // Rule No. 128
  rules[rule_count++] = makeRule(FAR,FAR,NEAR,NEAR,NEAR,NEAR,LEFT, LOWFORW, MEDFORW); // Rule No. 129
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,FAR,NEAR,NEAR,RIGHT, HIGHFORW, HIGHFORW); // Rule No. 130
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,FAR,NEAR,NEAR,STRAIGHT, HIGHFORW, HIGHFORW); // Rule No. 131
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,FAR,NEAR,NEAR,LEFT, MEDFORW, LOWFORW); // Rule No. 132
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,NEAR,FAR,NEAR,RIGHT, LOWFORW, LOWREV); // Rule No. 133
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,NEAR,FAR,NEAR,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 134
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,NEAR,FAR,NEAR,LEFT, MEDREV, HIGHFORW); // Rule No. 135
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,NEAR,NEAR,FAR,RIGHT, HIGHFORW, HIGHFORW); // Rule No. 136
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,NEAR,NEAR,FAR,STRAIGHT, HIGHFORW, HIGHFORW); // Rule No. 137
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,NEAR,NEAR,FAR,LEFT, MEDREV, HIGHFORW); // Rule No. 138
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,NEAR,NEAR,NEAR,RIGHT, MEDFORW, MEDFORW); // Rule No. 139
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,NEAR,NEAR,NEAR,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 140
  rules[rule_count++] = makeRule(FAR,NEAR,FAR,NEAR,NEAR,NEAR,LEFT, MEDREV, HIGHFORW); // Rule No. 141
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,FAR,NEAR,NEAR,RIGHT, MEDFORW, LOWREV); // Rule No. 142
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,FAR,NEAR,NEAR,STRAIGHT, MEDFORW, LOWREV); // Rule No. 143
  rules[rule_count++] = makeRule(FAR,NEAR,NEAR,FAR,NEAR,NEAR,LEFT, MEDREV, HIGHFORW); // Rule No. 144
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,FAR,NEAR,NEAR,RIGHT, MEDFORW, MEDFORW); // Rule No. 145
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,FAR,NEAR,NEAR,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 146
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,FAR,NEAR,NEAR,LEFT, MEDFORW, HIGHFORW); // Rule No. 147
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,NEAR,FAR,NEAR,RIGHT, MEDFORW, LOWFORW); // Rule No. 148
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,NEAR,FAR,NEAR,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 149
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,NEAR,FAR,NEAR,LEFT, LOWFORW, MEDFORW); // Rule No. 150
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,NEAR,NEAR,FAR,RIGHT, HIGHFORW, HIGHFORW); // Rule No. 151
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,NEAR,NEAR,FAR,STRAIGHT, MEDFORW, HIGHFORW); // Rule No. 152
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,NEAR,NEAR,FAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 153
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,NEAR,NEAR,NEAR,RIGHT, LOWFORW, MEDFORW); // Rule No. 154
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,NEAR,NEAR,NEAR,STRAIGHT, HIGHFORW, HIGHFORW); // Rule No. 155
  rules[rule_count++] = makeRule(NEAR,FAR,FAR,NEAR,NEAR,NEAR,LEFT, LOWREV, HIGHFORW); // Rule No. 156
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,FAR,FAR,NEAR,RIGHT, MEDFORW, LOWREV); // Rule No. 157
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,FAR,FAR,NEAR,STRAIGHT, MEDFORW, LOWFORW); // Rule No. 158
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,FAR,FAR,NEAR,LEFT, LOWREV, MEDFORW); // Rule No. 159
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,FAR,NEAR,FAR,RIGHT, HIGHFORW, MEDFORW); // Rule No. 160
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,FAR,NEAR,FAR,STRAIGHT, HIGHFORW, LOWFORW); // Rule No. 161
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,FAR,NEAR,FAR,LEFT, MEDFORW, HIGHFORW); // Rule No. 162
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,FAR,NEAR,NEAR,RIGHT, MEDFORW, LOWFORW); // Rule No. 163
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,FAR,NEAR,NEAR,STRAIGHT, MEDFORW, LOWFORW); // Rule No. 164
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,FAR,NEAR,NEAR,LEFT, LOWFORW, MEDFORW); // Rule No. 165
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,NEAR,FAR,FAR,RIGHT, HIGHFORW, MEDREV); // Rule No. 166
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,NEAR,FAR,FAR,STRAIGHT, LOWREV, MEDFORW); // Rule No. 167
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,NEAR,FAR,FAR,LEFT, LOWFORW, HIGHFORW); // Rule No. 168
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,NEAR,FAR,NEAR,RIGHT, LOWREV, MEDFORW); // Rule No. 169
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,NEAR,FAR,NEAR,STRAIGHT, LOWREV, MEDFORW); // Rule No. 170
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,NEAR,FAR,NEAR,LEFT, LOWREV, MEDFORW); // Rule No. 171
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,NEAR,NEAR,FAR,RIGHT, LOWREV, HIGHFORW); // Rule No. 172
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,NEAR,NEAR,FAR,STRAIGHT, LOWREV, HIGHFORW); // Rule No. 173
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,NEAR,NEAR,FAR,LEFT, LOWREV, HIGHFORW); // Rule No. 174
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,NEAR,NEAR,NEAR,RIGHT, LOWREV, MEDFORW); // Rule No. 175
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,NEAR,NEAR,NEAR,STRAIGHT, LOWREV, MEDFORW); // Rule No. 176
  rules[rule_count++] = makeRule(NEAR,FAR,NEAR,NEAR,NEAR,NEAR,LEFT, LOWREV, MEDFORW); // Rule No. 177
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,FAR,NEAR,NEAR,RIGHT, MEDFORW, LOWFORW); // Rule No. 178
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,FAR,NEAR,NEAR,STRAIGHT, LOWFORW, LOWFORW); // Rule No. 179
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,FAR,NEAR,NEAR,LEFT, LOWFORW, MEDFORW); // Rule No. 180
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,NEAR,FAR,NEAR,RIGHT, HIGHFORW, LOWREV); // Rule No. 181
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,NEAR,FAR,NEAR,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 182
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,NEAR,FAR,NEAR,LEFT, LOWFORW, MEDFORW); // Rule No. 183
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,NEAR,NEAR,FAR,RIGHT, HIGHFORW, HIGHFORW); // Rule No. 184
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,NEAR,NEAR,FAR,STRAIGHT, HIGHFORW, HIGHFORW); // Rule No. 185
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,NEAR,NEAR,FAR,LEFT, HIGHFORW, HIGHFORW); // Rule No. 186
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,NEAR,NEAR,NEAR,RIGHT, MEDFORW, MEDFORW); // Rule No. 187
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,NEAR,NEAR,NEAR,STRAIGHT, MEDFORW, MEDFORW); // Rule No. 188
  rules[rule_count++] = makeRule(NEAR,NEAR,FAR,NEAR,NEAR,NEAR,LEFT, MEDFORW, MEDFORW); // Rule No. 189
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,FAR,NEAR,NEAR,RIGHT, MEDFORW, LOWFORW); // Rule No. 190
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,FAR,NEAR,NEAR,STRAIGHT, MEDFORW, LOWFORW); // Rule No. 191
  rules[rule_count++] = makeRule(NEAR,NEAR,NEAR,FAR,NEAR,NEAR,LEFT, MEDFORW, LOWFORW); // Rule No. 192
}


  

int main() {
  wb_robot_init();
  init_rules();

  int initialDistance = 0;
  
  
  WbDeviceTag lidar = wb_robot_get_device("lidar");
  wb_lidar_enable(lidar, TIME_STEP);
  wb_lidar_enable_point_cloud(lidar);


  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");


  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);

  
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
    
    double distPerc =  (1 - (distanceToGoal / initialDistance));
    
    // *** Stopping function: check if the goal is reached ***
    if (distanceToGoal < GOAL_THRESHOLD) {
      wb_motor_set_velocity(left_motor, 0);
      wb_motor_set_velocity(right_motor, 0);
      printf("Goal reached! Distance to goal: %f\n", distanceToGoal);
      break;  
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

    if (act > 0.0) {
      printf("Rule %d activated: act = %f\n", i + 1, act);
    }
  }

  
  
  double leftSpeed = (total_activation > 0) ? (total_left / total_activation) * MAX_SPEED : 0;
  double rightSpeed = (total_activation > 0) ? (total_right / total_activation) * MAX_SPEED : 0;
  
  
  wb_motor_set_velocity(left_motor, leftSpeed);
  wb_motor_set_velocity(right_motor, rightSpeed);
  
  printf("L: %f, LF: %f, F: %f, FR: %f, R: %f, distPerc: %f, objectAngle: %f, GoalAngle: %f, LM: %f, RM: %f\n",
    leftSensor, frontLeftSensor, frontSensor, frontRightSensor, rightSensor,
    distPerc, objectAngle, goalAngle, leftSpeed, rightSpeed);


}
  wb_robot_cleanup();
  return 0;

}


