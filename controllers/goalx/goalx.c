#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <stdio.h>
#include <webots/supervisor.h>
#include <webots/lidar.h>

#include <math.h>
#include <stdlib.h> // For abs()

#define TIME_STEP 64  // Set the simulation step

#define goalX 0.630298
#define goalY 0.644297


double getGoalAngle(double robotPositionX, double robotPositionY){
  double objectAngle;
  double computedAngle;
  
  if (goalX == robotPositionX){
   computedAngle = 0;
  }
  else{
    double angleRadians = atan((robotPositionY - goalY) / (robotPositionX - goalX));
    computedAngle = angleRadians * (180.0 / M_PI);  
  }

  if (goalX > robotPositionX){
    objectAngle = computedAngle;
  }
  else{
    objectAngle = 180 + computedAngle;
  }

  if (robotPositionX == goalX){
    if (robotPositionY < goalY){
      objectAngle = 270;
    }
    else{
      objectAngle = 90;
    }
  }
  
objectAngle = fmod(objectAngle + 360.0, 360.0);
return objectAngle;
}


int main() {
    wb_robot_init();  // Initialize Webots
    
    int time_step = (int)wb_robot_get_basic_time_step();
    int i;

    WbDeviceTag lidar = wb_robot_get_device("lidar");
    wb_lidar_enable(lidar, time_step);
    
    
    WbNodeRef robot = wb_supervisor_node_get_from_def("Khepera4");
    WbFieldRef rotationField = wb_supervisor_node_get_field(robot, "rotation");
    WbNodeRef robot_node = wb_supervisor_node_get_from_def("Khepera4");

    // Get motor devices
    WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
    WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");


    // Enable motors
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
   

    // Set initial velocity
    wb_motor_set_velocity(left_motor, 0.0);
    wb_motor_set_velocity(right_motor, 0.0);
    

    double objectAngle;

    // Main loop
    while (wb_robot_step(TIME_STEP) != -1) {

      const float *range_image = wb_lidar_get_range_image(lidar);
      double leftSensor = range_image[0];
      for(int i = 0; i < 5; i++){
        if(range_image[i] < leftSensor){
          leftSensor = range_image[i];
        }
      }

      double frontleftSensor = range_image[5];
      for(int i = 5; i < 10; i++){
        if(range_image[i] < frontleftSensor){
          frontleftSensor = range_image[i];
        }
      }

      double frontSensor = range_image[10];
      for(int i = 10; i < 15; i++){
        if(range_image[i] < frontSensor){
          frontSensor = range_image[i];
        }
      }

      double frontrightSensor = range_image[15];
      for(int i = 15; i < 20; i++){
        if(range_image[i] < frontrightSensor){
          frontrightSensor = range_image[i];
        }
      }

      double rightSensor = range_image[10];
      for(int i = 20; i < 25; i++){
        if(range_image[i] < rightSensor){
          rightSensor = range_image[i];
        }
      }
    
      const double *rot = wb_supervisor_field_get_sf_rotation(rotationField);
      const double *orient = wb_supervisor_node_get_orientation(robot_node);
      
      double angleRad = rot[3];
      double angleDeg = angleRad * (180.0 / M_PI);
      const double *position = wb_supervisor_node_get_position(robot);
      
      double up_y = orient[1]; // can tell if the robot is face up or down
      
    
      if (angleDeg < 0){ // reading from the webots, and converting 0-180 and 180-0
        angleDeg = angleDeg *-1; //since naay negative 180-0
      }
      else{
        angleDeg=angleDeg;
      }
      
      if (up_y > 0 ){ // converting 180-0 range of degree when robot is facing below
      angleDeg = (180-angleDeg)+180;
      }
      //printf("Angle = %.2f\n", angleDeg);
      
      objectAngle = getGoalAngle(position[0], position[1]);
      //printf("Object angle: %f\n", objectAngle);
      

    
      if ( abs(angleDeg-objectAngle)<=3 ){
        wb_motor_set_velocity(left_motor, 6.0);
        wb_motor_set_velocity(right_motor, 6.0);
        printf("Face, Kep angle: %.3f , goal: %.3f\n",
            angleDeg, objectAngle);
      }
      

      else if (objectAngle <= 270 && objectAngle > 90 ){ //turn left
          //printf("Turn left\n");
          wb_motor_set_velocity(left_motor, -2.0);
          wb_motor_set_velocity(right_motor, 2.0);
          printf("Left, Kep angle: %.3f , goal: %.3f\n",
            angleDeg, objectAngle);
        }
        
      else if (objectAngle <= 90 || objectAngle > 270){ 
        //("Turn right");
        wb_motor_set_velocity(left_motor, 2.0);
        wb_motor_set_velocity(right_motor, -2.0);
        printf("Right, Kep angle: %.3f , goal: %.3f\n",
            angleDeg, objectAngle);
      }
    
      printf("Left: %.3f, FrontLeft: %.3f, Front: %.3f, FrontRight: %.3f, Right: %.3f\n", leftSensor, frontleftSensor, frontSensor, frontrightSensor, rightSensor);
    } // from while time step
    
    wb_robot_cleanup();
    return 0;
} // from int main