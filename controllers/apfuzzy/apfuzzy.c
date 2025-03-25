#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define MAX_SPEED 47.6
#define STOP_THRESHOLD 0.02
#define WHEEL_RADIUS 0.021
#define AXLE_LENGTH 0.105

// Goal position
#define GOAL_X 1
#define GOAL_Y 1

// APF parameters
#define ZETA 1.1547
#define ETA 0.5  // Increased repulsive force gain
#define DSTAR 0.5
#define QSTAR 0.75
#define KP_OMEGA 1.5
#define OMEGA_MAX (0.5 * M_PI)

#define NUMBER_OF_ULTRASONIC_SENSORS 5
static const char *ultrasonic_sensors_names[NUMBER_OF_ULTRASONIC_SENSORS] = {
  "left ultrasonic sensor", "front left ultrasonic sensor", "front ultrasonic sensor", "front right ultrasonic sensor",
  "right ultrasonic sensor"};

// Robot state
static double x_r = 0.0, y_r = 0.0, theta = 0.0;

// Normalizing angles
static double normalize_angle(double angle) {
  while (angle > M_PI) angle -= 2 * M_PI;
  while (angle < -M_PI) angle += 2 * M_PI;
  return angle;
}

// Compute repulsive force from ultrasonic sensors
static void compute_repulsive_force(double *nablaU_rep_x, double *nablaU_rep_y) {
  *nablaU_rep_x = 0;
  *nablaU_rep_y = 0;

  for (int i = 0; i < NUMBER_OF_ULTRASONIC_SENSORS; i++) {
    WbDeviceTag sensor = wb_robot_get_device(ultrasonic_sensors_names[i]);
    double dist = wb_distance_sensor_get_value(sensor);

    if (dist < QSTAR) {
      double angle = theta + i * M_PI / 4; // Adjust based on sensor orientation
      double obs_x = x_r + dist * cos(angle);
      double obs_y = y_r + dist * sin(angle);
      double rep_scale = ETA * (1 / QSTAR - 1 / dist) * pow(1 / dist, 2);
      *nablaU_rep_x += rep_scale * (x_r - obs_x);
      *nablaU_rep_y += rep_scale * (y_r - obs_y);
    }
  }
}

// Fuzzy logic membership functions
double membership_close(double x) {
  if (x <= 0.5) return 1;
  if (x > 0.5 && x < 1) return (1 - x) / 0.5;
  return 0;
}

double membership_medium(double x) {
  if (x > 0.5 && x <= 1.5) return (x - 0.5) / 1.0;
  if (x > 1.5 && x < 2) return (2 - x) / 0.5;
  return 0;
}

double membership_far(double x) {
  if (x >= 1.5) return 1;
  if (x > 1 && x < 1.5) return (x - 1) / 0.5;
  return 0;
}

// Fuzzy logic control
void fuzzy_control(double distance_to_goal, double angle_to_goal, double distance_to_obstacle, double *v, double *w) {
  // Fuzzify inputs
  double close = membership_close(distance_to_goal);
  double medium = membership_medium(distance_to_goal);
  double far = membership_far(distance_to_goal);

  // Apply fuzzy rules
  if (close > 0.5 && distance_to_obstacle > 1) {
    *v = MAX_SPEED;
    *w = 0;
  } else if (far > 0.5 && distance_to_obstacle < 0.5) {
    *v = 0;
    *w = MAX_SPEED;
  } else {
    *v = MAX_SPEED * 0.5;
    *w = 0;
  }
}

// Compute APF control signals
static void compute_apf_control(double *v, double *w) {
  double dx = GOAL_X - x_r;
  double dy = GOAL_Y - y_r;
  double distance = sqrt(dx * dx + dy * dy);

  // Attractive potential
  double nablaU_att_x, nablaU_att_y;
  if (distance <= DSTAR) {
    nablaU_att_x = ZETA * (x_r - GOAL_X);
    nablaU_att_y = ZETA * (y_r - GOAL_Y);
  } else {
    double scale = DSTAR / distance;
    nablaU_att_x = scale * ZETA * (x_r - GOAL_X);
    nablaU_att_y = scale * ZETA * (y_r - GOAL_Y);
  }

  // Repulsive potential (dynamic obstacles from sensors)
  double nablaU_rep_x, nablaU_rep_y;
  compute_repulsive_force(&nablaU_rep_x, &nablaU_rep_y);

  // Compute final potential gradient
  double nablaU_x = nablaU_att_x + nablaU_rep_x;
  double nablaU_y = nablaU_att_y + nablaU_rep_y;

  // Debug prints
  printf("Attractive Force: (%.3f, %.3f)\n", nablaU_att_x, nablaU_att_y);
  printf("Repulsive Force: (%.3f, %.3f)\n", nablaU_rep_x, nablaU_rep_y);
  printf("Total Force: (%.3f, %.3f)\n", nablaU_x, nablaU_y);

  // Compute reference angle and velocity
  double theta_ref = atan2(-nablaU_y, -nablaU_x);
  double error_theta = normalize_angle(theta_ref - theta);

  if (fabs(error_theta) < M_PI / 4) {
    *v = fmin(MAX_SPEED * (1 - fabs(error_theta) / (M_PI / 4)), MAX_SPEED);
  } else {
    *v = 0;
  }

  *w = KP_OMEGA * error_theta;
  *w = fmax(fmin(*w, OMEGA_MAX), -OMEGA_MAX);
}

int main() {
  wb_robot_init();
  int time_step = (int)wb_robot_get_basic_time_step();

  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  WbDeviceTag left_encoder = wb_robot_get_device("left wheel sensor");
  WbDeviceTag right_encoder = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_encoder, time_step);
  wb_position_sensor_enable(right_encoder, time_step);

  // Enable ultrasonic sensors
  WbDeviceTag ultrasonic_sensors[NUMBER_OF_ULTRASONIC_SENSORS];
  for (int i = 0; i < NUMBER_OF_ULTRASONIC_SENSORS; i++) {
    ultrasonic_sensors[i] = wb_robot_get_device(ultrasonic_sensors_names[i]);
    wb_distance_sensor_enable(ultrasonic_sensors[i], time_step);
  }

  double last_left_pos = 0.0, last_right_pos = 0.0;

  while (wb_robot_step(time_step) != -1) {
    double left_pos = wb_position_sensor_get_value(left_encoder);
    double right_pos = wb_position_sensor_get_value(right_encoder);

    double d_left = (left_pos - last_left_pos) * WHEEL_RADIUS;
    double d_right = (right_pos - last_right_pos) * WHEEL_RADIUS;
    double d_center = (d_left + d_right) / 2.0;
    double d_theta = (d_right - d_left) / AXLE_LENGTH;

    theta += d_theta;
    x_r += d_center * cos(theta);
    y_r += d_center * sin(theta);

    last_left_pos = left_pos;
    last_right_pos = right_pos;

    double v = 0.0, w = 0.0;
    compute_apf_control(&v, &w);

    if (sqrt(pow(GOAL_X - x_r, 2) + pow(GOAL_Y - y_r, 2)) < STOP_THRESHOLD) {
      wb_motor_set_velocity(left_motor, 0);
      wb_motor_set_velocity(right_motor, 0);
      printf("Yay!!! Goal reached at (%.3f, %.3f)\n", x_r, y_r);
      break;
    }

    double left_speed = (v - w * AXLE_LENGTH / 2.0);
    double right_speed = (v + w * AXLE_LENGTH / 2.0);

    left_speed = fmax(fmin(left_speed, MAX_SPEED), -MAX_SPEED);
    right_speed = fmax(fmin(right_speed, MAX_SPEED), -MAX_SPEED);

    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);

    printf("Position: x = %.3f, y = %.3f, theta = %.3f rad\n", x_r, y_r, theta);
  }

  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
  wb_robot_cleanup();
  return EXIT_SUCCESS;
}