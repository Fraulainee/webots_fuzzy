# Khepera IV Fuzzy-APF Navigation Controller

A Python-based controller for the Khepera IV robot that combines Artificial Potential Fields (APF) with Fuzzy Logic Control for navigation.

## Features
- Goal-seeking behavior using attractive potential fields
- Obstacle avoidance using repulsive potential fields
- Fuzzy logic control for smooth motion transitions
- Position tracking using wheel encoders
- Visualization of fuzzy membership functions

## Dependencies
Install required Python packages:
```bash
pip install -r requirements.txt
```

## Parameters
- Physical Parameters:
  - Wheel Radius: 0.021 meters
  - Axle Length: 0.105 meters

- Control Parameters:
  - Max Speed: 47.6
  - Attractive Force Constant: 1.0
  - Repulsive Force Constant: 1.0

## Debug Output
The controller provides real-time debug information:
- Robot position and heading
- Distance and angle to goal
- Attractive and repulsive forces
- Motor speeds
- Sensor readings
