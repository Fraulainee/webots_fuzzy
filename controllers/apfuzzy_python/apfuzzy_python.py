from controller import Robot
import numpy as np
from skfuzzy import control as ctrl
from skfuzzy import membership as mf

MAX_SPEED = 47.6  
K_ATTRACTIVE = 1.0  
GOAL_X = -0.5
GOAL_Y = -1
WHEEL_RADIUS = 0.021  
AXLE_LENGTH = 0.105  

# Create fuzzy control system
def create_fuzzy_system():
    # Input variables for potential field forces
    attractive_force = ctrl.Antecedent(np.arange(0, 2, 0.01), 'attractive_force')
    repulsive_force = ctrl.Antecedent(np.arange(0, 2, 0.01), 'repulsive_force')
    angle_diff = ctrl.Antecedent(np.arange(-180, 181, 1), 'angle_diff')
    
    # Output variables
    left_speed = ctrl.Consequent(np.arange(-MAX_SPEED, MAX_SPEED + 1, 0.1), 'left_speed')
    right_speed = ctrl.Consequent(np.arange(-MAX_SPEED, MAX_SPEED + 1, 0.1), 'right_speed')
    
    # Membership functions for attractive force (goal-seeking)
    attractive_force['weak'] = mf.trimf(attractive_force.universe, [0, 0, 0.5])
    attractive_force['medium'] = mf.trimf(attractive_force.universe, [0.3, 0.7, 1.1])
    attractive_force['strong'] = mf.trimf(attractive_force.universe, [0.9, 2, 2])
    
    # Membership functions for repulsive force (obstacle avoidance)
    repulsive_force['very_weak'] = mf.trimf(repulsive_force.universe, [0, 0, 0.3])
    repulsive_force['weak'] = mf.trimf(repulsive_force.universe, [0.2, 0.5, 0.7])
    repulsive_force['medium'] = mf.trimf(repulsive_force.universe, [0.5, 1, 1.5])
    repulsive_force['strong'] = mf.trimf(repulsive_force.universe, [1.2, 2, 2])
    
    
    # Membership functions for angle difference
    # Tighter ranges for more precise turning
    angle_diff['sharp_left'] = mf.trimf(angle_diff.universe, [-180, -90, -45])
    angle_diff['front'] = mf.trimf(angle_diff.universe, [-15, 0, 15])
    angle_diff['left'] = mf.trimf(angle_diff.universe, [-60, -15, 0])
    angle_diff['right'] = mf.trimf(angle_diff.universe, [0, 15, 60])
    angle_diff['sharp_right'] = mf.trimf(angle_diff.universe, [45, 90, 180])
    
    # Membership functions for wheel speeds
    # Adjusted for smoother turning behavior
    left_speed['reverse'] = mf.trimf(left_speed.universe, [-MAX_SPEED, -MAX_SPEED, -MAX_SPEED/3])
    left_speed['slow'] = mf.trimf(left_speed.universe, [-MAX_SPEED/2.5, 0, MAX_SPEED/2.5])
    left_speed['fast'] = mf.trimf(left_speed.universe, [MAX_SPEED/3, MAX_SPEED, MAX_SPEED])
    
    right_speed['reverse'] = mf.trimf(right_speed.universe, [-MAX_SPEED, -MAX_SPEED, -MAX_SPEED/3])
    right_speed['slow'] = mf.trimf(right_speed.universe, [-MAX_SPEED/2.5, 0, MAX_SPEED/2.5])
    right_speed['fast'] = mf.trimf(right_speed.universe, [MAX_SPEED/3, MAX_SPEED, MAX_SPEED])
    
    # Fuzzy rules combining potential field forces
    rules = [
        # First priority: Turn to face goal when path is clear
        ctrl.Rule(repulsive_force['weak'] & angle_diff['left'],
                 (left_speed['slow'], right_speed['fast'])),
        ctrl.Rule(repulsive_force['weak'] & angle_diff['right'],
                 (left_speed['fast'], right_speed['slow'])),
        
        # Move forward when facing goal
        ctrl.Rule(repulsive_force['weak'] & angle_diff['front'] & attractive_force['strong'],
                 (left_speed['fast'], right_speed['fast'])),
        ctrl.Rule(repulsive_force['weak'] & angle_diff['front'] & attractive_force['medium'],
                 (left_speed['fast'], right_speed['fast'])),
        
        # Obstacle avoidance (highest priority)
        ctrl.Rule(repulsive_force['strong'] & angle_diff['front'],
                 (left_speed['reverse'], right_speed['reverse'])),
        ctrl.Rule(repulsive_force['strong'] & angle_diff['left'],
                 (left_speed['fast'], right_speed['reverse'])),
        ctrl.Rule(repulsive_force['strong'] & angle_diff['right'],
                 (left_speed['reverse'], right_speed['fast'])),
        
        # Medium distance obstacle handling - improved turning behavior
        ctrl.Rule(repulsive_force['medium'] & angle_diff['front'],
                 (left_speed['fast'], right_speed['reverse'])),  # Sharp right turn when obstacle in front
        ctrl.Rule(repulsive_force['medium'] & angle_diff['left'],
                 (left_speed['fast'], right_speed['slow'])),  # Gentle right turn
        ctrl.Rule(repulsive_force['medium'] & angle_diff['right'],
                 (left_speed['slow'], right_speed['fast'])),  # Gentle left turn
        
        # Final approach (very close to goal)
        ctrl.Rule(attractive_force['weak'] & angle_diff['front'],
                 (left_speed['slow'], right_speed['slow'])),
        ctrl.Rule(attractive_force['weak'] & angle_diff['left'],
                 (left_speed['slow'], right_speed['fast'])),
        ctrl.Rule(attractive_force['weak'] & angle_diff['right'],
                 (left_speed['fast'], right_speed['slow'])),

        ctrl.Rule(repulsive_force['strong'] & angle_diff['sharp_left'],
                 (left_speed['reverse'], right_speed['fast'])),
        ctrl.Rule(repulsive_force['strong'] & angle_diff['sharp_right'],
                 (left_speed['fast'], right_speed['reverse'])),

        # Gradual slowdown as obstacles approach
        ctrl.Rule(repulsive_force['medium'] & angle_diff['front'],
                 (left_speed['slow'], right_speed['slow'])),

        # Emergency avoidance with immediate turn
        ctrl.Rule(repulsive_force['strong'] & angle_diff['front'],
                 (left_speed['reverse'], right_speed['reverse'])),
    ]
    
    # Create control system
    system = ctrl.ControlSystem(rules)
    return ctrl.ControlSystemSimulation(system)


robot = Robot()


timestep = int(robot.getBasicTimeStep())

# Initialize infrared sensors
ir_sensors = [
    robot.getDevice('front left infrared sensor'),     # 0
    robot.getDevice('front infrared sensor'),          # 1
    robot.getDevice('front right infrared sensor'),    # 2
    robot.getDevice('right infrared sensor'),          # 3
    robot.getDevice('rear right infrared sensor'),     # 4
    robot.getDevice('rear infrared sensor'),           # 5
    robot.getDevice('rear left infrared sensor'),      # 6
    robot.getDevice('left infrared sensor'),           # 7
    # Ground sensors not used for obstacle avoidance
    # robot.getDevice('ground left infrared sensor'),
    # robot.getDevice('ground right infrared sensor'),
    # robot.getDevice('ground front left infrared sensor'),
    # robot.getDevice('ground front right infrared sensor')
]

# Enable all infrared sensors
for sensor in ir_sensors:
    sensor.enable(timestep)

# Get and configure motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Encoders for tracking robot position
left_encoder = robot.getDevice('left wheel sensor')
right_encoder = robot.getDevice('right wheel sensor')  # FIXED: Now using the correct right wheel sensor
left_encoder.enable(timestep)
right_encoder.enable(timestep)

# Robot's initial position and state
x_r, y_r, theta = 0.0, 0.0, 0.0
last_left_pos, last_right_pos = 0.0, 0.0
last_time = robot.getTime()

# Navigation parameters
GOAL_THRESHOLD = 0.05  # meters, distance to consider goal reached
TURN_THRESHOLD = 5.0   # degrees, angle to consider aligned with goal
MAX_ATTRACTIVE_FORCE = 2.0  # cap on attractive force
FORWARD_SPEED_FACTOR = 0.6  # 60% of max speed for forward motion
TURN_SPEED_FACTOR = 0.4  # 40% of max speed for turning

# Obstacle avoidance parameters
SENSOR_THRESHOLD = 100  # IR reading threshold to detect obstacles (lowered for earlier detection)
DANGER_THRESHOLD = 500  # IR reading for immediate avoidance action (lowered)
STOP_THRESHOLD = 1000   # IR reading to stop and reconsider path (lowered)
SAFE_DISTANCE = 50      # IR reading considered safe for navigation

# Time tracking for odometry and status printing
last_time = robot.getTime()
last_print_time = 0
print_interval = 1.0  # Print status every second

# Create fuzzy control system
fuzzy_system = create_fuzzy_system()

# Debug flags
debug_odometry = True  # Set to True to print detailed odometry information
debug_fuzzy = False    # Set to True to print fuzzy system inputs/outputs

# For detecting if robot is stuck
last_positions = []
stuck_detection_interval = 5  # Check every 5 seconds
last_stuck_check_time = 0
position_change_threshold = 0.05  # Minimum movement expected in 5 seconds

# Main loop:
while robot.step(timestep) != -1:
    
    ir_values = [sensor.getValue() for sensor in ir_sensors]
    
    front_left_value = ir_values[0]   
    front_value = ir_values[1]        
    front_right_value = ir_values[2]  
    
    # Side and rear sensors for wider obstacle detection
    left_value = ir_values[7]         
    right_value = ir_values[3]        
    rear_left_value = ir_values[6]    
    rear_right_value = ir_values[4]   
    rear_value = ir_values[5]         

    # Read encoder values
    left_pos = left_encoder.getValue()
    right_pos = right_encoder.getValue()

    # Get current time and compute time step
    current_time = robot.getTime()
    dt = current_time - last_time
    if dt <= 0:  
        dt = 0.001
        
    # Compute wheel velocities (in rad/s)
    v_left = (left_pos - last_left_pos) / dt
    v_right = (right_pos - last_right_pos) / dt
    
    # Convert to linear velocities (m/s)
    v_left_linear = v_left * WHEEL_RADIUS
    v_right_linear = v_right * WHEEL_RADIUS
    
    # Compute displacement and rotation
    d_center = (v_left_linear + v_right_linear) * dt / 2.0
    d_theta = (v_right_linear - v_left_linear) * dt / AXLE_LENGTH
    
    # Update robot position with careful bounds checking
    if abs(d_theta) < np.pi/4:  # Reasonable rotation in one timestep
        theta = (theta + d_theta) % (2 * np.pi)  # Keep theta in [0, 2π]
    
    if abs(d_center) < 0.1:  # Reasonable movement in one timestep (10cm)
        avg_theta = theta - d_theta/2  # Use average heading during movement
        x_r += d_center * np.cos(avg_theta)
        y_r += d_center * np.sin(avg_theta)
    
    if debug_odometry and current_time - last_print_time >= print_interval:
        print(f"Odometry Debug:")
        print(f"  Encoder values: left={left_pos:.6f}, right={right_pos:.6f}")
        print(f"  Linear velocities: left={v_left_linear:.6f}m/s, right={v_right_linear:.6f}m/s")
        print(f"  d_center: {d_center:.6f}m, d_theta: {np.degrees(d_theta):.2f}°")
        print(f"  Position update: dx={d_center * np.cos(avg_theta):.6f}, dy={d_center * np.sin(avg_theta):.6f}")
    
    last_left_pos = left_pos
    last_right_pos = right_pos
    last_time = current_time

    # Check if robot is stuck
    current_time = robot.getTime()
    if current_time - last_stuck_check_time >= stuck_detection_interval:
        # Store position for stuck detection
        current_pos = (x_r, y_r)
        last_positions.append(current_pos)
        
        # Keep only the last 2 positions
        if len(last_positions) > 2:
            last_positions.pop(0)
        
        # Check if we have enough positions to detect if stuck
        if len(last_positions) >= 2:
            dx = last_positions[-1][0] - last_positions[0][0]
            dy = last_positions[-1][1] - last_positions[0][1]
            distance_moved = np.sqrt(dx**2 + dy**2)
            
            if distance_moved < position_change_threshold:
                print(f"WARNING: Robot might be stuck! Distance moved in last {stuck_detection_interval} seconds: {distance_moved:.4f}m")
                print("Executing unstuck maneuver...")
                
                # Execute unstuck maneuver: random turn + short backward movement
                # First back up
                left_motor.setVelocity(-MAX_SPEED * 0.5)
                right_motor.setVelocity(-MAX_SPEED * 0.5)
                # Wait a moment (letting simulation step forward)
                for _ in range(10):
                    if robot.step(timestep) == -1:
                        break
                
                # Then turn randomly
                import random
                if random.random() > 0.5:
                    left_motor.setVelocity(-MAX_SPEED * 0.4)
                    right_motor.setVelocity(MAX_SPEED * 0.4)
                else:
                    left_motor.setVelocity(MAX_SPEED * 0.4)
                    right_motor.setVelocity(-MAX_SPEED * 0.4)
                
                # Wait a moment (letting simulation step forward)
                for _ in range(20):
                    if robot.step(timestep) == -1:
                        break
        
        last_stuck_check_time = current_time

    # Calculate potential field forces
    # Attractive force towards goal
    delta_x = GOAL_X - x_r
    delta_y = GOAL_Y - y_r
    distance_to_goal = np.sqrt(delta_x**2 + delta_y**2)
    
    # Calculate angle to goal in global frame (-π to π)
    angle_to_goal = np.arctan2(delta_y, delta_x)
    
    # Convert current heading to -π to π range for consistent comparison
    robot_heading = theta
    if robot_heading > np.pi:
        robot_heading -= 2*np.pi
    
    # Calculate angle difference (how much robot needs to turn)
    angle_diff_rad = angle_to_goal - robot_heading
    
    # Normalize to -π to π for shortest turn
    if angle_diff_rad > np.pi:
        angle_diff_rad -= 2*np.pi
    elif angle_diff_rad < -np.pi:
        angle_diff_rad += 2*np.pi
    
    # Convert to degrees for fuzzy system
    angle_diff_deg = np.degrees(angle_diff_rad)
    
    # Debug angle calculations
    if current_time - last_print_time >= print_interval:
        print(f"Angle Debug:")
        print(f"  Robot heading: {np.degrees(robot_heading):.2f}° (raw: {np.degrees(theta):.2f}°)")
        print(f"  Angle to goal: {np.degrees(angle_to_goal):.2f}°")
        print(f"  Angle difference: {angle_diff_deg:.2f}°")
        print(f"  Turn direction: {'LEFT' if angle_diff_deg > 0 else 'RIGHT'}")
    
    # Stop when close to goal
    if distance_to_goal < GOAL_THRESHOLD:
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        print("Goal reached! Stopping...")
        break
    
    # Calculate attractive force (stronger when aligned with goal)
    alignment_factor = np.cos(angle_diff_rad)  # 1 when aligned, 0 when perpendicular
    attractive_magnitude = K_ATTRACTIVE * distance_to_goal
    
    # Convert sensor readings to normalized repulsive force (0-2 range for fuzzy system)
    # Front sensors have highest priority
    front_obstacle_value = max(front_left_value, front_value, front_right_value)
    side_obstacle_value = max(left_value, right_value)
    
    # Higher value means stronger repulsion force
    # Exponential scaling for more aggressive response to close obstacles
    if front_obstacle_value > DANGER_THRESHOLD:
        # Exponential scaling for front obstacles
        scale = (front_obstacle_value / DANGER_THRESHOLD) ** 2
        repulsive_force_norm = min(2.0, scale * 2.0)
    elif side_obstacle_value > DANGER_THRESHOLD:
        # Exponential scaling for side obstacles
        scale = (side_obstacle_value / DANGER_THRESHOLD) ** 2
        repulsive_force_norm = min(1.5, scale * 1.5)
    elif front_obstacle_value > SENSOR_THRESHOLD:
        # Linear scaling for moderate distances
        scale = (front_obstacle_value - SENSOR_THRESHOLD) / (DANGER_THRESHOLD - SENSOR_THRESHOLD)
        repulsive_force_norm = min(1.0, scale * 1.0)
    elif side_obstacle_value > SENSOR_THRESHOLD:
        # Linear scaling for moderate distances
        scale = (side_obstacle_value - SENSOR_THRESHOLD) / (DANGER_THRESHOLD - SENSOR_THRESHOLD)
        repulsive_force_norm = min(0.5, scale * 0.5)
    else:
        repulsive_force_norm = 0.0
    
    # Normalize attractive force to [0, 2] range for fuzzy system
    attractive_force_norm = min(2.0, attractive_magnitude)
    
    # Emergency stop and turn if obstacle is too close
    if front_obstacle_value > STOP_THRESHOLD:
        # First back up
        left_motor.setVelocity(-MAX_SPEED * 0.7)
        right_motor.setVelocity(-MAX_SPEED * 0.7)
        
        # Print emergency avoidance status
        print("Emergency obstacle avoidance activated!")
        print(f"Front obstacle readings: L={front_left_value:.0f}, C={front_value:.0f}, R={front_right_value:.0f}")
        
        # Continue to next iteration to allow backing up
        continue
    
    # Check if we need to turn away from obstacles
    elif front_obstacle_value > DANGER_THRESHOLD:
        # Determine which side has more clearance
        if front_left_value > front_right_value:
            # Obstacle more on left, turn right sharply
            left_motor.setVelocity(MAX_SPEED * 0.8)
            right_motor.setVelocity(-MAX_SPEED * 0.8)
        else:
            # Obstacle more on right, turn left sharply
            left_motor.setVelocity(-MAX_SPEED * 0.8)
            right_motor.setVelocity(MAX_SPEED * 0.8)
        
        # Continue turning until clear of obstacle
        continue
        
        # Print emergency avoidance status
        print("Emergency obstacle avoidance activated!")
        print(f"Front obstacle readings: L={front_left_value:.0f}, C={front_value:.0f}, R={front_right_value:.0f}")
    else:
        # SIMPLIFIED CONTROL: If angle_diff is large, just turn efficiently
        # This is to solve the infinite rotation issue
        if abs(angle_diff_deg) > 90:  # Major direction change needed
            print(f"Major direction change needed: {angle_diff_deg:.1f}°")
            
            # Calculate efficient turn speed based on how far we need to turn
            turn_factor = min(1.0, abs(angle_diff_deg) / 180.0)
            turn_speed = MAX_SPEED * turn_factor * 0.5  # Scale with angle difference
            
            if angle_diff_deg > 0:  # Need to turn left
                left_motor.setVelocity(-turn_speed)
                right_motor.setVelocity(turn_speed)
                print(f"Turning LEFT with speed: {turn_speed:.2f}")
            else:  # Need to turn right
                left_motor.setVelocity(turn_speed)
                right_motor.setVelocity(-turn_speed)
                print(f"Turning RIGHT with speed: {turn_speed:.2f}")
                
        elif abs(angle_diff_deg) > TURN_THRESHOLD:  # Minor directional adjustment
            # Need to turn to align with goal
            turn_speed = MAX_SPEED * TURN_SPEED_FACTOR
            
            if angle_diff_deg > 0:  # Turn left
                left_motor.setVelocity(-turn_speed)
                right_motor.setVelocity(turn_speed)
            else:  # Turn right
                left_motor.setVelocity(turn_speed)
                right_motor.setVelocity(-turn_speed)
                
        else:
            # We're aligned with goal and no obstacles - just move forward
            if repulsive_force_norm < 0.5:  # No significant obstacles
                forward_speed = MAX_SPEED * FORWARD_SPEED_FACTOR
                left_motor.setVelocity(forward_speed)
                right_motor.setVelocity(forward_speed)
            else:
                # Use fuzzy logic for obstacle avoidance
                try:
                    # Input values to fuzzy system
                    fuzzy_system.input['attractive_force'] = attractive_force_norm
                    fuzzy_system.input['repulsive_force'] = repulsive_force_norm
                    fuzzy_system.input['angle_diff'] = angle_diff_deg
                    
                    # Compute fuzzy control output
                    fuzzy_system.compute()
                    left_speed = fuzzy_system.output['left_speed'] 
                    right_speed = fuzzy_system.output['right_speed']
                    
                    if debug_fuzzy:
                        print("Fuzzy System Inputs/Outputs:")
                        print(f"  Inputs: attractive={attractive_force_norm:.2f}, repulsive={repulsive_force_norm:.2f}, angle_diff={angle_diff_deg:.1f}°")
                        print(f"  Outputs: left_speed={left_speed:.2f}, right_speed={right_speed:.2f}")
                    
                    # Ensure speeds are within limits
                    left_speed = max(min(left_speed, MAX_SPEED), -MAX_SPEED)
                    right_speed = max(min(right_speed, MAX_SPEED), -MAX_SPEED)
                    
                    # Set motor speeds from fuzzy logic output
                    left_motor.setVelocity(left_speed)
                    right_motor.setVelocity(right_speed)
                    
                except Exception as e:
                    print(f"Warning: Fuzzy system failed: {e}, using fallback control")
                    
                    # Simple reactive obstacle avoidance
                    if front_obstacle_value > SENSOR_THRESHOLD:
                        # Turn away from obstacle
                        if front_left_value > front_right_value:
                            left_motor.setVelocity(MAX_SPEED * 0.5)
                            right_motor.setVelocity(-MAX_SPEED * 0.3)
                        else:
                            left_motor.setVelocity(-MAX_SPEED * 0.3)
                            right_motor.setVelocity(MAX_SPEED * 0.5)
                    else:
                        # Direct steering toward goal
                        forward_speed = MAX_SPEED * FORWARD_SPEED_FACTOR
                        left_motor.setVelocity(forward_speed)
                        right_motor.setVelocity(forward_speed)
    

    if current_time - last_print_time >= print_interval:
        print("\n" + "="*50)
        print("Navigation Status:")
        print(f"Current Position: ({x_r:.3f}, {y_r:.3f})")
        print(f"Goal Position: ({GOAL_X:.3f}, {GOAL_Y:.3f})")
        print(f"Distance to Goal: {distance_to_goal:.3f}m")
        print("\nOrientation:")
        print(f"Robot Heading: {np.degrees(theta):.1f}°")
        print(f"Angle to Goal: {np.degrees(angle_to_goal):.1f}°")
        print(f"Angle Difference: {angle_diff_deg:.1f}°")
        print("\nForces & Motion:")
        print(f"Attractive Force: {attractive_force_norm:.2f}")
        print(f"Repulsive Force: {repulsive_force_norm:.2f}")
        print(f"Motor Speeds: Left={left_motor.getVelocity():.2f}, Right={right_motor.getVelocity():.2f}")
        print("\nSensor Readings:")
        print(f"Front: L={front_left_value:.0f}, C={front_value:.0f}, R={front_right_value:.0f}")
        print(f"Sides: L={left_value:.0f}, R={right_value:.0f}")
        print(f"Rear: L={rear_left_value:.0f}, C={rear_value:.0f}, R={rear_right_value:.0f}")
        print("="*50)
        last_print_time = current_time