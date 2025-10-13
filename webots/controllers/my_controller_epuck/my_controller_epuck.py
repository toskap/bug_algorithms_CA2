from controller import Robot, DistanceSensor, Motor
import math


robot = Robot()

timestep = int(robot.getBasicTimeStep())

# Define robot motors and set initial positions and velocities
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Define max speed
MAX_SPEED = 6.28

# Define robot sensors
gps = robot.getDevice('gps')
gps.enable(timestep)

gyro = robot.getDevice('gyro(1)')
gyro.enable(timestep)

ps_sensors = []
for i in range(8):
    sensor_name = "ps" + str(i)
    ps_sensors.append(robot.getDevice(sensor_name))
    ps_sensors[i].enable(timestep)
    

    
ps_orientations = [1.27, 0.77, 0.0, 5.21, 4.21, 3.1415, 2.37, 1.87]

# Define positions    
target_pos = [0.88, 0.88]  # Target position (flowerpot) remains unchanged
start_angle = math.pi/2  # Robot initial angle
robot_angle = start_angle  # Robot angle begins at its initial angle. This value is updated
path_angle = 0.0  # Angle between initial position and target position

def best_fit_ps_sensor(robot_angle):
    # Get sensor values
    gps_values = gps.getValues()
    gyro_values = gyro.getValues()

    # Calculate relative angle of robot
    distance_x = target_pos[0] - gps_values[0]
    distance_y = target_pos[1] - gps_values[1]
    angle_to_target = math.atan(distance_y/distance_x) % (2*math.pi)
    alpha = abs((robot_angle - angle_to_target)) % (2*math.pi)

    # Find position sensors' relative angles
    ps_rel_angles = []
    for i in range(8):
        ang = (robot_angle - math.pi/2 + ps_orientations[i] - angle_to_target) % (2*math.pi) 
        ps_rel_angles.append(abs(ang))
        # print(str(i) + ": " + str(abs(ang)))
    
    # Check that the maxmimum angle (closest to 2pi) is not closer to the target angle than the minimum angle (closest to 0)     
    # best_sensor is assigned to the sensor with the smallest angular distance from the target.
    # This value is overwritten if the maximum angle is in fact closer than the minimum angle.
    min_ps_rel_angles = min(ps_rel_angles)
    max_ps_rel_angles = max(ps_rel_angles)
    max_ps_rel_angles_2 = abs(max_ps_rel_angles - 2*math.pi)
        
    best_sensor = ps_rel_angles.index(min_ps_rel_angles)
    
    if max_ps_rel_angles_2 < min_ps_rel_angles:
        best_sensor = ps_rel_angles.index(max_ps_rel_angles)
    
    return best_sensor
       
       
def orient_robot(must_orient, robot_angle):
    # Get sensor values
    gps_values = gps.getValues()
    gyro_values = gyro.getValues()

    # Initial set up
    tolerance = 0.01
    
    if initial_setup == True:
        distance_x = target_pos[0] - gps_values[0]
        distance_y = target_pos[1] - gps_values[1]
        angle_to_target = start_angle - math.atan(distance_y/distance_x)
        angle = (robot_angle - angle_to_target) % (2*math.pi)

        left_speed  = 0.03 * MAX_SPEED  # left speed makes you turn right
        right_speed = -0.03 * MAX_SPEED  # right speed makes you turn left
        
        robot_angle += (gyro_values[2]*timestep/1000)  # /1000 for converting timestep from ms to s
        
        if angle <= (0 + tolerance) and angle >= (0 - tolerance):
            must_orient = False
            
            left_speed  = 0.0 
            right_speed = 0.0
            
            path_angle = robot_angle

    return left_speed, right_speed, must_orient, robot_angle
 

# bug0 algorithm
def bug0_algorithm(robot_angle):
    # Get sensor values
    gps_values = gps.getValues()
    gyro_values = gyro.getValues()
    ps_values = []
    for i in range(8):
        ps_values.append(ps_sensors[i].getValue())
        
    # Detect obstacle
    obst_threshold = 75.0
    right_obstacle = ps_values[0] > obst_threshold or ps_values[1] > obst_threshold or ps_values[2] > obst_threshold
    left_obstacle = ps_values[5] > obst_threshold or ps_values[6] > obst_threshold or ps_values[7] > obst_threshold
    obstacle = False
    for i in range(8):
        if ps_values[i] > obst_threshold:
            obstacle = True
        
    # If no obstacles, continue forward
    left_speed = 0.8 * MAX_SPEED
    right_speed = 0.8 * MAX_SPEED
    
    best_sensor = best_fit_ps_sensor(robot_angle)
    
    for i in range(8):
        print(str(i) + ": " + str(ps_values[i]))
    
    # If obstacles, apply bug0 algorithm
    # if obstacle and ps_values[5] < 120.0 and ps_values[6] <:
    if obstacle:
        left_speed  = 0.03 * MAX_SPEED
        right_speed = -0.03 * MAX_SPEED 
        
        robot_angle += (gyro_values[2]*timestep/1000)
       
    # If a clear path to the target is spotted by the best oriented sensor, start going straight towards the target 

    elif obstacle and ps_values[best_sensor] < 65:
        # print("ELIF")
        print(best_sensor)
        distance_x = target_pos[0] - gps_values[0]
        distance_y = target_pos[1] - gps_values[1]
        angle_to_target = start_angle - math.atan(distance_y/distance_x)
        angle = (robot_angle - angle_to_target) % (2*math.pi)

        left_speed  = 0.03 * MAX_SPEED  # left speed makes you turn right
        right_speed = -0.03 * MAX_SPEED  # right speed makes you turn left
        
        robot_angle += (gyro_values[2]*timestep/1000)  # /1000 for converting timestep from ms to s
        
        tolerance = 0.01
        if angle <= (0 + tolerance) and angle >= (0 - tolerance):
            left_speed = 0.8 * MAX_SPEED
            right_speed = 0.8 * MAX_SPEED


    return left_speed, right_speed, robot_angle               


initial_setup = True

# Run this loop to get the robot to get the robot to move
while robot.step(timestep) != -1:
    gyro_values = gyro.getValues()
    # left_speed, right_speed, robot_angle = bug0_algorithm(robot_angle)
     
    # Initial set up
    if initial_setup == True:
        left_speed, right_speed, initial_setup, robot_angle = orient_robot(initial_setup, robot_angle)
            
    
    # bug0 algorithm
    if initial_setup == False:
        left_speed, right_speed, robot_angle = bug0_algorithm(robot_angle)
     
    # left_speed  = 0.3 * MAX_SPEED
    # right_speed = -0.3 * MAX_SPEED 
    # robot_angle += (gyro_values[2]*timestep/1000)
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    



    pass
