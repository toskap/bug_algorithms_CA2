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
    
ds_right = robot.getDevice('ds_right')
ds_left = robot.getDevice('ds_left')
ds_front = robot.getDevice('ds_front')

ds_right.enable(timestep)
ds_left.enable(timestep)
ds_front.enable(timestep)

        
ps_orientations = [1.27, 0.77, 0.0, 5.21, 4.21, 3.1415, 2.37, 1.87]

pen = robot.getDevice('pen')
pen.write(True)

# Define positions    
target_pos = [0.88, 0.88]  # Target position (flowerpot) remains unchanged
start_angle = math.pi/2  # Robot initial angle
robot_angle = start_angle  # Robot angle begins at its initial angle. This value is updated
path_angle = 0.0  # Angle between initial position and target position

 
# stop if at target
def stop_at_target():
    gps_values = gps.getValues()

    at_target = False
    distance_x = target_pos[0] - gps_values[0]
    distance_y = target_pos[1] - gps_values[1]
    accepted_dist_to_target = 0.1
    
    if abs(distance_x) < accepted_dist_to_target and abs(distance_y) < accepted_dist_to_target:
        at_target = True
        print(distance_x)
        print(distance_y)
        print(target_pos[0])
        print(target_pos[1])
        
    return at_target

# bug0 algorithm
def bug0_algorithm(robot_angle):
    # Get sensor values
    gps_values = gps.getValues()
    gyro_values = gyro.getValues()
    
    ps_values = []
    for i in range(8):
        ps_values.append(ps_sensors[i].getValue())
        
    ds_right_values = ds_right.getValue()
    ds_left_values = ds_left.getValue()
    ds_front_values = ds_front.getValue()
  
    # Detect obstacle
    obst_threshold = 80.0
    right_obstacle = ps_values[0] > obst_threshold or ps_values[1] > obst_threshold or ps_values[2] > obst_threshold
    left_obstacle = ps_values[5] > obst_threshold or ps_values[6] > obst_threshold or ps_values[7] > obst_threshold
    obstacle = False
    for i in range(8):
        if ps_values[i] > obst_threshold:
            obstacle = True
        
    # Tolerance for turning to orient 
    distance_x = target_pos[0] - gps_values[0]
    distance_y = target_pos[1] - gps_values[1]
    angle_to_target = math.atan(distance_y/distance_x)
    angle = (robot_angle - angle_to_target) % (2*math.pi)
    
    tolerance = 0.1
    # If angle is within tolerance, continue straight. Otherwise, rotate
    within_tolerance = angle <= (0 + tolerance) and angle >= (0 - tolerance)
    
 
    # Rotate to move next to obstacle state
    if obstacle and ds_left_values <= 215.0 and ds_front_values > 72.0:
        print("STATE: rotate next to obstacle")
        left_speed  = 0.03 * MAX_SPEED
        right_speed = -0.03 * MAX_SPEED 
        
        # robot_angle += (gyro_values[2]*timestep/1000)
   
    # Move alongside obstacle state
    elif obstacle and ds_front_values < 72.0:
        print("STATE: move along obstacle")
        left_speed = 0.8 * MAX_SPEED
        right_speed = 0.8 * MAX_SPEED
        
    # Orient to target state
    elif within_tolerance == False:
        print("STATE: orient to target")
        left_speed  = 0.03 * MAX_SPEED 
        right_speed = -0.03 * MAX_SPEED  
        # robot_angle += (gyro_values[2]*timestep/1000)  # /1000 for converting timestep from ms to s          
   
    # Move to target state
    else: 
        print("STATE: move to target")
        left_speed = 0.8 * MAX_SPEED
        right_speed = 0.8 * MAX_SPEED

    return left_speed, right_speed, robot_angle               


initial_setup = True

# Run this loop to get the robot to get the robot to move
at_target = False
while robot.step(timestep) != -1 and at_target == False:
    gyro_values = gyro.getValues()

    at_target = stop_at_target()
        
    left_speed, right_speed, robot_angle = bug0_algorithm(robot_angle)

    if at_target == True:
        print("Reached target. Stop robot.")
        left_speed = 0.0
        right_speed = 0.0
        
    robot_angle += (gyro_values[2]*timestep/1000)  # /1000 for converting timestep from ms to s          
    robot_angle = robot_angle % (2*math.pi)
        
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    
    


    pass
