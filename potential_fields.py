#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np
import threading

#####################
# BEGIN Global Variable Definitions

robot = [0,0,0]
laser_scan = None
goal = [0,0,0]

# END Global Variable Definitions
#####################+

#####################
# BEGIN ROS Topic Callback Functions [DON'T MESS WITH THIS STUFF]
#####################

def euler_from_quaternion(quaternion):
    x,y,z,w = quaternion

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def robot_callback(data):
    #This function updates the robots position and yaw, based on the ground truth (we don't have localization yet)
    global robot
    [r,p,yaw] = euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
    robot = [data.pose.pose.position.x,data.pose.pose.position.y,yaw]

def laser_callback(data):
    #This function sets the global laser_scan variable to hold the most recent laser scan data
    global laser_scan
    laser_scan = data

#####################
##### END CALLBACK FUNCTIONS   
#####################

##################### 
# BEGIN HELPER FUNCTIONS [USE THESE IN YOUR CODE, BUT YOU SHOULDN'T NEED TO MODIFY]
#####################

def add_forces(a, b):
    #This function adds to force vectors together and returns the result
    assert len(a) == len(b), "Force vectors differ in length"
    c = [a[i] + b[i] for i in range(len(a))]
    return c

def wrap_angle(angle):
    #This function will take any angle and wrap it into the range [-pi, pi]
    while angle >= math.pi:
        angle = angle - 2*math.pi
        
    while angle <= -math.pi:
        angle = angle + 2*math.pi
    return angle

#####################
##### END HELPER FUNCTIONS
#####################

#####################
# BEGIN MODIFIABLE LAB CODE [ALTHOUGH MOST MODIFICATIONS SHOULD BE WHERE SPECIFIED]
#####################

#This function takes in a force [x,y] (in robot coordinates) and returns the drive command (Twist) that should be sent to the robot motors
def drive_from_force(force):
    #####################################################
    #PARAMETERS : MODIFY TO GET ROBOT TO MOVE EFFECTIVELY
    
    # Reduced from 1.0 to make turns less aggressive
    turn_multiplier = 0.5
    
    # Increased threshold to allow more flexible turning
    spin_threshold = math.pi/2
    
    # Increased drive multiplier to maintain forward momentum
    drive_multiplier = 1.5
    
    #END OF PARAMETERS
    #####################################################

    twist = Twist()

    force_angle = math.atan2(force[1], force[0])
    force_mag = math.hypot(force[0], force[1])

    twist.angular.z = turn_multiplier * force_angle

    # Only drive forward if the angle is close to the current direction
    if abs(force_angle) < spin_threshold:
        twist.linear.x = drive_multiplier * force_mag

    return twist

# This function determines and returns the attractive force (force_to_goal) to the goal.  
# This force should be in robot coordinates
def goal_force():
    # This is the robot's actual global location, set in robot_callback
    global robot  # format [x_position, y_position, yaw]
    global goal   # format [goal_x_position, goal_y_position]

    #####################################################
    # PARAMETERS : MODIFY TO GET ROBOT TO MOVE EFFECTIVELY
    strength = 1.5
    # END OF PARAMETERS
    #####################################################

    force_to_goal = [0, 0]

    #########################
    # PART A : BEGIN
    #########################

    target_angle = math.atan2(goal[1] - robot[1], goal[0] - robot[0])
    current_angle = robot[2]
    angle_to_goal = wrap_angle(target_angle - current_angle)
    
    force_to_goal = [strength * math.cos(angle_to_goal), 
                     strength * math.sin(angle_to_goal)]

    #########################
    # PART A : END
    #########################

    return force_to_goal


#This function looks at the current laser reading, then computes and returns the obstacle avoidance force vector (in local robot coordinates)
def obstacle_force():  
    global laser_scan
    
    if laser_scan is None:
        return [0,0]

    force_from_obstacles = [0,0]

    cur_angle = laser_scan.angle_min 

    for i in range(len(laser_scan.ranges)):
        # Use linear magnitude function instead of constant
        strength = get_pf_magnitude_linear(laser_scan.ranges[i])
    
        # Compute obstacle force vector
        obstacle_force_x = -strength * math.cos(cur_angle)
        obstacle_force_y = -strength * math.sin(cur_angle)
        
        force_from_obstacles[0] += obstacle_force_x
        force_from_obstacles[1] += obstacle_force_y

        cur_angle = cur_angle + laser_scan.angle_increment

    return force_from_obstacles

# This function returns the magnitude of repulsive force for the input distance
# using a linear drop-off function
def get_pf_magnitude_linear(distance):
    #####################################################
    #PARAMETERS: MODIFY TO GET THINGS WORKING EFFECTIVELY
        
    #How close to the obstacle do we have to be to begin feeling repulsive force
    distance_threshold = 1.5 

    #The maximum strength of the repulsive force
    max_strength = 1.5

    #END OF PARAMETERS
    #####################################################
    
    #########################
    #  PART C : BEGIN
    #########################

    # PART C CODE HERE: 
    #   1. Compute the magnitude of the force for the given distance and return it
    
    if distance < distance_threshold:
        # Calculate percentage: closer to 0 means closer to max strength
        percentage = 1.0 - (distance / distance_threshold)
        return percentage * max_strength
    
    return 0

    #########################
    #  PART C : END
    #########################

# This function returns the magnitude of repulsive force for the input distance
# using a constant value if the obstacles is closer than a threshold
def get_pf_magnitude_constant(distance):

    #####################################################
    #PARAMETERS: MODIFY TO GET THINGS WORKING EFFECTIVELY
        
    #How close to the obstacle do we have to be to begin feeling repulsive force
    distance_threshold = 1.0 

    #Strength of the repulsive force
    strength = 1.0

    #END OF PARAMETERS
    #####################################################

    if distance < distance_threshold:
        return strength

    return 0

# This is the main loop of the lab code.  It runs continuously, navigating our robot
# (hopefully) towards the goal, without hitting any obstacles
def main():
    rclpy.init()
    node = rclpy.create_node('potential_fields') #Initialize the ros node
    pub = node.create_publisher(Twist, 'cmd_vel', 10) #Create our publisher to send drive commands to the robot
    scan_subscription = node.create_subscription(LaserScan, 'base_scan', laser_callback, 10) #Subscribe to the laser scan topic
    pose_subscription = node.create_subscription(Odometry, 'odom', robot_callback, 10) #Subscribe to the robot pose topic

    global goal
    node.declare_parameter('next_waypoint', '0 0 0')
    waypoint_param = node.get_parameter('next_waypoint').get_parameter_value().string_value
    goal = [float(f) for f in waypoint_param.split()]

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    rate = node.create_rate(10) #10hz

    try:
        node.get_logger().info('Starting potential_fields to goal position: ' + str(goal))
        while rclpy.ok():

            #1. Compute attractive force to goal
            g_force = goal_force()
            
            #2. Compute obstacle avoidance force
            o_force = obstacle_force()

            #3. Get total force by adding together
            total_force = add_forces(g_force, o_force)
            
            #4. Get final drive command from total force
            twist = drive_from_force(total_force) 

            #5. Publish drive command, then sleep 
            pub.publish(twist)
            
            rate.sleep() #sleep until the next time to publish

    except KeyboardInterrupt:
        pass

    thread.join()

if __name__ == '__main__':
    main()
