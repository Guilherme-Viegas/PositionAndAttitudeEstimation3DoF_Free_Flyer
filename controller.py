
# Uses the same dynamics as the 6Dof (just a reduced state) and different model to compute Force and Torque because there are only 3 blades
# Receive user input for the objective position and attitude (later on will be received from a subscriber to pose of aruco pkg)
# Computes the necessary rotations per second on each of ACROBAT's blades to reach that position and attitude
# Based on 2 papers:
#   - "A multi-objective optimization approach to the design of a free-flyer space robot for in-orbit manufacturing and assembly" by Vale, Rocha, Leite and Ventura
#   - "Towards an autonomous free-flying robot fleet for intra-vehicular trasnportation of loads in unmanned space stations" by Ventura, Roque and Ekal
#!python2
import numpy as np # Import Numpy library
import math
import sys

def get_rotation_matrix_from_euler_angles(euler_angles):
    x = euler_angles[0]
    y = euler_angles[1]
    z = euler_angles[2]
    return np.array([
            [np.cos(y)*np.cos(x) ,  -np.cos(z)*np.sin(x)+np.sin(z)*np.cos(x)*np.sin(y) , np.sin(z)*np.sin(x)+np.cos(z)*np.sin(y)*np.cos(x)],    #TODO(): Double check if it's correct
            [np.cos(y)*np.sin(x) , np.cos(z)*np.cos(x)+np.sin(z)*np.sin(y)*np.sin(x) , -np.sin(z)*np.cos(x)+np.cos(z)*np.sin(y)*np.sin(x)],
            [-np.sin(y) , np.sin(z)*np.cos(y) , np.cos(z)*np.cos(y)]
        ])

# Calculates rotation matrix to euler angles
def get_euler_anles_from_rotation_matrix(R) :

    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])

FREE_FLYER_MASS = 0.340 # Kg, Random testing value... Still have to search for the exact mass of the ACROBAT
FREE_FLYER_MOMENT_OF_INERTIA = np.array((0.1348056, 0.1902704, 0.1435024)) # Kg.m^2 ... Still have to search for exact moment of inertia vector of the ACROBAT
FREE_FLYER_BLADE_MAX_RPS = 568 # ACROBAT propellers rotations per second

# The aruco library estimates the position of the tag relative to the camera, where the tag's coordinate system has z pointing towards us and Y up, X right
# Meaning we want the robot to be 30cm in front of the tag == 30cm in the Z axis of tag
DESIRED_POSITION = np.array((0, 0, 0.2)) # To be 10cm in front of the AR tag
DESIRED_LINEAR_VELOCITY = np.array((0, 0, 0)) # We want the robot to be stopped in the end
DESIRED_ATTITUDE = np.array((0, 0, 0)) # To be aligned with the AR tag TODO: This might have be confirmed... don't known if this will make the front of the robot face backwards
DESIRED_ROTATION_MATRIX = get_rotation_matrix_from_euler_angles(DESIRED_ATTITUDE)
DESIRED_ANGULAR_VELOCITY = np.array((0, 0, 0)) # We want the robot to be stopped in the end
DESIRED_ANGULAR_ACCELERATION = np.array((0, 0, 0))

# Controller Gains TODO: Need to be calibrated
K_x = 4 # Controller Proportional Gain (Translational part)
K_v = 0.1 # Controller Derivative Gain (Translational part)
K_r = 2 # Controller Porportional Gains (Rotational part)
K_w = 0.1 # Controller Derivative Gain (Rotational part)

# Check with the real ACROBAT what are the blade indexes that exist
# a1 = np.array((-0.02219657522, 0.01859006027, 0.01279597757, -0.01859006027, -0.03499255279, 0.01859006027)).T
# a2 = np.array((0.00023789747, 0.01859006027, 0.01279597757, 0.0191789862, 0.0004043464, -0.00083633887)).T
# a3 = np.array((0.01079543949, 0.9998789634, 0.01079611706, 0.9979681394, -0.00933602561, -0.1332524502)).T
# a4 = np.array((0.7653778177, 0.01913843278, 0.9996875163, -0.03321824755, -0.0134563338, 0.02569007069)).T
# a5 = np.array((-0.001067244345, 0.01896885746, -0.05617408802, 0.05638539532, -0.00126732433, -0.00128318081)).T
# a6 = np.array((.9982558165, 0.05796384873, 0.9983184715, -0.01714869459, 0.07147917464, 0.01299743049)).T

#Considering 2 degrees of freedom x, y and rotation z
a1 = np.array((0.2588, -0.9659, 0.8528812)).T
a2 = np.array((-0.9659, 0.2588, -0.85289255)).T
a3 = np.array((0.7071, 0.7071, -0.85290402)).T

A = np.column_stack((a1, a2, a3))
A_inverse = np.linalg.inv(A)

def compute_force_and_torque(current_position, current_attitude):
    # ************* Testing values, will be erased later ************* Should be received from IMU (?)
    current_linear_velocity = np.array((0, 0, 0))
    current_angular_velocity = np.array((0, 0, 0))
    # ****************************************************************
    attitude_rotation_matrix = get_rotation_matrix_from_euler_angles(current_attitude)
    # Translational Part
    error_x = current_position - DESIRED_POSITION
    error_v = current_linear_velocity - DESIRED_LINEAR_VELOCITY # current_velocity has to be somehow received by the ACROBAT sensors (subscribe to topic) 
    acceleration = -K_x * error_x - K_v * error_v # K_x and K_v are the proportionate and derivative gains (constants) and error_x and error_v the position and velocity errors
    force = np.dot( (FREE_FLYER_MASS * attitude_rotation_matrix), acceleration)

    # Rotational Part
    inverse_of_S_w = get_inverse_S_w( (np.dot(DESIRED_ROTATION_MATRIX.T, attitude_rotation_matrix) - np.dot(attitude_rotation_matrix.T, DESIRED_ROTATION_MATRIX)) )
    error_r = ( 1 / (2*np.sqrt(1 + np.trace( np.dot(DESIRED_ROTATION_MATRIX.T, attitude_rotation_matrix ))) )) * inverse_of_S_w
    error_w = current_angular_velocity - np.dot(np.dot( attitude_rotation_matrix.T, DESIRED_ROTATION_MATRIX), DESIRED_ANGULAR_VELOCITY)
    S_w_matrix = get_S_w( np.dot( np.dot(attitude_rotation_matrix.T, DESIRED_ROTATION_MATRIX), DESIRED_ANGULAR_VELOCITY ) )
    torque = -K_r * error_r - K_w * error_w + np.dot(np.dot(np.dot(np.dot(S_w_matrix, FREE_FLYER_MOMENT_OF_INERTIA), attitude_rotation_matrix.T), DESIRED_ROTATION_MATRIX), DESIRED_ANGULAR_VELOCITY) + np.dot(np.dot(np.dot(FREE_FLYER_MOMENT_OF_INERTIA, attitude_rotation_matrix.T), DESIRED_ROTATION_MATRIX), DESIRED_ANGULAR_ACCELERATION)
    force = np.array((force[0], force[2]))
    torque = np.array((torque[1]))

    return force, torque

# Matrix operations that recovers angular velocity vector from a skew-symmetrix matrix (Check paper)
# S(w) = [0, -w_z, w_y; w_z, 0, -w_x; -w_y, w_x, 0]
def get_inverse_S_w(matrix):
    angular_velocity = (matrix[2][1], matrix[0][2], matrix[1][0])
    return np.array(angular_velocity)

def get_S_w(vect):
    s_matrix = np.array([[0, -vect[2], vect[1]], [vect[2], 0, -vect[0]], [-vect[1], vect[0], 0]])
    return s_matrix

# Converts from force and torque to pwm signals to each of the propellers
def compute_pwm_control(force, torque):
    input_vect = force
    input_vect = np.append(input_vect, torque)
    q = np.dot(A_inverse, input_vect)
    rpm = forces_to_rpm(q)
    q = map_rpm_to_pulsewidth(rpm)
    return np.array(q)

# The ACROBAT papers says that F_max = 2 and M_Max = 2
# Pulse = 0 ==> OFF; Pulse = 1000 ==> Safe anti-clockwise
# Pulse = 1500 ==> Centre; Pulse = 2000 ==> Safe clockwise
def map_rpm_to_pulsewidth(rpm_vector):
    rpm_vector = rpm_vector
    difference = 1000.0

    for idx in range(len(rpm_vector)):
        rpm_vector[idx] = 1500 + rpm_vector[idx] * difference / 2.0
    return rpm_vector

def forces_to_rpm(forces_vector):
    rpm_vector = []
    for force in forces_vector:
        if force < 0:
            rpm_vector.append(0.0 - math.sqrt(-force))
        else:
            rpm_vector.append(math.sqrt(force))
    return rpm_vector