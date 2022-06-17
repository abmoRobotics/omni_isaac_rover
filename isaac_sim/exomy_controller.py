from omni.isaac.core.controllers import BaseController
from omni.isaac.core.utils.types import ArticulationAction
import numpy as np
import math

class CoolController(BaseController):
    FL, FR, CL, CR, RL, RR = range(0, 6)
    def __init__(self):
        super().__init__(name="exomy_controller")
        # An open loop controller that uses a unicycle model
        self.wheel_x = 19.5
        self.wheel_y = 15
        return

    def forward(self, command):
        # command [lin_vel_x, ang_vel]
        # command will have two elements, first element is the forward velocity
        # second element is the angular velocity (yaw only).
        lin_vel_x = command[0]
        ang_vel = command[1]

        steering_angles = [0.0]*6
        motor_speeds = [0.0]*6

        if ang_vel != 0:
            radius = (abs(lin_vel_x)/abs(ang_vel))*100
            # Distance from center og the rover to the top (centimeters):
        y_top = 19.5 # check if it's correct
        # Distance from center of the rover to the side (centimeters):
        x_side = 15 # check if it's correct


        # If the angular velociy is 0, the angles for the wheel are set to 0
        if ang_vel == 0: 
            steering_angles[self.FL] = 0
            steering_angles[self.FR] = 0
            steering_angles[self.CL] = 0
            steering_angles[self.CR] = 0
            steering_angles[self.RL] = 0
            steering_angles[self.RR] = 0
        # If the turning point is within the chassis of the robot, turn on the spot:
        elif radius <= x_side : 
            steering_angles[self.FL] = math.atan2(y_top,x_side)
            steering_angles[self.FR] = -math.atan2(y_top,x_side)
            steering_angles[self.CL] = 0
            steering_angles[self.CR] = 0
            steering_angles[self.RL] = -math.atan2(y_top,x_side)
            steering_angles[self.RR] = math.atan2(y_top,x_side)
        # Steering angles if turning anticlockwise moving forward or clockwise moving backwards
        elif (ang_vel > 0 and np.sign(lin_vel_x) > 0) or (ang_vel < 0 and np.sign(lin_vel_x) < 0):
            steering_angles[self.FL] = -math.atan2(y_top,(radius-x_side))
            steering_angles[self.FR] = -math.atan2(y_top,(radius+x_side))
            steering_angles[self.CL] = 0
            steering_angles[self.CR] = 0
            steering_angles[self.RL] = math.atan2(y_top,(radius-x_side))
            steering_angles[self.RR] = math.atan2(y_top,(radius+x_side))
        # Steering angles if turning clockwise moving forward or anticlockwise moving backwards
        elif (ang_vel < 0 and np.sign(lin_vel_x) > 0) or (ang_vel > 0 and np.sign(lin_vel_x) < 0):
            steering_angles[self.FL] = math.atan2(y_top,(radius+x_side))
            steering_angles[self.FR] = math.atan2(y_top,(radius-x_side))
            steering_angles[self.CL] = 0
            steering_angles[self.CR] = 0
            steering_angles[self.RL] = -math.atan2(y_top,(radius+x_side))
            steering_angles[self.RR] = -math.atan2(y_top,(radius-x_side))

        """
        Motor speeds calculation
        """
        # Speed moving forward/backward = linear velocity 
        if ang_vel == 0: 
            motor_speeds[self.FL] = lin_vel_x
            motor_speeds[self.FR] = lin_vel_x
            motor_speeds[self.CL] = lin_vel_x
            motor_speeds[self.CR] = lin_vel_x
            motor_speeds[self.RL] = lin_vel_x
            motor_speeds[self.RR] = lin_vel_x
        # Speed turning in place (anticlockwise), velocity of corner wheels = angular velocity 
        elif radius <= x_side and ang_vel > 0: 
            frontLeft = math.sqrt((y_top*y_top)+(x_side*x_side))*abs(ang_vel)
            centerLeft = x_side*abs(ang_vel)
            relation = centerLeft/frontLeft # relation between corner wheel and center wheel velocity (center wheels slower)
            motor_speeds[self.FL] = -abs(ang_vel)
            motor_speeds[self.FR] = abs(ang_vel)
            motor_speeds[self.CL] = -abs(ang_vel)*relation
            motor_speeds[self.CR] = abs(ang_vel)*relation
            motor_speeds[self.RL] = -abs(ang_vel)
            motor_speeds[self.RR] = abs(ang_vel)
        # Speed turning in place (clockwise), velocity of corner wheels = angular velocity 
        elif radius <= x_side and ang_vel < 0: 
            frontLeft = math.sqrt((y_top*y_top)+(x_side*x_side))*abs(ang_vel)
            centerLeft = x_side*abs(ang_vel)
            relation = centerLeft/frontLeft # relation between corner wheel and center wheel velocity (center wheels slower)
            motor_speeds[self.FL] = abs(ang_vel)
            motor_speeds[self.FR] = -abs(ang_vel)
            motor_speeds[self.CL] = abs(ang_vel)*relation
            motor_speeds[self.CR] = -abs(ang_vel)*relation
            motor_speeds[self.RL] = abs(ang_vel)
            motor_speeds[self.RR] = -abs(ang_vel)
        # Speed turning anticlockwise moving forward/backward, velocity of frontRight wheel = linear velocity 
        elif ang_vel > 0:
            frontLeft = (math.sqrt((y_top*y_top)+((radius-x_side)*(radius-x_side)))*abs(ang_vel))*np.sign(lin_vel_x)
            frontRight = (math.sqrt((y_top*y_top)+((radius+x_side)*(radius+x_side)))*abs(ang_vel))*np.sign(lin_vel_x)
            frontRelation = frontLeft/frontRight # relation of speed between the front wheels (frontLeft is slower)
            centerLeft = ((radius-x_side)*abs(ang_vel))*np.sign(lin_vel_x)
            centerRight = ((radius+x_side)*abs(ang_vel))*np.sign(lin_vel_x)
            centerRelation = centerLeft/centerRight # relation of speed between the center wheels (centerLeft is slower)
            frontCenterRelation = centerRight/frontRight # relation between center and front wheels (center is slower)
            motor_speeds[self.FL] = lin_vel_x*frontRelation
            motor_speeds[self.FR] = lin_vel_x
            motor_speeds[self.CL] = lin_vel_x*frontCenterRelation*centerRelation
            motor_speeds[self.CR] = lin_vel_x*frontCenterRelation
            motor_speeds[self.RL] = lin_vel_x*frontRelation
            motor_speeds[self.RR] = lin_vel_x
        # Speed turning clockwise moving forward/backward, velocity of frontLeft wheel = linear velocity
        elif ang_vel < 0:
            frontLeft = (math.sqrt((y_top*y_top)+((radius+x_side)*(radius+x_side)))*abs(ang_vel))*np.sign(lin_vel_x)
            frontRight = (math.sqrt((y_top*y_top)+((radius-x_side)*(radius-x_side)))*abs(ang_vel))*np.sign(lin_vel_x)
            frontRelation = frontRight/frontLeft # relation of speed between the front wheels (frontRight is slower)
            centerLeft = ((radius+x_side)*abs(ang_vel))*np.sign(lin_vel_x)
            centerRight = ((radius-x_side)*abs(ang_vel))*np.sign(lin_vel_x)
            centerRelation = centerRight/centerLeft # relation of speed between the center wheels (centerRight is slower)
            frontCenterRelation = centerLeft/frontLeft # relation between center and front wheels (center is slower)
            motor_speeds[self.FL] = lin_vel_x
            motor_speeds[self.FR] = lin_vel_x*frontRelation
            motor_speeds[self.CL] = lin_vel_x*frontCenterRelation
            motor_speeds[self.CR] = lin_vel_x*frontCenterRelation*centerRelation
            motor_speeds[self.RL] = lin_vel_x
            motor_speeds[self.RR] = lin_vel_x*frontRelation

            # Motor speeds are converted to int's
            # motor_speeds[self.FL] = int(motor_speeds[self.FL])
            # motor_speeds[self.FR] = int(motor_speeds[self.FR])
            # motor_speeds[self.CL] = int(motor_speeds[self.CL])
            # motor_speeds[self.CR] = int(motor_speeds[self.CR])
            # motor_speeds[self.RL] = int(motor_speeds[self.RL])
            # motor_speeds[self.RR] = int(motor_speeds[self.RR])

            # Steering angles are first converted to degrees[# and then to int's(Removed by Emil)]
            steering_angles[self.FL] = np.rad2deg(steering_angles[self.FL])
            steering_angles[self.FR] = np.rad2deg(steering_angles[self.FR])
            steering_angles[self.CL] = np.rad2deg(steering_angles[self.CL])
            steering_angles[self.CR] = np.rad2deg(steering_angles[self.CR])
            steering_angles[self.RL] = np.rad2deg(steering_angles[self.RL])
            steering_angles[self.RR] = np.rad2deg(steering_angles[self.RR])
        # joint_velocities = [0.0] * 15
        # joint_velocities[0] = steering_angles[self.FL]
        # joint_velocities[1] = steering_angles[self.FR]
        # joint_velocities[2] =steering_angles[self.CL]
        # joint_velocities[3] =steering_angles[self.CR] 
        # joint_velocities[4] =steering_angles[self.RL] 
        # joint_velocities[5] =steering_angles[self.RR] 
        # joint_velocities[6] =motor_speeds[self.FL] 
        # joint_velocities[7] =motor_speeds[self.FR] 
        # joint_velocities[8] =motor_speeds[self.CL] 
        # joint_velocities[9] =motor_speeds[self.CR]
        # joint_velocities[10] =motor_speeds[self.RL]
        # joint_velocities[11] =motor_speeds[self.RR] 
        # joint_velocities[12] =motor_speeds[self.RR]  
        # joint_velocities[13] =motor_speeds[self.RR]
        # joint_velocities[14] =motor_speeds[self.RR] 
        # joint_velocities = [0.0] * 15
        # joint_velocities[0] = 0#steering_angles[self.FL]
        # joint_velocities[1] =0# steering_angles[self.FR]
        # joint_velocities[2] =0# steering_angles[self.CL]
        # joint_velocities[3] =0# steering_angles[self.CR] POS_CL
        # joint_velocities[4] =0 #steering_angles[self.RL] POS_FL
        # joint_velocities[5] =0# steering_angles[self.RR] POS_RL
        # joint_velocities[6] =0# motor_speeds[self.FL] POS_RR
        # joint_velocities[7] =0# motor_speeds[self.FR] POS_CR
        # joint_velocities[8] =0 #motor_speeds[self.CL] POS_FR
        # joint_velocities[9] =0# motor_speeds[self.CR] VEL_CL
        # joint_velocities[10] =0# motor_speeds[self.RL] VEL_FL
        # joint_velocities[11] =0 #motor_speeds[self.RL] VEL_RL
        # joint_velocities[12] =0# motor_speeds[self.RR] VEL_RR
        # joint_velocities[13] =0 #motor_speeds[self.RR] VEL_CR
        # joint_velocities[14] =0 #motor_speeds[self.FR] VEL_FR

        steering_indices = [4,8,3,7,5,6]
        motor_speeds_indices = [10,14,9,13,11,12]
        steering_angles[0]=-steering_angles[0]
        steering_angles[1]=-steering_angles[1]
        steering_angles[2]=-steering_angles[2]
        steering_angles[3]=-steering_angles[0]
        steering_angles[4]=-steering_angles[4]
        steering_angles[5]=-steering_angles[5]
                # A controller has to return an ArticulationAction
        return ArticulationAction(joint_velocities=motor_speeds, joint_indices=motor_speeds_indices), ArticulationAction(joint_positions=steering_angles, joint_indices=steering_indices)
        #return ArticulationAction(joint_velocities=[100],joint_indices=[14])


