#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import math

# Robot location
robot_position_ = Point()
robot_yaw_ = 0 # angle in radian

# Goal location

destination_position0_ = Point()
destination_position0_.x = 1.3
destination_position0_.y = -1.7
destination_position0_.z = 0

destination_position1_ = Point()
destination_position1_.x = 2.2
destination_position1_.y = -1.7
destination_position1_.z = 0

destination_position2_ = Point()
destination_position2_.x = 2.8
destination_position2_.y = -1.7
destination_position2_.z = 0

destination_position3_ = Point()
destination_position3_.x = 3.8
destination_position3_.y = -1.7
destination_position3_.z = 0



# parameters
position_state_ = 0 # --------
# 0 need to fix yaw
# 1 go directly
# 2 reached the place
# ------------------
door_count1_ = 0
door_count2_ = 0
door_count3_ = 0

yaw_precision_ = math.pi / 90 # +/- 2 degree allowed (stopping angle)
dist_precision_ = 0.3 # robo stops with this gap (0.3m)
distance_ = 0.5
near_wall_ = False
t_start_ = 0
regions_ = {
    'Front': 0,
    'F-Left': 0,
    'Left': 0,
    'Right': 0,
    'F-Right': 0,
    'Full-Right': 0,
}
area_ = 6 # --------
# 0 maze
# 1 coridor
# 2 count door
# 3 set for net move
# 4 final move
# ------------------
wall_following_direction_ = 'clockwise' # clockwise or anti-clockwise

# publishers ---------->
pub_cmd_vel_ = None

# Utility functions ---------->
def toRad(val):
    return val * math.pi /180 # math.pi rad = 180 degree

def avg(list):
    return sum(list) / len(list)

def normalize_angle(angle):
    # angle comes in radian. if it is above pi then need to normalize
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

# Other functions ---------->
def to_odom_rotation(angle,kp):
    global robot_yaw_,area_
    fixed_angle = 0 # -180 to +180
    if(angle > 359) :
        fixed_angle = 0
    elif (angle > 180) :
        fixed_angle = angle - 360
    else :
        fixed_angle = angle
    zRotation = round((toRad(fixed_angle) - robot_yaw_), 3)
    if(fabs(zRotation) < 0.001):
        return 0
    else:
        return kp * zRotation


def to_odom_rotation2(des_angle_in_radian,kp):
    # 2pi radian = 360 degree
    global robot_yaw_
    err_yaw = normalize_angle(des_angle_in_radian - robot_yaw_) # angle
    return kp * err_yaw
    # fabs() for floating point absolute values
    # abs() for integer absolute values

def print_regions():
    global regions_
    print('Front   : ' + str(regions_['Front']))
    print('F-Left    : ' + str(regions_['F-Left']))
    print('F-Right   : ' + str(regions_['F-Right']))
    print('Left    : ' + str(regions_['Left']))
    print('Right   : ' + str(regions_['Right']))
    print('Full-Right   : ' + str(regions_['Full-Right']))
    print("----------------------")

def change_state(new_state):
    global position_state_
    position_state_ = new_state

def avoid_obstacles(Flength,Rlength):
    global pub_cmd_vel_,distance_,regions_
    command = Twist()
    if(Flength > distance_):
        if(Rlength < (distance_ / 2)):
            command.angular.z = 1.2
            command.linear.x = -0.1
        elif(Rlength > (distance_ * 0.75)):
            command.angular.z = -0.8
            command.linear.x = 0.22
        else:
            command.angular.z = 0.8
            command.linear.x = 0.22
    else:
        command.angular.z = 1.0
        command.linear.x = 0.0
        pub_cmd_vel_.publish(command)
        while(Flength < 0.3 and not rospy.is_shutdown() ):
            pub_cmd_vel_.publish(command)
    pub_cmd_vel_.publish(command)

def done_moving():
    command = Twist()
    command.linear.x = 0
    command.angular.z = 0
    pub_cmd_vel_.publish(command)

def fix_yaw(des_pos):
    global robot_yaw_, pub_cmd_vel_, yaw_precision_
    destination_yaw = math.atan2(des_pos.y - robot_position_.y, des_pos.x - robot_position_.x)
    err_yaw = normalize_angle(destination_yaw - robot_yaw_)
    command = Twist()
    if math.fabs(err_yaw) > yaw_precision_:
        command.angular.z = 0.7 if err_yaw > 0 else -0.7
    pub_cmd_vel_.publish(command)
    if math.fabs(err_yaw) <= yaw_precision_:
        change_state(1)

def go_to_point_switch(req):
    global active_
    active_ = req

def go_straight_ahead(des_pos):
    global robot_yaw_, pub_cmd_vel_, yaw_precision_, position_state_,dist_precision_
    destination_yaw = math.atan2(des_pos.y - robot_position_.y, des_pos.x - robot_position_.x)
    #atan and atan2 gives angle in radian (anti-tangent)
    #atan = gives angle value between -90 and 90
    #atan2 = gives angle value between -180 and 180. this is mostly used
    err_yaw = destination_yaw - robot_yaw_ # angle
    err_pos = math.sqrt(pow(des_pos.y - robot_position_.y, 2) + pow(des_pos.x - robot_position_.x, 2)) # gap
    # move the robo
    if err_pos > dist_precision_:
        command = Twist()
        command.linear.x = 0.6 # m/s
        command.angular.z = 0.2 if err_yaw > 0 else -0.2 # rad/s.
        pub_cmd_vel_.publish(command)
    else:
        change_state(2)
    # keep the robo head to the destination
    if math.fabs(err_yaw) > yaw_precision_:
        change_state(0)

def go_curve(des_pos):
    global robot_yaw_, pub_cmd_vel_, yaw_precision_, position_state_,dist_precision_
    destination_yaw = math.atan2(des_pos.y - robot_position_.y, des_pos.x - robot_position_.x)
    err_yaw = destination_yaw - robot_yaw_
    err_pos = math.sqrt(pow(des_pos.y - robot_position_.y, 2) + pow(des_pos.x - robot_position_.x, 2))
    if err_pos > 0.3:
        command = Twist()
        command.linear.x = 0.6 # m/s
        command.angular.z = 0.2 if err_yaw > 0 else -0.2 # rad/s.
        pub_cmd_vel_.publish(command)
    else:
        done_moving()

# Callback functions ---------->
def laser_clbk(msg):
    global regions_
    # Front 0,1,2,3,4, 355,356,357,358,359 = 10 --> (355-4)
    # F-Left 14:60 = 45 --> (15-60)
    # Left 74:105 = 30 --> (75-105)
    # Right 268:271 = 3 --> (269-271)
    # F-Right 299:345 = 45 --> (300-345)
    # Full-Right 179:360 = 180 --> (180-360)
    regions_ = {
        'Front': min(min(msg.ranges[0:5]), min(msg.ranges[355:])),
        'F-Left': min(msg.ranges[14:60]),
        'Left': min(msg.ranges[74:105]),
        'Right': avg(msg.ranges[268:271]),
        'F-Right': min(msg.ranges[299:345]),
        'Full-Right': min(msg.ranges[179:]),

    }

def odom_clbk(msg):
    global robot_yaw_,robot_position_
    robot_position_ = msg.pose.pose.position
    odom_ori = msg.pose.pose.orientation
    euler = euler_from_quaternion([odom_ori.x, odom_ori.y, odom_ori.z, odom_ori.w])
    robot_yaw_ = euler[2]


# ............................Main Function............................
def main():
    global pub_cmd_vel_, regions_, wall_following_direction_, near_wall_, distance_
    global area_, position_state_,yaw_precision_
    global t_start_
    global door_count1_, door_count2_, door_count3_
    global destination_position1_, destination_position2_, destination_position3_
    rospy.init_node('maze_solver')
    pub_cmd_vel_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_scan = rospy.Subscriber('/scan', LaserScan, laser_clbk)
    sub_odom = rospy.Subscriber('/odom', Odometry, odom_clbk)

    command = Twist()
    command.linear.x = 0.0
    command.angular.z = 0.0
    time.sleep(1)  # wait for node to initialize
    command.linear.x = 0.1
    if(wall_following_direction_ == 'anti-clockwise'):
        command.angular.z = 0.5
        # print("Turning left...")
    else:
        command.angular.z = -0.5
        # print("Turning right...")
    pub_cmd_vel_.publish(command)
    time.sleep(2)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if(area_ == 0):
            if(wall_following_direction_ == 'anti-clockwise'):
                position_state_ == 0
                area_ = 1
            else:
                if position_state_ == 0:
                    # Rotate
                    fix_yaw(destination_position0_)
                elif position_state_ == 1:
                    # Go straight
                    go_straight_ahead(destination_position0_)
                else:
                    # Already reached the position
                    area_ = 1
                    position_state_ = 0
            print(str(door_count1_) + str(door_count2_) + str(door_count3_))
        elif(area_ == 1):
            # Going to a global position
            if position_state_ == 0:
                # Rotate
                fix_yaw(destination_position1_)
            elif position_state_ == 1:
                # Go straight
                go_straight_ahead(destination_position1_)
            else:
                # Already reached the position
                area_ = 2
                position_state_ = 0
            # Track Door count --->
            if(math.isinf(regions_['F-Left']) and math.isinf(regions_['Left']) and math.isinf(regions_['Right']) and math.isinf(regions_['Front'])) and not math.isinf(regions_['F-Right']):
                door_count1_ = door_count1_ + 1
            # Max door count 1
            if(door_count1_ > 1):
                door_count1_ = 1
            print(str(door_count1_) + str(door_count2_) + str(door_count3_))
        elif(area_ == 2):
            if position_state_ == 0:
                fix_yaw(destination_position2_)
            elif position_state_ == 1:
                go_straight_ahead(destination_position2_)
            else:
                area_ = 3
                position_state_ = 0
            if(math.isinf(regions_['F-Left']) and math.isinf(regions_['Left']) and math.isinf(regions_['Right']) and math.isinf(regions_['Front'])) and not math.isinf(regions_['F-Right']):
                door_count2_ = door_count2_ + 1
            if(door_count2_ > 1):
                door_count2_ = 1
            print(str(door_count1_) + str(door_count2_) + str(door_count3_))
        elif(area_ == 3):
            if position_state_ == 0:
                fix_yaw(destination_position3_)
            elif position_state_ == 1:
                go_straight_ahead(destination_position3_)
            else:
                area_ = 4

            if(math.isinf(regions_['F-Left']) and math.isinf(regions_['Left']) and math.isinf(regions_['Right']) and math.isinf(regions_['Front'])) and not math.isinf(regions_['F-Right']):
                door_count3_ = door_count3_ + 1
            if(door_count3_ > 1):
                door_count3_ = 1
            print(str(door_count1_) + str(door_count2_) + str(door_count3_))
        elif(area_ == 4):
            rotation_angle = to_odom_rotation2(0,0.6)
            command.angular.z = rotation_angle
            command.linear.x = 0
            pub_cmd_vel_.publish(command)
            if(math.fabs(rotation_angle) < yaw_precision_):
                area_ = 5
                position_state_ = 0
                done_moving()
        elif(area_ == 5):
            door_final_count = door_count1_ + door_count2_ + door_count3_
            print("door_final_count" + str(door_final_count))
            if(t_start_ == 0 and door_final_count == 1):
                initTime = rospy.Time.now().to_sec()
                t_start_ = initTime
                t_2 = initTime + 6
                t_3 = initTime + 15
                while (t_3 > t_start_):
                    if(t_2 > t_start_ ):
                        command.angular.z = to_odom_rotation2(1.57,0.2)
                    else:
                        command.angular.z = to_odom_rotation2(3.14,0.2)
                    command.linear.x = 0.22
                    pub_cmd_vel_.publish(command)
                    t_start_ = rospy.Time.now().to_sec()
                else:
                    print("done")
                    done_moving()
            elif(t_start_ == 0 and door_final_count == 2 ):
                initTime = rospy.Time.now().to_sec()
                t_start_ = initTime
                t_2 = initTime + 6
                t_3 = initTime + 15
                while (t_3 > t_start_):
                    if(t_2 > t_start_ ):
                        command.angular.z = to_odom_rotation2(4.71,0.2)
                    else:
                        command.angular.z = to_odom_rotation2(3.14,0.2)
                    command.linear.x = 0.22
                    pub_cmd_vel_.publish(command)
                    t_start_ = rospy.Time.now().to_sec()
                else:
                    print("done")
                    done_moving()
            elif(t_start_ == 0 and door_final_count == 3 ):
                initTime = rospy.Time.now().to_sec()
                t_start_ = initTime
                t_2 = initTime + 3
                t_3 = initTime + 9
                t_4 = initTime + 18
                while (t_4 > t_start_):
                    if(t_2 > t_start_ ):
                        command.angular.z = 0
                    elif(t_3 > t_start_ ):
                        command.angular.z = to_odom_rotation2(2,0.3)
                    else:
                        command.angular.z = to_odom_rotation2(5,0.3)
                    command.linear.x = 0.22
                    pub_cmd_vel_.publish(command)
                    t_start_ = rospy.Time.now().to_sec()
                else:
                    print("done")
                    done_moving()
            else:
                print("done done")
                done_moving()
        else:
            while(near_wall_ == False and not rospy.is_shutdown()):
                if(regions_['Front'] > distance_ and regions_['F-Right'] > distance_ and regions_['F-Left'] > distance_):  # Nothing there, go straight
                    command.angular.z = 0.1
                    command.linear.x = 0.22
                elif(regions_['F-Right'] < distance_):
                    wall_following_direction_ = 'anti-clockwise'
                    near_wall_ = True
                    # print("Right wall found")
                    print("aniti")
                elif(regions_['F-Left'] < distance_):
                    wall_following_direction_ = 'clockwise'
                    near_wall_ = True
                    # print("Left wall found")
                    print("clock")
                else:
                    if(wall_following_direction_ == 'anti-clockwise'):
                        command.angular.z = -0.25
                    else:
                        command.angular.z = 0.25
                    command.linear.x = 0.0
                pub_cmd_vel_.publish(command)
            else:
                if(regions_['Front'] > distance_):
                    if(wall_following_direction_ == 'anti-clockwise' and  regions_['F-Right'] < (distance_ / 2)):
                        # print("{:.2f}m - Too close. Go back".format(regions_['F-Right']))
                        command.angular.z = 1.2
                        command.linear.x = -0.1
                    elif(wall_following_direction_ == 'anti-clockwise' and  regions_['F-Right'] > (distance_ * 0.75)):
                        # print("{:.2f}m - Right-Wall-following; turn right.".format(regions_['F-Right']))
                        command.angular.z = -0.8
                        command.linear.x = 0.22
                    elif(wall_following_direction_ == 'clockwise' and regions_['F-Left'] < (distance_ / 2)):
                        # print("{:.2f}m - Too close. Go back".format(regions_['F-Left']))
                        command.angular.z = -1.2
                        command.linear.x = -0.1
                    elif(wall_following_direction_ == 'clockwise' and regions_['F-Left'] > (distance_ * 0.75)):
                        # print("{:.2f}m - Left-Wall-following; turn left.".format(regions_['F-Left']))
                        command.angular.z = 0.8
                        command.linear.x = 0.22
                    else:
                        if(wall_following_direction_ == 'anti-clockwise'):
                            command.angular.z = 0.8
                            # print("{:.2f}m - Right-Wall-following; turn left.".format(regions_['F-Right']))
                        else:
                            command.angular.z = -0.8
                            # print("{:.2f}m - Left-Wall-following; turn right.".format(regions_['F-Left']))
                        command.linear.x = 0.22
                else:
                    if(wall_following_direction_ == 'anti-clockwise'):
                        command.angular.z = 1.0
                        # print("Front obstacle detected. Turning away - Left.")
                    else:
                        command.angular.z = -1.0
                        # print("Front obstacle detected. Turning away - Right.")
                    command.linear.x = 0.0
                    pub_cmd_vel_.publish(command)
                    while(regions_['Front'] < 0.3 and not rospy.is_shutdown() ):
                        pub_cmd_vel_.publish(command)
                        # print("Front obstacle detected. Turning away cont.")
                pub_cmd_vel_.publish(command)
        if(wall_following_direction_ == 'anti-clockwise' and math.isinf(regions_['F-Left']) and math.isinf(regions_['Left']) and math.isinf(regions_['Front'])):
            if(not area_ == 1 and not area_ == 2 and not area_ == 3 and not area_ == 4 and not area_ == 5):
                area_ = 0
                position_state_ = 0
        if(wall_following_direction_ == 'clockwise' and math.isinf(regions_['F-Right']) and math.isinf(regions_['Right']) and math.isinf(regions_['Front'])):
            if(not area_ == 1 and not area_ == 2 and not area_ == 3 and not area_ == 4 and not area_ == 5):
                area_ = 0
                position_state_ = 0
        rate.sleep()
    else:
        print("Rospy is shoutdown")

if __name__ == '__main__':
    main()
