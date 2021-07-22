#!/usr/bin/env python
import math
import rospy
import time
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

Cmd_vel_pub = None

Front = 0
Left = 0
FLeft = 0
FRight = 0
Right = 0

Robot_current_position = Point()
Robot_current_angle = 0

Min_angle_gap = math.pi / 90
Min_distance_gap = 0.3

Destination1 = Point()
Destination1.x = 2.1
Destination1.y = -1.3
Destination1.z = 0

Destination2 = Point()
Destination2.x = 2.9
Destination2.y = -1.3
Destination2.z = 0

Destination3 = Point()
Destination3.x = 4.0
Destination3.y = -1.3
Destination3.z = 0

Door1 = 0
Door2 = 0
Door3 = 0

Outer_state = 0
Section = 0

# Callbacks -->
def Scan_callback(msg):
    global Front, FLeft, Left, Right, FRight
    Front = min(min(msg.ranges[0:5]), min(msg.ranges[355:]))
    FLeft = min(msg.ranges[14:60])
    Left = min(msg.ranges[74:105])
    Right = min(msg.ranges[268:271])
    FRight = min(msg.ranges[299:345])

def Odom_callback(msg):
    global Robot_current_position, Robot_current_angle
    Robot_current_position = msg.pose.pose.position
    odom_ori = msg.pose.pose.orientation
    euler = euler_from_quaternion([odom_ori.x, odom_ori.y, odom_ori.z, odom_ori.w])
    Robot_current_angle = euler[2]

# Utitity Functions -->
def Change_state(new_state):
    global Outer_state
    Outer_state = new_state

def Normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def Fix_angle(des_pos):
    global Robot_current_angle, Cmd_vel_pub, Min_angle_gap
    destination_yaw = math.atan2(des_pos.y - Robot_current_position.y, des_pos.x - Robot_current_position.x)
    err_yaw = Normalize_angle(destination_yaw - Robot_current_angle)
    command = Twist()
    if math.fabs(err_yaw) > Min_angle_gap:
        command.angular.z = 0.3 if err_yaw > 0 else -0.3
    Cmd_vel_pub.publish(command)
    if math.fabs(err_yaw) <= Min_angle_gap:
        Change_state(1)

def Go_directly(des_pos):
    global Robot_current_angle, Cmd_vel_pub, Min_angle_gap, Outer_state, Min_distance_gap
    destination_yaw = math.atan2(des_pos.y - Robot_current_position.y, des_pos.x - Robot_current_position.x)
    err_yaw = destination_yaw - Robot_current_angle
    err_pos = math.sqrt(pow(des_pos.y - Robot_current_position.y, 2) + pow(des_pos.x - Robot_current_position.x, 2))
    if err_pos > Min_distance_gap:
        command = Twist()
        command.linear.x = 0.2 # m/s
        command.angular.z = 0.2 if err_yaw > 0 else -0.2 # rad/s.
        Cmd_vel_pub.publish(command)
    else:
        Change_state(2)
    if math.fabs(err_yaw) > Min_angle_gap:
        Change_state(0)



# Main Function
def MAIN():
    global Cmd_vel_pub
    global Front, Left, Right, FLeft, FRight
    global Section, Outer_state
    global Door1, Door2, Door3

    Cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    scan_sub = rospy.Subscriber('/scan', LaserScan, Scan_callback)
    odom_sub = rospy.Subscriber('/odom', Odometry, Odom_callback)

    rospy.init_node('navigator')
    rate = rospy.Rate(10)

    command = Twist()
    command.linear.x = 0.0
    command.angular.z = 0.0

    time.sleep(1)  # wait for node to initialize
    near_wall = 0  # start with 0, when we get to a wall, change to 1
    distance = 0.5
    command.angular.z = -0.5
    command.linear.x = 0.1
    Cmd_vel_pub.publish(command)
    time.sleep(2)

    while not rospy.is_shutdown():
        if(Section == 1):
            if(Outer_state == 0):
                Fix_angle(Destination1)
            elif(Outer_state == 1):
                Go_directly(Destination1)
            else:
                Section = 2
                Outer_state = 0
            if(math.isinf(Left) and math.isinf(Front)) and math.isinf(Right):
                Door1 = Door1 + 1
            if(Door1 > 1):
                Door1 = 1
            print(str(Door1) + str(Door2) + str(Door3))
        elif(Section == 2):
            if(Outer_state == 0):
                Fix_angle(Destination2)
            elif(Outer_state == 1):
                Go_directly(Destination2)
            else:
                Section = 3
                Outer_state = 0
            if(math.isinf(Left) and math.isinf(Front)) and math.isinf(Right):
                Door2 = Door2 + 1
            if(Door2 > 1):
                Door2 = 1
            print(str(Door1) + str(Door2) + str(Door3))

        elif(Section == 3):
            if(Outer_state == 0):
                Fix_angle(Destination3)
            elif(Outer_state == 1):
                Go_directly(Destination3)
            else:
                command.angular.z = 0
                command.linear.x = 0
                Cmd_vel_pub.publish(command)
                Section = 4

            if(math.isinf(Left) and math.isinf(Front)) and math.isinf(Right):
                Door3 = Door3 + 1
            if(Door3 > 1):
                Door3 = 1
            print(str(Door1) + str(Door2) + str(Door3))
        elif(Section == 4):
            final_door_count = Door1 + Door2 + Door3
            if(final_door_count == 1):
                command.angular.z = 0
                command.linear.x = 0.22
                Cmd_vel_pub.publish(command)
            elif(final_door_count == 2):
                command.angular.z = -0.2
                command.linear.x = 0.22
                Cmd_vel_pub.publish(command)
            else:
                command.angular.z = 0.2
                command.linear.x = 0.22
                Cmd_vel_pub.publish(command)

        else:
            #Searching for a wall
            while(near_wall == 0 and not rospy.is_shutdown()):
                if(Front > distance and FRight > distance and FLeft > distance):  # Nothing there, go straight
                    command.angular.z = -0.1
                    command.linear.x = 0.22
                elif(FLeft < distance):
                    near_wall = 1
                else:
                    command.angular.z = -0.25
                    command.linear.x = 0.0

                Cmd_vel_pub.publish(command)
            #Already found a wall
            else:
                if(Front > distance):
                    if(FRight < (distance / 2)):
                        command.angular.z = +1.2
                        command.linear.x = -0.1
                    elif(FRight > (distance * 0.75)):
                        command.angular.z = -0.8
                        command.linear.x = 0.22
                    else:
                        command.angular.z = +0.8
                        command.linear.x = 0.22
                else:
                    command.angular.z = +1.0
                    command.linear.x = 0.0
                    Cmd_vel_pub.publish(command)
                    while(Front < 0.3 and not rospy.is_shutdown()):
                        Cmd_vel_pub.publish(command)
                Cmd_vel_pub.publish(command)
        if(math.isinf(Front) and math.isinf(FLeft) and not math.isinf(Right)):
            if(not Section == 2 and not Section == 3 and not Section == 4):
                Section = 1
        rate.sleep()

if __name__ == '__main__':
    MAIN()
