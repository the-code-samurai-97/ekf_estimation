#!/usr/bin/env python
import rospy, numpy as np,math,matplotlib.pyplot as plt
from ParametersInitialization import *
from EKF_estimation_algorithm import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
#  globaling variables
global new_msg1, new_msg2,EKF,fpositionx,fpositiony,fnewxhat1,fnewyhat1,yaw,fnewthetahat1;
mp = np.array([[1,0,0],[0,1,0],[0,0,0.3]]);
fpositionx = list();fpositiony = list();yaw=list();fnewxhat1 = [];fnewyhat1 = [];fnewthetahat1=[]
# Plotting Variables
def plot_x(fpositionx,fpositiony,fnewxhat1,fnewyhat1):
    global counter
    if counter % 10 == 0:
        plt.subplot(1,2,1)
        plt.plot(fpositionx,fpositiony,'b')
        plt.title('Current Position')
        plt.subplot(1,2,2)
        plt.plot(fnewxhat1,fnewyhat1,'r')
        plt.title('EKF Estimated pose')
        plt.draw()
        plt.pause(0.000000001)

    counter += 1
# declaring the data types
new_msg1 = Odometry()
new_msg2 = Twist()
EKF = Odometry()
EKF_estimated_Pose = Odometry()

# subscriber call back function
def robot_to_map_callback(msg):
    global new_msg1
    new_msg1 = msg
def cmd_vel_callback(msg):
	global new_msg2
	new_msg2 = msg

#initializing node and subscribing and publishing
rospy.init_node('pose_estimator')
sub1 = rospy.Subscriber('robot_to_map',Odometry, robot_to_map_callback)
sub2 = rospy.Subscriber('RosAria/cmd_vel',Twist, cmd_vel_callback)
EKF_estimated_Pose = rospy.Publisher('EKF_estimated_Pose',Odometry)

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    counter = 0
    #estimation
    [roll,pitch,yaw_z] = euler_from_quaternion([new_msg1.pose.pose.orientation.x,new_msg1.pose.pose.orientation.y,new_msg1.pose.pose.orientation.z,new_msg1.pose.pose.orientation.w]);
    vLidar = np.array([[new_msg1.pose.pose.position.x,new_msg1.pose.pose.position.y,yaw_z]]);
    [fnewxhat,fnewyhat,fnewthetahat,mnewP,fresidual] = EKF_Lidar(sParameter,new_msg1.pose.pose.position.x,new_msg1.pose.pose.position.y,yaw_z,new_msg2.linear.x,new_msg2.angular.z,np.transpose(vLidar),mp,0.1);
    #publishing as EKF_estimation
    EKF.pose.pose.position.x = fnewxhat;
    EKF.pose.pose.position.y = fnewyhat;
    EKF.pose.pose.position.z = 0.0;    
    EKF_quaternion = quaternion_from_euler(0, 0, fnewthetahat)
    EKF.pose.pose.orientation.x = EKF_quaternion[0]
    EKF.pose.pose.orientation.y = EKF_quaternion[1]
    EKF.pose.pose.orientation.z = EKF_quaternion[2]
    EKF.pose.pose.orientation.w = EKF_quaternion[3]
    EKF_estimated_Pose.publish(EKF)
    #plotting data
    fpositionx.append(new_msg1.pose.pose.position.x);
    fpositiony.append(new_msg1.pose.pose.position.y);
    yaw.append(yaw_z)
    fnewxhat1.append(fnewxhat)
    fnewyhat1.append(fnewyhat)
    fnewthetahat1.append(fnewthetahat1)
    plot_x(fpositionx,fpositiony,fnewxhat1,fnewyhat1)
    plt.ion()
    rate.sleep()
rospy.spin()