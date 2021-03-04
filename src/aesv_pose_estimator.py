#!/usr/bin/env python
import rospy, numpy as np,math,matplotlib.pyplot as plt
from ParametersInitialization import *
from EKF_estimation_algorithm import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from ackermann_msgs.msg import AckermannDriveStamped,AckermannDrive
from tf.transformations import quaternion_from_euler, euler_from_quaternion
#  globaling variables
global counter,time,time1,init,new_msg1, new_msg2,EKF,fpositionx,fpositiony,fnewxhat1,fnewyhat1,yaw,fnewthetahat1,CUMSUM_S_tk,omega_tun_param,Threshold_param,fnewxhat,fnewyhat,fnewthetahat;
time = 0;counter = 0;
Threshold_param = 1*np.ones([3,3]);
omega_tun_param = 5* np.ones([3,3]);
mp = np.array([[1,0,0],[0,1,0],[0,0,0.3]]);
CUMSUM_S_tk=np.zeros([3,3]);
init = 0;
time1=list();fpositionx = list();fpositiony = list();yaw=list();fnewxhat1 = [];fnewyhat1 = [];fnewthetahat1=[]
# Plotting Variables
def plot_x(x1,x2,x3,t):
    global counter,time1,time
    if counter % 10 == 0:
        
        plt.subplot(1,3,1)
        plt.plot(t,x1,'b')
        plt.title('residuals blue1')
        plt.subplot(1,3,2)
        plt.title('residuals red2')
        plt.plot(t,x2,'r')
        plt.subplot(1,3,3)
        plt.title('residuals green3')
        plt.plot(t,x3,'g')
        #plt.title('Estimated pose')
        plt.draw()
        plt.pause(0.000000001)
    counter += 1
# declaring the data types
new_msg1 = Odometry()
new_msg2 = AckermannDriveStamped()
EKF = Odometry()
EKF_estimated_Pose = Odometry()

# subscriber call back function
def robot_to_map_callback(msg):
    global new_msg1
    new_msg1 = msg
def cmd_vel_callback(msg):
    global new_msg2
    #print(msg)
    new_msg2 = msg

#initializing node and subscribing and publishing
rospy.init_node('AESV_pose_estimator')
sub1 = rospy.Subscriber('global_odom',Odometry, robot_to_map_callback)
sub2 = rospy.Subscriber('cmd_vel',AckermannDriveStamped, cmd_vel_callback)
EKF_estimated_Pose = rospy.Publisher('EKF_estimated_Pose',Odometry)

rate = rospy.Rate(100)
while not rospy.is_shutdown():
    
    #print(new_msg2.drive.steering_angle_velocity)
    ##estimation
    [roll,pitch,yaw_z] = euler_from_quaternion([new_msg1.pose.pose.orientation.x,new_msg1.pose.pose.orientation.y,new_msg1.pose.pose.orientation.z,new_msg1.pose.pose.orientation.w]);
    vLidar = np.array([[new_msg1.pose.pose.position.x,new_msg1.pose.pose.position.y,yaw_z]]);
    if init==0:
        time = rospy.get_time()
        fnewxhat = new_msg1.pose.pose.orientation.x
        fnewyhat = new_msg1.pose.pose.orientation.y
        fnewthetahat = yaw_z
    
    init = 1
    [fnewxhat,fnewyhat,fnewthetahat,mnewP,fresidual] = EKF_Lidar(sParameter,fnewxhat,fnewyhat,fnewthetahat,new_msg2.drive.speed,new_msg2.drive.steering_angle_velocity,np.transpose(vLidar),mp,1/30);
    
    # print("thetahat",fnewthetahat)
    # ### Detection
    print("residual",fresidual[0][0])
    # covMatrix = np.cov(fresidual,bias=True)
    # first_mul=np.dot(np.transpose(fresidual),np.linalg.inv(covMatrix))
    # G_tk_white_residual=np.dot(first_mul,fresidual)
    # CUMSUM_S_tk = np.ndarray.max((CUMSUM_S_tk+G_tk_white_residual)-omega_tun_param)
    # if CUMSUM_S_tk >= np.ndarray.max(Threshold_param):
    #     print("Alarm")
    # else:
    #     print("no Alarm")
    # print("G_tk",G_tk_white_residual)
    
    ##publishing as EKF_estimation
    
    
    # EKF.header.frame_id = new_msg1.header.frame_id
    # EKF.child_frame_id = new_msg1.child_frame_id
    # EKF.header.stamp = new_msg1.header.stamp
    # EKF.pose.pose.position.x = fnewxhat;
    # EKF.pose.pose.position.y = fnewyhat;
    # EKF.pose.pose.position.z = 0.0;    
    # EKF_quaternion = quaternion_from_euler(0, 0, fnewthetahat)
    # EKF.pose.pose.orientation.x = EKF_quaternion[0]
    # EKF.pose.pose.orientation.y = EKF_quaternion[1]
    # EKF.pose.pose.orientation.z = EKF_quaternion[2]
    # EKF.pose.pose.orientation.w = EKF_quaternion[3]
    # EKF_estimated_Pose.publish(EKF)
    ##plotting data
    
    # fpositionx.append(new_msg1.pose.pose.position.x);
    # fpositiony.append(new_msg1.pose.pose.position.y);
    # yaw.append(yaw_z)
    if np.max(fresidual)==0:
        continue
    else:
        
        time1.append(rospy.get_time() - time)
        fnewxhat1.append(fresidual[0][0])
        fnewyhat1.append(fresidual[1][0])
        fnewthetahat1.append(fresidual[2][0])
        plot_x(fnewxhat1,fnewyhat1,fnewthetahat1,time1)
    plt.ion()
    rate.sleep()
rospy.spin()