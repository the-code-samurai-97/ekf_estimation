import numpy as np
import math
from ParametersInitialization import *

def EKF_Lidar(sParameter,fxhat,fyhat,fthetahat,fv,fw,vLidar,mP,fTs):
    def wrapToPi(x):
        temp_angle = math.fmod(x,2*np.pi)
        if temp_angle > np.pi:
            temp_angle = temp_angle - 2*np.pi
        if temp_angle < -np.pi:
            temp_angle = temp_angle + 2*np.pi
        return temp_angle
    
    
    #from numpy.lib.function_base import unwrap
    #from numpy.lib.shape_base import expand_dims
    
    global ROBOT_WHEEL_BASE
    temp_fxhat = fxhat;
    temp_fyhat = fyhat;
    temp_fthetahat = fthetahat;
    temp_mP = mP;
    
    temp_vy = vLidar;
    
    temp_mF = np.array([[1,0,-fTs*fv*math.sin(temp_fthetahat)],[0,1,fTs*fv*math.cos(temp_fthetahat)],[0,0,1]]);
    temp_mL = np.array([[fTs*math.cos(temp_fthetahat),0],[fTs*math.sin(temp_fthetahat),0],[0,fTs]]);
    temp_mP = np.dot(np.dot(temp_mF,temp_mP),np.transpose(temp_mF))+np.dot(np.dot(temp_mL,sParameter.sProcess.mQ),np.transpose(temp_mL))
    
    #### time update
    temp_fxhat = temp_fxhat + fTs * fv * math.cos(temp_fthetahat);
    temp_fyhat = temp_fyhat + fTs * fv * math.sin(temp_fthetahat);
    temp_fthetahat = temp_fthetahat + fTs * fw;
    temp_fthetahat = wrapToPi(temp_fthetahat);
    
    #temp_vx =np.transpose(np.array([[,temp_fyhat,]));emp_vx = np.array([[temp_fxhat],[temp_fyhat],[temp_fthetahat]]);
    temp_vx = np.array([[temp_fxhat],[temp_fyhat],[temp_fthetahat]]);
    temp_mH = np.eye(3);
    
    temp_mM = np.eye(3);
    
    #kalman process
    
    temp_mK =np.dot(np.dot(temp_mP,np.transpose(temp_mH)),(np.linalg.inv((np.dot(np.dot(temp_mH,temp_mP),np.transpose(temp_mH))) + (np.dot(np.dot(temp_mM,sParameter.sLidar.mR),np.transpose(temp_mM))) )))
    temp_vz = np.array([[temp_fxhat],[temp_fyhat],[temp_fthetahat]]);

    fresidual = temp_vy - temp_vz;
    #############
    
    fresidual[2,0] = wrapToPi(fresidual[2,0]);
    ###############
    temp_vx = temp_vx + np.dot(temp_mK,fresidual);
    temp_mP = np.dot((np.eye(3)- np.dot(temp_mK,temp_mH)),temp_mP)
    
    fnewxhat = float(temp_vx[0]);
    fnewyhat = float(temp_vx[1]);
    fnewthetahat = float(temp_vx[2]);
    
    #######33
    
    fnewthetahat = wrapToPi(fnewthetahat);
    
    ##########
    mnewP = temp_mP;
    
    return fnewxhat,fnewyhat,fnewthetahat,mnewP,fresidual

    
                                                       