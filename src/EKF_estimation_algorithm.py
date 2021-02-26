import numpy as np
import math
from ParametersInitialization import *
def wraptopi(x):
    xwrap=np.remainder(x, 2*np.pi);
    mask = np.abs(xwrap)>np.pi;
    xwrap[mask] -= 2*np.pi * np.sign(xwrap[mask]);
    return xwrap

def EKF_Lidar(sParameter,fxhat,fyhat,fthetahat,fv,fw,vLidar,mP,fTs):
    
    #from numpy.lib.function_base import unwrap
    #from numpy.lib.shape_base import expand_dims
    
    global ROBOT_WHEEL_BASE
    temp_fxhat = fxhat;
    temp_fyhat = fyhat;
    temp_fthetahat = fthetahat;
    temp_mP = mP;
    
    temp_vy = vLidar;
    
    temp_mF = np.array([[1,0,-fTs*math.sin(temp_fthetahat)],[0,1,fTs*math.cos(temp_fthetahat)],[0,0,1]]);
    temp_mF = np.array([[fTs*math.cos(temp_fthetahat),0],[fTs*math.sin(temp_fthetahat),0],[0,fTs]]);
    
    #temp_vx =np.transpose(np.array([[,temp_fyhat,]));emp_vx = np.array([[temp_fxhat],[temp_fyhat],[temp_fthetahat]]);
    temp_vx = np.array([[temp_fxhat],[temp_fyhat],[temp_fthetahat]]);
    temp_mH = np.eye(3);
    
    temp_mM = np.eye(3);
    
    #kalman process
    
    temp_mK = temp_mP * np.transpose(temp_mH) * (np.linalg.inv((temp_mH * temp_mP * np.transpose(temp_mH)) + (temp_mM * sParameter.sLidar.mR * np.transpose(temp_mM)) ));
    temp_vz = np.array([[temp_fxhat],[temp_fyhat],[temp_fthetahat]]);

    fresidual = temp_vy - temp_vz;
    #############
    
    #fresidual[2,0] = wraptopi(fresidual[2,0]);
    ###############
    temp_vx += np.matmul(temp_mK,fresidual);
    temp_mP = (np.eye(3)- np.matmul(np.matmul(temp_mK,temp_mH),temp_mP));
    
    fnewxhat = temp_vx[0];
    fnewyhat = temp_vx[1];
    fnewthetahat = temp_vx[2];
    #######33
    
    #fnewthetahat = wraptopi(fnewthetahat);
    ##########
    mnewP = temp_mP;
    
    return fnewxhat,fnewyhat,fnewthetahat,mnewP,fresidual

    
                                                       