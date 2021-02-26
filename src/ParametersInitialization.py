import numpy
class sParameter:
     #Experiment trick
    sEffectiveSpeed = 1.5000;
    class sProcess:
        #robot parameters
        mQ = numpy.array([[0.5,0],[0,0.01]]);
    sProcess = sProcess()
    class sGPS:
        #GPS parameter
        mR = numpy.array([[0.1,0,0],[0,0.1,0],[0,0,0.02]]);
        # CUSUM parameters
        fCov = numpy.array([[ 0.0834, 0, 0],[0, 0.0211, 0],[0, 0, 0.0019 ]]);#%[ 0.3, 0, 0; 0, 0.15, 0; 0, 0, 0.02 ];
        fomega = 24;
        ftao = 10;
    sGPS = sGPS()
    class sGPSHeading:
        #GPS parameter
        mR = 0.02;
    sGPSHeading = sGPSHeading()
    class sLidar:
        #LIDAR parameter
        mR = numpy.array([[0.04,0,0],[0,0.04,0],[0,0,0.01]]);
       # CUSUM parameters
        fCov = numpy.array([[0.0152, 0, 0],[0, 0.0056, 0],[0, 0, 2.0171/10000]]);#%[ 0.05, 0, 0; 0, 0.03, 0; 0, 0, 0.003];
        fomega = 35;
        ftao = 10;
    sLidar = sLidar()
    class sLidarHeading:
        #LIDAR parameter
        mR = 0.01;
    sLidarHeading = sLidarHeading()
    class sFusion:
        # CUSUM parameters
        fCov = numpy.array([[0.4693, 0, 0, 0, 0, 0],[0, 0.1655, 0, 0, 0, 0],[0, 0, 0.0017, 0, 0, 0],[0 ,0 ,0, 0.1193, 0, 0],[0, 0, 0, 0, 0.0145, 0],[0, 0, 0, 0, 0, 4.0364e-04]]); #%[ 1.5, 0, 0, 0, 0, 0; 0, 0.7, 0, 0, 0, 0; 0, 0, 0.04, 0, 0, 0; 0 ,0 ,0, 0.5, 0, 0; 0, 0, 0, 0, 0.06, 0; 0, 0, 0, 0, 0, 0.005];
        fomega = 34;
        ftao = 10;
    sFusion = sFusion()
    class sFusionHeading:
        # CUSUM parameters
        fCov = 0.1;
        fomega = 1.2;
        ftao = 10;
    sFusionHeading = sFusionHeading()
    class sGPSAttack:
        # Attack parameters
        iInitialT = 500; ##%500 for both attack;%480 for gps stealthy;%500 for gps fdi2;%350 for gps fdi1; %450 for gps dos;
        fOffset = 5;
        fGradient = 2;
    sGPSAttack =  sGPSAttack()
    class sLidarAttack:
        # Attack parameters
        iInitialT = 530; ##%530 for both attack;%550 for replay attack;
        fDelayT = 5;
    sLidarAttack = sLidarAttack()
    class sEstimator:
        # Estimator parameters
        ik_x = 5;
        igamma_x = 7;
        iepsilonx_bar = 3;
        ik_y = 5;
        igamma_y = 7;
        iepsilony_bar = 3;
        ik_theta = 5;
        igamma_theta = 5;
        iepsilontheta_bar = 0.5;
    sEstimator = sEstimator()
sParameter = sParameter()

global TS
global ROBOT_WHEEL_BASE
global ATTACK_TYPE # 0: No attack 1: GPS 2: Lidar
global GPS_ATTACK_TYPE # 1: fixed point 2: jumping 3: gradually drift  
  
    #####################
TS = 1/10;
ROBOT_WHEEL_BASE = 1.53;
ATTACK_TYPE = 0;
GPS_ATTACK_TYPE = 0;
###########
