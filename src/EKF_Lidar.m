% By Yuanzhe Wang for testing Extended Kalman filter based sensor fusion
% Mar. 10, 2020

function [ fnewxhat, fnewyhat, fnewthetahat, mnewP, fresidual ] = EKF_Lidar( sParameter, fxhat, fyhat, fthetahat, fv, fw, vLidar, mP, fTs )

global ROBOT_WHEEL_BASE

temp_fxhat = fxhat;
temp_fyhat = fyhat;
temp_fthetahat = fthetahat;
temp_mP = mP;

temp_vy = vLidar;

temp_mF = [ 1,              0,         -fTs * fv * sin(temp_fthetahat)
            0,              1,          fTs * fv * cos(temp_fthetahat)
            0,                    0,                              1   ];
        
temp_mL = [ fTs * cos(temp_fthetahat),               0
            fTs * sin(temp_fthetahat),               0
            0,                                      fTs ];

temp_mP = temp_mF * temp_mP * temp_mF' + temp_mL * sParameter.sProcess.mQ * temp_mL';

% time update
temp_fxhat = temp_fxhat + fTs * fv * cos(temp_fthetahat);
temp_fyhat = temp_fyhat + fTs * fv * sin(temp_fthetahat);
temp_fthetahat = temp_fthetahat + fTs * fw;
temp_fthetahat = wrapToPi(temp_fthetahat);

temp_vx = [temp_fxhat, temp_fyhat, temp_fthetahat]';
                     
                                  
temp_mH = eye(3);

temp_mM = eye(3);

% Kalman process
temp_mK = temp_mP * temp_mH' * inv( temp_mH * temp_mP * temp_mH' + temp_mM * sParameter.sLidar.mR * temp_mM' );


temp_vz = [ temp_fxhat; temp_fyhat; temp_fthetahat ];

fresidual = temp_vy - temp_vz;
fresidual(3) = wrapToPi(fresidual(3));
               
temp_vx = temp_vx + temp_mK * fresidual;
temp_mP = ( eye(3) - temp_mK * temp_mH ) * temp_mP;

fnewxhat = temp_vx(1);
fnewyhat = temp_vx(2);
fnewthetahat = temp_vx(3);
fnewthetahat = wrapToPi(fnewthetahat);
mnewP = temp_mP;





