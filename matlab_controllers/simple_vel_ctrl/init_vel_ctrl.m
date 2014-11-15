clear all

%simulation
basic_time_step=0.002;



%gravity
g=9.81;


%mass of the QC
m0=1.3;


%ATTITUDE CONTROLLER

% KDR = 5; 
% KPR = 5;
% 
% KDP = 5; 
% KPP = 5;
% 
% KDY = 5;
% KPY = 5;


KDR = 11/5; 
KPR = 30/5;

KDP = 11/5; 
KPP = 30/5;

KDY = 11;
KPY = 30;


% 
% KDR = 0.001; 
% KPR = 0.02;
% 
% KDP = 0.001; 
% KPP = 0.02;
% 
% KDY = 11;
% KPY = 30;


% KDR = 20; 
% KPR = 100;
% 
% KDP = 20; 
% KPP = 100;
% 
% KDY = 20;
% KPY = 100;



%POSITION CONTROLLER
kd=[2 0 0;
    0 2 0
    0 0 7];

% kp=[1 0 0;
%     0 1 0;
%     0 0 10];

kp=[0 0 0;
    0 0 0;
    0 0 0];


ki=0*[.1 0 0;
    0 .1 0;
    0 0 2];


%max_roll and max_pitch cmds
max_roll=sin(15*pi/180);
max_pitch=sin(15*pi/180);

