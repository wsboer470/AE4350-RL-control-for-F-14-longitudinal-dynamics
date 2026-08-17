function [sys, sys_aug] = F14LongModel()
clear; clc;

%========================================================================
% DEFINE GRUMMANN F-14 MODEL VARIABLES
%========================================================================
tau_a = 0.05; %sec
sigma_wG = 3.0; %ft/sec
a = 2.5348; %sec
b = 64.13; %ft
VT0 = 690.4; %ft/sec
sigma_alpha = 5.236e-3; %rad
omega_alpha = 10.0; %rad/sec
Zd = -63.9979; %ft/(rad-sec^2)

Md = -6.8847; %1/(rad-sec^2)
U0 = 689.4; %ft/sec
Zw = -0.6485; %1/sec
Mq = -0.6571; %1/sec
Mw = -5.92e-3; %1/(ft-sec)

%========================================================================
% DEFINE GRUMMANN F-14 MODEL STATE SPACE W/ ACTUATOR DYNAMICS
%========================================================================
%Matrices
A = [Zw U0;
    Mw Mq];
    
B = [Zd ;
    Md] ;

C = [1/U0 0;
    0 1];

D = [0;
    0];

sys_sp = ss(A,B,C,D)
eig(sys_sp)

%Add actuator dynamics

ActuatorSys = ss(-1/tau_a,1/tau_a,1,0)

A_new = [A,B;
    zeros(1,2), ActuatorSys.A];

B_new = [zeros(2,1);
    ActuatorSys.B];

C_new = [C, zeros(2,1)];

D_new = D;

sys = ss(A_new,B_new,C_new,D_new)

sys.InputName = {'d_c'};
sys.OutputName = {'alpha';'q'};

%========================================================================
% ADD DRYDEN TURBULENCE MODEL DYNAMICS
%========================================================================
%Turbulence Transfer Functions
s = tf('s');

wG_n1 = sigma_wG/sqrt(a^3)*((sqrt(3)*a*s+1)/(s+1/a)^2);
qG_n1 = pi/(4*b)*(s/(s+pi*VT0/(4*b)))*wG_n1;

TurbSys = ss([-Zw*wG_n1;-Mw*wG_n1-Mq*qG_n1]);
TurbSys = minreal(TurbSys)

A_aug = [A_new,[TurbSys.C; zeros(1,3)];
         zeros(3,3),TurbSys.A];

B_aug = [B_new,[TurbSys.D; zeros(1,1)];
         zeros(3,1),TurbSys.B];

C_aug = [C_new,zeros(2,3)];

D_aug = [D_new TurbSys.D];

sys_aug = ss(A_aug, B_aug, C_aug, D_aug);
sys_aug.InputName = {'d_c'; 'n1'};
sys_aug.OutputName = {'alpha'; 'q'};


% Check poles
disp('sys_aug Open Loop Poles:')
pole(sys_aug)

% % Step response on q channel
% figure;
% %opt = stepDataOptions('StepAmplitude', 1);
% step(sys_aug(2,:), 10)
% title('sys_aug - Pitch Rate Step Response')
% ylabel('q (rad/s)')
% grid on
% 
% stepinfo(sys_aug(2,:), 1)
% 
% end
