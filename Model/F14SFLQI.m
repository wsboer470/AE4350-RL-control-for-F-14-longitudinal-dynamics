function [K, Kx, Ki] = F14SFLQI();
[sys, sys_aug] = F14LongModel();

%We create an SFLQI controller for sys - sys_aug takes turbulence into
%account which does not need to be fed back - this will be reconstructed by
%the Kalman state estimator

%Tracking q without turbulence:
Cyr = sys.C(2,:)

A_i = [sys.A,zeros(3,1); 
    -Cyr,0];
B_i = [sys.B;0];

Q = [1 0 0 0;
    0 100 0 0;
    0 0 1 0;
    0 0 0 100000000];

R = 10000000;

K = lqr(A_i,B_i,Q,R);
Kx = K(1:3)
Ki = K(4)



%Closing the loop

A_cl = [sys.A - sys.B*Kx, -sys.B*Ki;
    -Cyr, 0];
B_cl = [zeros(3,1); 1];
C_cl = [sys.C, zeros(2,1);
    0 0 1 0];
D_cl = [zeros(3,1)];

G_sflqi = ss(A_cl,B_cl,C_cl,D_cl);
G_sflqi.InputName = {'q_{ref}'};
G_sflqi.OutputName = {'alpha'; 'q'; 'delta'};

% Check poles
disp('SFLQI Closed Loop Poles:')
pole(G_sflqi)

% Step response on q channel
figure;
%opt = stepDataOptions('StepAmplitude', 1);
step(G_sflqi(2,:), 2)
title('SFLQI - Pitch Rate Step Response')
ylabel('q (rad/s)')
grid on

stepinfo(G_sflqi(2,:), 1)

figure;
step(G_sflqi(3,:), 2)
title('SFLQI - Elevator Deflection Step Response')
ylabel('\delta (rad)')
grid on

stepinfo(G_sflqi(3,:), 1)