[sys, sys_aug] = F14LongModel();
[K, Kx, Ki] = F14SFLQI();
%========================================================================
% LQG - DYNAMIC OUTPUT FEEDBACK
%========================================================================
%Obtain original aircraft dynamics w/ actuator output too

B_noise = eye(3)
sigma_wG = 3.0; % ft/sec
sysforkalman = ss(sys.A,[sys.B B_noise],sys.C,zeros(2,4))

%Design Kalman Observer
Qn = sigma_wG^2 * eye(3) %n1 input
Rn = diag([0.001, 0.01]); % alpha sensor noisier than gyro

[Kf,L,P] = kalman(sysforkalman, Qn, Rn, [], [1,2], [1]);
L = L(1:3, :) %We now have an observer gain matrix

A_obs = sys.A - L*sys.C; 
B_obs = [sys.B, L];
C_obs = eye(3);

sys_aug_kalman = ss(A_obs, B_obs, C_obs, 0) 

sys_aug_kalman.InputName = {'d_c'; 'alpha'; 'q'} ;
sys_aug_kalman.OutputName = {'alpha_hat';'q_hat';'delta_hat'};

%We create an SFLQI controller for sys - sys_aug takes turbulence into
%account which does not need to be fed back - this will be reconstructed by
%the Kalman state estimator, we import the gains from F14SFLQI.m


%Now we must connect sys_aug (plant w/ turbulence modelling), sys_aug_kalman (state estimator dynamics), LQR gains and incorporate an integrator for tracking

Ksys = ss(K);
Ksys.InputName = {'alpha_hat';'q_hat';'delta_hat';'xi'};
Ksys.OutputName = {'d_c'}

integrator_sys = ss(tf(1,[1,0]));
integrator_sys.InputName = {'e'};
integrator_sys.OutputName = {'xi'};

%Connect sys_aug and sys_aug_kalman

c1 = connect(sys_aug, sys_aug_kalman,{'d_c'; 'n1'},{'alpha_hat'; 'q_hat'; 'delta_hat'});

%Define reference tracking summation block

refsmblk = sumblk('e = qr - q_hat');

%connect this sumblk and integrator to c1

c2 = connect(c1, refsmblk, integrator_sys, {'qr'; 'd_c'; 'n1'}, {'xi'; 'alpha_hat'; 'q_hat' ;'delta_hat'});

%Finally, connect Ksys with the rest and implement a feedback loop

c3 = connect(c2, -Ksys, {'qr';'n1'}, {'qr'; 'q_hat'; 'd_c'; 'delta_hat'});



size(c3.A)
sort(eig(A_i - B_i*K))       % your LQR design poles
sort(eig(sys.A - L*sys.C))   % your Kalman design poles
sort(pole(c3))                % should contain both sets, plus 3 leftover turbulence poles


%========================================================================
% LQG - CLOSED LOOP SIMULATION (step reference + turbulence noise)
%========================================================================
t = (0:0.001:5)';               % time vector
qr_step = ones(size(t));         % step reference on qr (rad/s)
n1_noise = 1000*randn(size(t));       % unit-intensity white noise on n1

u = [qr_step, n1_noise];         % columns must match c3.InputName order: {'qr','n1'}
y = lsim(c3, u, t);              % columns match c3.OutputName order: {'qr','q_hat','d_c','delta_hat'}

figure;
plot(t, y(:,1), '--', t, y(:,2))
legend('q_{ref}', '$\hat{q}$', 'Interpreter','latex')
xlabel('Time (s)'); ylabel('q (rad/s)')
title('LQG - Pitch Rate Tracking (with turbulence)')
grid on

figure;
plot(t, y(:,3), t, y(:,4))
legend('d_c (commanded)', '\delta\_hat (estimated actuator state)')
xlabel('Time (s)'); ylabel('Deflection (rad)')
title('LQG - Elevator Deflection (with turbulence)')
grid on

