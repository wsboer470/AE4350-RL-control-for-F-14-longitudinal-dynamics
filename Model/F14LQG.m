[sys, sys_aug] = F14LongModel();

%========================================================================
% LQG - DYNAMIC OUTPUT FEEDBACK
%========================================================================
%Obtain original aircraft dynamics w/ actuator output too

B_noise = eye(3)

sysforkalman = ss(sys.A,[sys.B B_noise],sys.C,zeros(2,4))

%Design Kalman Observer
Qn = sigma_wG^2 * eye(3)    %n1 input
Rn = diag([0.001, 0.01]);  % alpha sensor noisier than gyro

[L,P,E] = kalman(sysforkalman, Qn, Rn, [], [1,2], [1]);
L = L(1:3, :)
L.InputName = {'d_c'; 'alpha'; 'q'} ;
L.OutputName = {'alpha_hat';'q_hat';'delta_hat'};

sys_aug_kalman = minreal(connect(sys_aug,L, {'d_c';'n1'}, {'alpha_hat';'q_hat';'delta_hat'}));

%Construct closed loop from SFLQI gain derived earlier

A_lqg_aug = [sys_aug_kalman.A,           zeros(9,1);
             -[0 1 0 0 0 0 0 0 0],       0         ];
B_lqg_aug = [sys_aug_kalman.B;
             zeros(1,2)];
Br_lqg = [zeros(9,1); 1];  % reference enters integrator

% Outputs: [alpha_hat, q_hat, delta_hat] + xi
C_lqg_aug = [sys_aug_kalman.C, zeros(3,1);
             zeros(1,9),       1         ];
D_lqg_aug = zeros(4,2);

sys_lqg_aug = ss(A_lqg_aug, [B_lqg_aug, Br_lqg], C_lqg_aug, zeros(4,3));

% Step 2: close loop with K gains
% u = -[Kx, Ki] * [alpha_hat, q_hat, delta_hat, xi]
A_cl_lqg = A_lqg_aug - B_lqg_aug(:,1)*[Kx, Ki]*C_lqg_aug;
B_cl_lqg = Br_lqg;
C_cl_lqg = C_lqg_aug(1:3,:);  % outputs: alpha_hat, q_hat, delta_hat
D_cl_lqg = zeros(3,1);

% sys_lqg_cl = ss(A_cl_lqg, B_cl_lqg, C_cl_lqg, D_cl_lqg);
% pole(sys_lqg_cl)

A_pure_lqg = sys_aug_kalman.A - sys_aug_kalman.B(:,1)*Kx*sys_aug_kalman.C;
pole(ss(A_pure_lqg))

size(A_pure_lqg)
A_pure_lqg

eig(A_pure_lqg)

% Sum = sumblk('q_err = q_ref - q_hat');
% 
% integ = ss(tf(1,[1,0]));
% integ.InputName = {'q_err'};
% integ.OutputName = {'xi'};
% 
% K_block = ss([Kx, Ki]);
% K_block.InputName = {'alpha_hat'; 'q_hat'; 'delta_hat'; 'xi'};
% K_block.OutputName = {'u_lqg'};
% 
% NegSum = sumblk('d_c = -u_lqg');
% 
% sys_lqg_cl = connect(sys_aug_kalman, Sum, integ, K_block, NegSum, {'q_ref'; 'n1'}, {'alpha_hat'; 'q_hat'; 'delta_hat'})
% pole(sys_lqg_cl)
% figure;
% step(sys_lqg_cl(:,1), 5)
% 
% 
% pole(sys_aug_kalman)
% pole(L)
% 
% pole(sys_aug_kalman)
% eig(sys_aug_kalman.A)