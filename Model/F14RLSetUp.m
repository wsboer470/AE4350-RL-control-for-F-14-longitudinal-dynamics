[sys, sys_aug] = F14LongModel();
%========================================================================
% F14 RL SET-UP
%========================================================================

%Discretising the F14 Longitudinal Model
Ts = 0.01;                         
sys_d = c2d(sys_aug, Ts);
Ad = sys_d.A;
Bd = sys_d.B;
Cd = sys_d.C;
Dd = sys_d.D;



%Ensure to scale Dryden noise input by Ts to account for discretisation

%Setting up the RL environment


%Set up the reward function that will be used


%Set up the Actor's model - multi-layered NN to learn dynamic control
%behaviours


%Set up Critic's Model - Tie reward function to value function that will be
%present in simulations



%Set up training loop






%compile training results


%evaluate controller



%Write csv file (or equivalent) to report training runs and hyperparameter
%tuning

