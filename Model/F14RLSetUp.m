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

deMax    = 0.4363;   % ~25 deg in radians
maxsteps = 500;
%Ensure to scale Dryden noise input by Ts to account for discretisation

%Setting up the RL environment
obsInfo = rlNumericSpec([5 1]);
actInfo = rlNumericSpec([1 1], 'LowerLimit', -deMax, 'UpperLimit', deMax);

resetFcn = @() resetF14(Ad, maxSteps);
stepFcn  = @(action, LoggedSignals) stepF14(action, LoggedSignals, Ad, Bd, Cd, Dd, Ts, deMax);


env = rlFunctionEnv(obsInfo, actInfo, stepFcn, resetFcn);


obs = reset(env);
for k = 1:5
    testAction = 0.05;
    [obs, reward, isDone, info] = step(env, testAction);
    fprintf('step %d: alpha=%.4f q=%.4f reward=%.4f done=%d\n', k, obs(1), obs(2), reward, isDone);
end
%Set up the Actor's model - multi-layered NN to learn dynamic control
%behaviours state in, elevator command out
actorNet = [
    featureInputLayer(5, 'Name', 'obs')
    fullyConnectedLayer(128)
    reluLayer
    fullyConnectedLayer(128)
    reluLayer
    fullyConnectedLayer(1)
    tanhLayer('Name', 'act')
];

actor = rlContinuousGaussianActor(actorNet, obsInfo, actInfo);

% Critic network: (state, action) in. Q-value out
statePath  = featureInputLayer(5, 'Name', 'obs');
actionPath = featureInputLayer(1, 'Name', 'act');
commonPath = [ concatenationLayer(1, 2, 'Name', 'concat')
    fullyConnectedLayer(128)
    reluLayer
    fullyConnectedLayer(1)];

%Set up the reward function that will be used
 
critic = rlQValueFunction(criticNet, obsInfo, actInfo, 'ObservationInputNames', 'obs', 'ActionInputNames', 'act');



%Set up Critics' Model - Tie reward function to value function that will be
%present in simulations

criticNet = layerGraph(statePath);
criticNet = addLayers(criticNet, actionPath);
criticNet = addLayers(criticNet, commonPath);
criticNet = connectLayers(criticNet, 'obs', 'concat/in1');
criticNet = connectLayers(criticNet, 'act', 'concat/in2');

%Set up training loop






%compile training results


%evaluate controller



%Write csv file (or equivalent) to report training runs and hyperparameter
%tuning

