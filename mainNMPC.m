%%Non linear MPC
clear all
nx = 12;    %state variables
ny = 12;     %outputs
nu = 6;     
nlobjecto = nlmpc(nx, ny, nu);
%Sample time
nlobjecto.Ts = 0.001;
%Horizon
nlobjecto.PredictionHorizon = 5;
nlobjecto.ControlHorizon = 5;
%NMPC
nlobjecto.Model.StateFcn = 'droneDynamics';
nlobjecto.Model.OutputFcn = 'OutputFcn';
% Forza esplicitamente che ci sono 12 output variables
nlobjecto.OutputVariables = repmat(struct('Min', -Inf, 'Max', Inf), 12, 1);



% Cost function
nlobjecto.Optimization.CustomCostFcn = 'CostFunction';
%Weights
nlobjecto.Weights.OutputVariables = [10e5 10e5 10e6 10e-4 10e-4 10e2 10e-2 10e-2 10e-2 10e-2 10e-2 10e-2];
nlobjecto.Weights.ManipulatedVariables = 0.01*[1 1 1 1 1 1];
nlobjecto.Weights.ManipulatedVariablesRate = [0.1 0.1 0.1 0.1 0.1 0.1];
%Constraints
nlobjecto.Optimization.CustomIneqConFcn = 'myConstraintsFcn';
nlobjecto.Weights.ECR = 1e4;       


