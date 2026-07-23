clear
clc
close all

%%

n = 2; % Number of evaders

% pursuer_position = randn(2,1);
% evader_positions = 1+3*randn(2,n);
% target_position = [0;0];

% pursuer_position = [0;0];
% evader_positions = [2,-2;2,2];
% target_position = [1;-1];

% Paper example 1
pursuer_position = [-0.45;-0.55];
evader_positions = [0.85,0.5;0.65,-0.2];
target_position = [-0.6;0];

% Paper example 2
% pursuer_position = [-0.44;0];
% evader_positions = [0.6,0.55;0.73,-0.73];
% target_position = [-0.6;0];

% Paper example 3
% pursuer_position = [0.55;-0.31];
% evader_positions = [0.63,-0.17;0.74,-0.9];
% target_position = [-0.2;0];

pursuer_position = [-0.865;-0.2946];
evader_positions = [-0.02,1.079;-0.939,-0.45];
target_position = [-0.25;0.5];

% System parameters
timestep = 0.05;
tolerance = 0.35;

env = Environment(n,timestep,tolerance,pursuer_position,evader_positions,target_position);

%%

pursuer_policy = "closest_next_step";
% Options: "closest_next_step", "standard", "squaresum", "squaresump", "heuristic", "closest"
env.set_pursuer_policy(pursuer_policy);

[done, history] = env.simulate(10000, true);