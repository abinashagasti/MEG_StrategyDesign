clear
clc
close all

%%

n = 2; % Number of evaders
 
% pursuer_position = [0;-1]; % Pursuer position 2*1
% evader_positions = [0;1]; % Evader positions 2*n
% target_position = [1;-0.5]; % Target position

% pursuer_position = randn(2,1);
% evader_positions = 1+3*randn(2,n);
% target_position = [0;0];

pursuer_position = [0;1];
evader_positions = [2,-2;2,2];
target_position = [0;0];

timestep = 0.01;
env = Environment(n,timestep,pursuer_position,evader_positions,target_position);

%%

pursuer_policy = "closest_next_step";
% Options: "closest_next_step", "standard", "squaresum", "squaresump", "heuristic", "closest"
env.set_pursuer_policy(pursuer_policy);

[done, history] = env.simulate(10000, true);
