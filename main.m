clear
clc
close all

%%

n = 1; % Number of evaders
 
pursuer_position = [0;-1]; % Pursuer position 2*1
evader_positions = [0;1]; % Evader positions 2*n
target_position = [1;-0.5]; % Target position
timestep = 0.01;
pursuer_policy = "closest_next_step";
% Options: "closest_next_step", "standard", "squaresum", "squaresump", "heuristic", "closest"

env = Environment(n,timestep,pursuer_position,evader_positions,target_position,pursuer_policy);

%%

[done, history] = env.simulate(10000, true);
