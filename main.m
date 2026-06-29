clear
clc
close all

%%

n = 1; % Number of evaders
 
pursuer_position = [0;-1]; % Pursuer position 2*1
evader_positions = [0;1]; % Evader positions 2*n
target_position = [1;-0.5]; % Target position
timestep = 0.01;

env = Environment(n,timestep,pursuer_position,evader_positions,target_position);

%%

