clear all 
close all 
clc;

filepath = './circular_workspace.traj';

no_of_cp = 10; % minimo numero di punti per ottenere la traj circ
no_of_samples = 70;

rho = 0.3;
theta = linspace (-pi, pi, no_of_cp);

ctrl_points = NaN * ones(3, no_of_cp);
ctrl_points(1,:) = 0.9 * ones(1, no_of_cp); 
ctrl_points(2,:) =  rho * cos(theta);
ctrl_points(3,:) = rho * sin(theta) + 0.9; % da rViz la circonf era troppo bassa

[x_lambda, lambda] = generate_path(ctrl_points, no_of_samples, true); 

x_lambda(4,:) = zeros(1, no_of_samples);
x_lambda(5,:) = pi/2 * ones(1, no_of_samples);
x_lambda(6,:) = zeros(1, no_of_samples);

export_ros_workspace_path(filepath, lambda, x_lambda);