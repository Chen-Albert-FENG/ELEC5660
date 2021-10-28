% Used for HKUST ELEC 5660 

close all;
clear all;
clc;
addpath('./utils','./readonly');

h1 = subplot(3,3,1);
h2 = subplot(3,3,2);
h3 = subplot(3,3,3);
h4 = subplot(3,3,4);
h5 = subplot(3,3,5);
h6 = subplot(3,3,6);
h7 = subplot(3,3,7);
h8 = subplot(3,3,8);
h9 = subplot(3,3,9);
set(gcf, 'Renderer', 'painters');
set(gcf, 'Position', [100, 100, 1400, 1000]);
% set(gcf, 'WindowStyle','Modal');

% Environment map in 3D space 
map1 = [1.0 1.0 1.0 ; ... % start point
       1.0 2.0 1.0 ; ...  %  obstacle
       3.0 3.0 1.0 ; ...  %     .
       4.0 3.0 2.0 ; ...  %     .
       1.0 5.0 1.0 ; ... %     .
       3.0 5.0 1.0 ; ...  %     .
       2.0 7.0 2.0 ; ...  %  obstcle
       2.0 9.0 1.0 ];     % target point

map2 = [1.0 1.0 1.0 ; ...
       2.0 1.0 1.0 ; ... 
       3.0 3.0 1.0 ; ... 
       1.0 3.0 1.0 ; ... 
       2.0 5.0 1.0 ; ...
       4.0 5.0 1.0 ; ...  
       3.0 7.0 1.0 ; ...  
       4.0 9.0 1.0 ]; 

test_map = map1;

% Waypoint Generator Using the A*   
Optimal_path = path_from_A_star(test_map);
disp(Optimal_path);

% Trajectory Generator Using waypoint
trajectory_generator([], Optimal_path, h1, test_map);
global T_pre e_p_int phi_c_pre theta_c_pre count P_MSE_list V_MSE_list des cur des_v cur_v
global P_RMS V_RMS
count = -1;
T_pre = 0;
e_p_int = zeros(3,1);
phi_c_pre = 0;
theta_c_pre = 0;
P_MSE_list = [];
V_MSE_list = [];
des = [];
des_v = [];
cur = [];
cur_v = [];

P_RMS = 0;
V_RMS = 0;

% Run Trajectory
run_trajectory_readonly(h1, h2, h3, h4, h5, h6, h7, h8, h9);
P_MSE_list(1) = cur(1);
P_MSE_list(end) = des(end);
V_MSE_list(1) = cur_v(1);
V_MSE_list(end) = des_v(end);
P_RMS = sum(P_MSE_list);
V_RMS = sum(V_MSE_list);
P_RMS = sqrt(P_RMS/count);
V_RMS = sqrt(V_RMS/count);
disp("Postion_RMS: "+num2str( P_RMS));
disp("Velocity_RMS: "+num2str(V_RMS));

