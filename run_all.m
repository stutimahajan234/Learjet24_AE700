clc; clear; close all;

addpath(genpath(pwd));   % connect all folders

% Load aircraft parameters
MAV = learjet_parameters();   

% Open main simulation
 %open_system('models/full_sim/mavsim_full');

% Run simulation
%sim('models/full_sim/mavsim_full');