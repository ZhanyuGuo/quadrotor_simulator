%% test quadrotor motion equation

%% clear before running
close all; clear; clc;

%% add path
addpath(genpath('./trajectory_generation/'), genpath('./controller'));

%% parameters
global params;
params = quadModel_readonly();


