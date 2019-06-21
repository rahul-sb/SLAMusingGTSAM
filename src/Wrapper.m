%% Wrapper for P2Ph1 Project for CMSC828T Course at University of Maryland, College Park
% Code by: Rahul Subramonian Bama

% Download and install GTSAM toolbox from the following link
% https://borg.cc.gatech.edu/download.html
% 
% Add to MATLAB path the "data" and "libraries" folders and subfolders.

clc
clear all
close all

%% Add ToolBox to Path eg. ToolboxPath = 'gtsam_toolbox';
% addpath(ToolboxPath);

%% Load Data
% Download data from the following link: 
% https://drive.google.com/drive/folders/0B6NXn7BBGQf5MS0tUGR0Nno0Nk0
load('DataSquare.mat');
load('CalibParams.mat');

%% SLAM Using GTSAM
[LandMarksComputed, AllPosesComputed] = SLAMusingGTSAM(DetAll, K, TagSize, qIMUToC, TIMUToC,...
                                                IMU, 0, TLeftImgs);
                                            
