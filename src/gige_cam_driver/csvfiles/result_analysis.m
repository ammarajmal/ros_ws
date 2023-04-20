clear all
close all
clc

% % Plotting of 3D Displacement results
% Summary of example objective

% % reading data form files
% Description of first code block
% Read CSV file into a table with original column headers
dataTable = readtable('camera_1_30s_2023-04-20_06-13-09_2Hz.csv', 'VariableNamingRule', 'preserve');

% % Section 2 Title
xDisplacementsCamera = dataTable.("field.transforms0.transform.translation.x");
yDisplacementsCamera = dataTable.("field.transforms0.transform.translation.y");
zDisplacementsCamera = dataTable.("field.transforms0.transform.translation.x");

time = dataTable.("%time")
figure()
t_time = time(1:10,:)
disp_ = yDisplacementsCamera(1:10,:)
plot(t_time, disp_)

