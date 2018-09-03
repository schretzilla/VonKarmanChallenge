%%  3D Cyclic Coordinate Descent Algorithm
%   Avery Bodenstein
%   9/3/2018

%%  Inputs
clc; clear; close all;

target = [1;3;3]; % target point

num_of_link = 7; % number of links
%   Set up initial joint X locations
xdata = (0:num_of_link);
%   Set up initial joint Y locations
ydata = zeros(1,num_of_link+1);
%   Set up initial joint Z locations
zdata = zeros(1,num_of_link+1);
%   Set up initial joint angles
angledata = zeros(1,num_of_link+1);
%   Set up intiial joint axes
axisdata = zeros(3,num_of_link+1);
%   Error threashold
threashold = 0.5;

%   Set up figure
figure
title('Planar Robot')
axis([-10 10 -10 10 -10 10])
axis square
hold on

%% Run CCD Algorithm

hnd = plot3(xdata,ydata,zdata,'-r');    % plot robot manipulator
plot3(target(1),target(2),target(3),'*')    % plot target

%input('Enter to Continue...');

[output_angles, output_positions, output_axes] = CCD_3D(angledata,[xdata;ydata;zdata],axisdata,0,target,threashold,10);

for i = 1:1:size(xdata,2)
    plot(output_positions(1,:), output_positions(2,:), output_positions(3,:), 'ob')
end

set(hnd,'xdata',output_positions(1,:),'ydata',output_positions(2,:),'zdata',output_positions(3,:)); % update data to plot  
