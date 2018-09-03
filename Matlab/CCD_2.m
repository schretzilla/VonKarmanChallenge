%%  2D Cyclic Coordinate Descent Algorithm
%   Avery Bodenstein
%   9/1/2018

%%  Inputs
clc; clear; close all;

target = [1;0.5]; % target point

num_of_link = 7; % number of links
%   Set up initial joint X locations
xdata = (0:num_of_link);
%   Set up initial joint Y locations
ydata = zeros(1,num_of_link+1);
%   Set up initial joint angles
angledata = zeros(1,num_of_link+1);
%   Error threashold
threashold = 0.5;

%   Set up figure
figure
title('Planar Robot')
axis([-10 10 -10 10])
hold on

%% Run CCD Algorithm

hnd = plot(xdata,ydata,'-r');    % plot robot manipulator
plot(target(1),target(2),'*')    % plot target

input('Enter to Continue...');

[output_angles, output_positions] = CCD_2D(angledata,[xdata;ydata],0,target,threashold,10);

for i = 1:1:size(xdata,2)
    plot(output_positions(1,:), output_positions(2,:), 'ob')
end

set(hnd,'xdata',output_positions(1,:),'ydata',output_positions(2,:)); % update data to plot  
