%%  Line rotation Test
%   Avery Bodenstein
%   9/3/18

%%  Inputs
clc; clear; close all;

%   Set up rotation inputs
old_point = [1,2,3];
line_origin = [0,0,0];
line_vector = [0,0,1];
line = [0,0,0;0,0,1];
theta = 0:270;

%   Set up Figure
figure
xlim([-5,5])
ylim([-5,5])
zlim([-5,5])
axis square
hold on

%   Plot origin point and line
plot3(old_point(1),old_point(2),old_point(3),'ko')
plot3(line(:,1),line(:,2),line(:,3),'b-')

%   Calculate new points
new_point = zeros(length(theta),3);
for i = 1:length(theta)
    new_point(i,:) = rot_line(old_point,line_origin,line_vector,theta(i));
end

%   Plot new points
plot3(new_point(:,1),new_point(:,2),new_point(:,3),'r-')