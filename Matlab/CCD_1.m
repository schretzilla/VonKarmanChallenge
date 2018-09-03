%%  2D Cyclic Coordinate Descent Algorithm
%   Avery Bodenstein
%   9/1/2018

%%  Inputs
clc; clear; close all;

target = [1;0.5]; % target point (in column matrix)

num_of_link = 7; %number of links
%   Set up initial joint X locations
xdata = (0:num_of_link);
%xdata = [0,0,0];
%   Set up initial joint Y locations
ydata = zeros(1,num_of_link+1);
%ydata = [0,1,2];
%   Set up initial joint angles
angledata = zeros(1,num_of_link+1);
jointAngle = angledata;
totalAngle = jointAngle;
%   Error threashold
threashold = 0.5;

%   Set up figure
figure
title('Planar Robot')
axis([-20 20 -20 20])
hold on

%% initialization of data

hnd = plot(xdata,ydata,'-r');    % plot robot manipulator
plot(target(1),target(2),'*')    % plot target

error = dist([xdata(num_of_link+1) ydata(num_of_link+1)], target);
while (error > threashold)
    iteration = num_of_link + 1;
    while (iteration > 1) 
        %% CCD Algorthm
        % end effector position
        pe = [xdata(num_of_link+1); ydata(num_of_link+1)];
        % current joint position
        pc = [xdata(iteration-1); ydata(iteration-1)];
        % vector from end effector to current joint
        a = (pe - pc)/norm(pe-pc);
        % vector from current joint to target
        b = (target - pc)/norm(target-pc);
        % rotation angle for current joint to minimize b
        theta = acosd(dot(a, b));
        % check for rotation angle
        direction = cross([a(1) a(2) 0],[b(1) b(2) 0]);
        if direction(3) < 0
            theta = -theta;
        end
        
        % Joint Constraints
        if (theta > 60)
            theta = 60;
        elseif (theta < -60)
            theta = -60;
        end

        angledata(iteration) = theta;
        jointAngle(iteration) = theta;

        
        %% Perform Rotation
        
        %   For first joint
        i = iteration;
        R = [cosd(angledata(i)) -sind(angledata(i)); sind(angledata(i)) cosd(angledata(i))]; % rotation matrix
        temp = R * ([xdata(i); ydata(i)] - [xdata(i-1); ydata(i-1)]) + [xdata(i-1); ydata(i-1)];
        xdata(i) = temp(1);
        ydata(i) = temp(2);
        angledata(i) = angledata(i) + angledata(i-1);
        set(hnd,'xdata',xdata,'ydata',ydata); %update data to plot
        
        %   For next joint(s)
        i = iteration+1;
        while (i <= num_of_link+1)
            temp = R * ([xdata(i); ydata(i)] - [xdata(iteration-1); ydata(iteration-1)]) + [xdata(iteration-1); ydata(iteration-1)];
            xdata(i) = temp(1);
            ydata(i) = temp(2);
            angledata(i) = angledata(i) + angledata(i-1);
            set(hnd,'xdata',xdata,'ydata',ydata); %update data to plot
            i = i+1;
        end
        
        % advance iteration
        iteration = iteration - 1;

    end
    error = dist([xdata(num_of_link+1) ydata(num_of_link+1)], target)
    totalAngle = totalAngle + jointAngle
    set(hnd,'xdata',xdata,'ydata',ydata); %update data to plot 
end

for i = 1:1:size(xdata,2)
    plot(xdata(i), ydata(i), 'ob')
end

set(hnd,'xdata',xdata,'ydata',ydata); %update data to plot  

disp('done!');