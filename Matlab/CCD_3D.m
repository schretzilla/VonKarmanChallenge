function [ output_angles, output_positions, output_axes ] = CCD_3D( current_angles,...
    current_positions, current_axes, constraints, goal, threashold, max_iterations )
%CCD_3D uses cyclic coordinate descent to calculate joint rotations for an
%inverse kinematic solution to a revolute joint 3d robot. Note that this
%only works for robots composed of entirely revolute joints, with only one
%axis of rotational freedom.
%
%   output_angles: 1 x n-1 array of calculated joint angles
%
%   output_positions: 3 x n array of calculated joint positions [x1, x2, ...,
%   xn; y1, y2, ..., yn; z1, z2, ..., zn];
%
%   output_axes: 3 x n array of calculated joint rotation axes [x;y;z]
%
%   current_angles: 1 x n-1 array of input joint angles [x; y; z]
%
%   current_positions: 3 x n array of current joint positions [x1, x2, ...,
%   xn; y1, y2, ..., yn; z1, z2, ..., zn];
%
%   current_axes: 3 x n array of current joint rotation axes [x;y;z]
%
%   constraints: 2 x n-1 array of max/min joint angles 
%   [min_theta_1, ..., min_theta_n; max_theta_1, ..., max_theta_n]
%
%   goal: 3 x 1 array of desired end effector position
%
%   threashold: single value, tolerance for how close the end effector
%   needs to be to the goal before returning
%
%   max_iterations: maximum CCD iterations before returning

%%  Inputs

num_of_link = length(current_positions)-1; %number of links
%   Retrieve initial joint X locations
xdata = current_positions(1,:);
%   Retrieve initial joint Y locations
ydata = current_positions(2,:);
%   Retrieve initial joint Z locations
zdata = current_positions(3,:);
%   Retrieve initial joint angles
angledata = current_angles;
jointAngle = angledata;
output_angles = jointAngle;
%   Retrieve initial joint rotation axes
axisdata = current_axes;

%% Plot
%   Set up figure
figure
title('3D Robot')
axis([-10 10 -10 10 -10 10])
xlabel('X')
ylabel('Y')
zlabel('Z')
axis square
hold on

% plot robot
hnd = plot3(xdata,ydata,zdata,'-r');
plot3(xdata,ydata,zdata,'ko')

% plot target
plot3(goal(1),goal(2),goal(3),'k*')

% plot rotation axes
x_axis_data = [xdata;xdata+axisdata(1,:)];
y_axis_data = [ydata;ydata+axisdata(2,:)];
z_axis_data = [zdata;zdata+axisdata(3,:)];
hnd2 = plot3(x_axis_data,y_axis_data,z_axis_data);

%%  Initialization of data

error = dist([xdata(num_of_link+1), ydata(num_of_link+1), zdata(num_of_link+1)], goal);
num_iterations = 0;
while (error > threashold && num_iterations < max_iterations)
    active_joint = num_of_link + 1;
    while (active_joint > 1) 
        %% CCD Algorthm
        % end effector position
        pe = [xdata(num_of_link+1); ydata(num_of_link+1); zdata(num_of_link+1)];
        % current joint position
        pc = [xdata(active_joint-1); ydata(active_joint-1); zdata(active_joint-1)];
        % vector from end effector to current joint
        a = (pe - pc)/norm(pe-pc);
        % vector from current joint to target
        b = (goal - pc)/norm(goal-pc);
        % get current joint rotation axis
        n = [axisdata(1,active_joint-1);axisdata(2,active_joint-1);axisdata(3,active_joint-1)];
        % project a,b onto plane normal to joint rotation axis
        a_prime = a - dot(a,n) .* n/norm(n);
        b_prime = b - dot(b,n) .* n/norm(n);
        % rotation angle for current joint to minimize b
        theta = acosd(dot(a_prime, b_prime));
        % check for rotation angle
        direction = cross([a_prime(1) a_prime(2) a_prime(3)],[b_prime(1) b_prime(2) b_prime(3)]);
        if dot(direction,n) < 0
            theta = -theta;
        end
        
        % Joint Constraints
        if (theta > 30)
            theta = 30;
        elseif (theta < -30)
            theta = -30;
        end
        disp(theta)

        angledata(active_joint) = theta;
        jointAngle(active_joint) = theta;

        
        %% Perform Rotation
        
        %   For first joint
        i = active_joint;
        while (i <= num_of_link+1)
            %   Rotate each joint around active joint
            new_point = rot_line([xdata(i),ydata(i),zdata(i)],...
                [xdata(active_joint-1),ydata(active_joint-1),zdata(active_joint-1)],...
                axisdata(:,active_joint-1),theta);
            %   rotate axis around active joint
            new_axis = rot_line([xdata(i)+axisdata(1,i),ydata(i)+axisdata(2,i),zdata(i)+axisdata(3,i)],...
                [xdata(active_joint-1),ydata(active_joint-1),zdata(active_joint-1)],...
                axisdata(:,active_joint-1),theta);
            %   update joint location
            xdata(i) = new_point(1);
            ydata(i) = new_point(2);
            zdata(i) = new_point(3);
            %   update axis
            new_axis = new_axis - [xdata(i);ydata(i);zdata(i)];
            new_axis = new_axis ./ norm(new_axis);
            axisdata(1,i) = new_axis(1);
            axisdata(2,i) = new_axis(2);
            axisdata(3,i) = new_axis(3);
            %   plot new link location
            set(hnd,'xdata',xdata,'ydata',ydata,'zdata',zdata);
            %   plot new rotation axes
            set(hnd2(i),'xdata',[xdata(i);xdata(i)+axisdata(1,i)],...
                'ydata',[ydata(i);ydata(i)+axisdata(2,i)],...
                'zdata',[zdata(i);zdata(i)+axisdata(3,i)]);
            i = i+1; 
        end
        
        % advance iteration
        active_joint = active_joint - 1;

    end
    % calculate error
    error = dist([xdata(num_of_link+1) ydata(num_of_link+1) zdata(num_of_link+1)], goal);
    % update output angles with current iteration deltas
    output_angles = output_angles + jointAngle;
    % update output axes
    output_axes = axisdata;
    % add to iteration count
    num_iterations = num_iterations + 1;
end

output_positions = [xdata;ydata;zdata];

end

