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
%   output_axes: 3 x n-1 array of calculated joint rotation axes [x;y;z]
%
%   current_angles: 1 x n-1 array of input joint angles [x; y; z]
%
%   current_positions: 3 x n array of current joint positions [x1, x2, ...,
%   xn; y1, y2, ..., yn; z1, z2, ..., zn];
%
%   current_axes: 3 x n-1 array of current joint rotation axes [x;y;z]
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

%%  Initialization of data

error = dist([xdata(num_of_link+1), ydata(num_of_link+1), ydata(num_of_link+1)], goal);
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

        angledata(active_joint) = theta;
        jointAngle(active_joint) = theta;

        
        %% Perform Rotation
        
        %   For first joint
        i = active_joint;
        %   Calculate rotation matrix
        R = [cosd(angledata(i)) -sind(angledata(i)); sind(angledata(i)) cosd(angledata(i))]; % rotation matrix
        %   Rotate outer point around joint
        temp = R * ([xdata(i); ydata(i)] - [xdata(i-1); ydata(i-1)]) + [xdata(i-1); ydata(i-1)];
        xdata(i) = temp(1);
        ydata(i) = temp(2);
        angledata(i) = angledata(i) + angledata(i-1);
        
        %   For next joint(s)
        i = active_joint+1;
        while (i <= num_of_link+1)
            %   Rotate each joint around active joint
            temp = R * ([xdata(i); ydata(i)] - [xdata(active_joint-1); ydata(active_joint-1)]) + [xdata(active_joint-1); ydata(active_joint-1)];
            xdata(i) = temp(1);
            ydata(i) = temp(2);
            angledata(i) = angledata(i) + angledata(i-1);
            i = i+1;
        end
        
        % advance iteration
        active_joint = active_joint - 1;

    end
    % calculate error
    error = dist([xdata(num_of_link+1) ydata(num_of_link+1)], goal);
    % update output angles with current iteration deltas
    output_angles = output_angles + jointAngle;
    % add to iteration count
    num_iterations = num_iterations + 1;
end

output_positions = [xdata;ydata];

end
