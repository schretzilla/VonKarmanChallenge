function [ new_point ] = rot_line( input_point, axis_origin, axis_vector, theta )
%rot_line rotates a point through a rotation theta around an axis with an
%origin axis_origin and direction axis_vector.
%   Note that theta is in Degrees

%   Normalize axis vector
axis_vector = axis_vector./norm(axis_vector);

%   Assign shorter names for readability
x = input_point(1);
y = input_point(2);
z = input_point(3);
a = axis_origin(1);
b = axis_origin(2);
c = axis_origin(3);
u = axis_vector(1);
v = axis_vector(2);
w = axis_vector(3);
th = theta;

%   Initialize output point
new_point = zeros(3,1);

%   Calculate output point
new_point(1) = (a*(v^2+w^2) - u*(b*v+c*w-u*x-v*y-w*z)) * (1-cosd(th)) +...
    x*cosd(th) + (-c*v+b*w-w*y+v*z)*sind(th);
new_point(2) = (b*(u^2+w^2) - v*(a*u+c*w-u*x-v*y-w*z)) * (1-cosd(th)) +...
    y*cosd(th) + (c*u-a*w+w*x-u*z)*sind(th);
new_point(3) = (c*(u^2+v^2) - w*(a*u+b*v-u*x-v*y-w*z)) * (1-cosd(th)) +...
    z*cosd(th) + (-b*u+a*v-v*x+u*y)*sind(th);

end

