function [ distance ] = dist( point1, point2 )
%calculates distance between two points in 2d
distance = sqrt( (point1(1)-point2(1))^2 + (point1(2)-point2(2))^2 );
end

