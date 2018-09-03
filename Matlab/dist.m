function [ distance ] = dist( point1, point2 )
%calculates distance between two points in any dimensional space
temp = 0;
for i = 1:length(point1)
    temp = temp+(point1(i)-point2(i))^2;
end
distance = sqrt(temp);

end

