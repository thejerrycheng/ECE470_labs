function my_robot = mypuma(DH)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

joint_num = size(DH, 1);

my_robot = Link.empty;

for i = 1:joint_num
    my_robot = my_robot + Link('revolute', DH(i, :));
end 

end