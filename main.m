clear all;
clc;

%% define the robot structure 

% initialize all thetas to zero
link_1 = [0 0.76    0       pi/2    ];
link_2 = [0 -0.2365 0.4323  0       ];
link_3 = [0 0       0       pi/2    ];
link_4 = [0 0.4318  0       -pi/2   ];
link_5 = [0 0       0       pi/2    ];
link_6 = [0 0.20    0       0       ];

% the DH table: 
DH = [link_1; link_2; link_3; link_4; link_5; link_6];

% construct the robot using the toolbox function 'Link' and 'SerialLink'
myrobot = mypuma560(DH);

%% 4.2 plot a sample joint space trajectory 

% based on the lab manual instruction given: 
theta_1 = linspace(0, pi, 200)';
theta_2 = linspace(0, pi/2, 200)';
theta_3 = linspace(0, pi, 200)';
theta_4 = linspace(pi/4, 3*pi/4, 200)';
theta_5 = linspace(-pi/3, pi/3, 200)';
theta_6 = linspace(0, 2*pi, 200)';

q = [theta_1 theta_2 theta_3 theta_4 theta_5 theta_6];

% figure;
% plot(myrobot, q);

%% forward kinematics: 
o = zeros(200, 3);
for i = 1:200
    H   = forward(q(i, :)', myrobot); % end-effector pose relative to base
    o(i, :) = H(1:3, 4)';   
end 

figure; 
plot3(o(:,1), o(:,2), o(:,3), 'r');
hold on;
plot(myrobot, q);
%% inverse kinematics 

% test trial: 
H = [cos(pi/4) -sin(pi/4) 0  0.20; 
     sin(pi/4)  cos(pi/4) 0  0.23; 
     0          0         1  0.15; 
     0          0         0  1];

q = inverse(H, myrobot); % function that input the desired position and orientation (in matrix form)...
% output the joint angles in a 1*6 matrix form 


p1 = [0.1; 0.23; 0.15];
p2 = [0.3; 0.3;  1];
t  = linspace(0, 1, 100);
d  = p1*(1-t) + p2*t;
R  = rotz(pi/4);

q  = zeros(size(t,2), 6); 
for i = 1:size(t, 2)
    Ht      = [R     d(:, i); 
               0 0 0      1];
    q(i, :) = inverse(Ht, myrobot);
end


lines = vertcat(p1', p2');

figure
plot3(d(1,:),d(2,:),d(3,:),'r', 'LineWidth', 4);
hold on;
% plot3(line(1,:),line(2,:),line(3,:),'k');
plot3(lines(:, 1),lines(:, 2),lines(:, 3));
hold on;
plot(myrobot, q);













