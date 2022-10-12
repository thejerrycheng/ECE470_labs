clear all;
clc;

thetas = zeros(6); % initialize all thetas to zero

link_1 = [thetas(1) 0.76    0       pi/2    ];
link_2 = [thetas(2) -0.2365 0.4323  0       ];
link_3 = [thetas(3) 0       0       pi/2    ];
link_4 = [thetas(4) 0.4318  0       -pi/2   ];
link_5 = [thetas(5) 0       0       pi/2    ];
link_6 = [thetas(6) 0.20    0       0       ];

DH = [link_1; link_2; link_3; link_4; link_5; link_6];

my_robot = mypuma(DH);

theta_1 = linspace(0, pi/2, 200)';
theta_2 = linspace(0, pi/2, 200)';
theta_3 = linspace(0, pi/2, 200)';
theta_4 = linspace(pi/4, 3*pi/4, 200)';
theta_5 = linspace(-pi/3, pi/3, 200)';
theta_6 = linspace(0, 2*pi, 200)';

q = [theta_1 theta_2 theta_3 theta_4 theta_5 theta_6];
