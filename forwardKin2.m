function H0_6 = forwardKin2(joint_theta)
theta = [joint_theta(1) joint_theta(2) joint_theta(3) joint_theta(4) joint_theta(5) joint_theta(6)];  

% arm manipulator Fixed arguments:
alpha = [-90, 0, -90, 90, 90, 0];
a = [0, 2, 2, 0 ,0 ,0];
d = [4, 0, 0, 3, 0, 4];

H0_1 = DHmatrix(theta(1),d(1),a(1),alpha(1));
H1_2 = DHmatrix(theta(2),d(2),a(2),alpha(2));
H2_3 = DHmatrix(theta(3),d(3),a(3),alpha(3));
H3_4 = DHmatrix(theta(4),d(4),a(4),alpha(4));
H4_5 = DHmatrix(theta(5),d(5),a(5),alpha(5));
H5_6 = DHmatrix(theta(6),d(6),a(6),alpha(6));

H0_2 = H0_1*H1_2;
H0_3 = H0_2*H2_3;
H0_4 = H0_3*H3_4;
H0_5 = H0_4*H4_5;
H0_6 = H0_5*H5_6;   % Total Forward K
end