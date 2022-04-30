%{
%Full program for control of robot
%By: Charles Chano, ... add names

%Assumed Robot Link Lengths




%}
clear all;clc;
%//////////////////////////////////
%INPUTS
theta_pose_home = [0 0 0 0 0 0];
T06_home = forwardKin(theta_pose_home);

%Input goal position and orientation, but set it to to the same as the home
%pose
Goal_pt = [-5 -5 4]';
Goal_orientation = T06_home(1:3,1:3);
T06_goal = zeros(4,4);
T06_goal(4,4) = 1;
T06_goal(1:3,4) = Goal_pt;
T06_goal(1:3,1:3) = Goal_orientation;

%Determine rate of pt to pt motion
t_final = 9;%sec

%/////////////////////////////////////

%Get starting position
theta_start = theta_pose_home;

%Determine goal joint angles with inverse
[theta_finish, q_d] = invCalc(T06_goal);
%theta_finish = [pi 0 0 0 0 0];

%Start timer to record elapsed time during trajectory motion
tic
dt = toc;

while dt <= t_final
%Get current time since start of time keeping
dt = toc;

%///////////////////////////////////////////////////////////////
%next_traj_thetas = next_theta(theta_start, theta_finish, dt, t_final);
    D = theta_finish - theta_start;
    
    next_traj_thetas = zeros(length(theta_start));
    for i=1:length(theta_start)
        coef_a0 = theta_start(i);
        % recall coef_a1 = 0;
        coef_a2 = (3/(t_final^2))*D(i);
        coef_a3 = -(2/(t_final^3))*D(i);
        next_traj_thetas(i) = coef_a0 + (coef_a2*(dt^2)) + (coef_a3*(dt^3));
    end
%//////////////////////////////////////////////////////////////////////

[TransToJoint_j,end_effector_trans] = next_TransformPose(next_traj_thetas);

%Get coordinate point to put for plotting frame 0
Base_Pos = 0;
    
X = cat(1,Base_Pos,squeeze(TransToJoint_j(1,4,:)));
Y = cat(1,Base_Pos,squeeze(TransToJoint_j(2,4,:)));
Z = cat(1,Base_Pos,squeeze(TransToJoint_j(3,4,:)));
X = cat(1,X,squeeze(end_effector_trans(1,:)'));
Y = cat(1,Y,squeeze(end_effector_trans(2,:)'));
Z = cat(1,Z,squeeze(end_effector_trans(3,:)'));


plot3(X,Y,Z,'r-o','LineWidth',3) 

xlim([-10 10])
ylim([-10 10])
zlim([0 10])

zlabel('z');
ylabel('y');
xlabel('x');
grid on
grid minor
drawnow

%Get current time since start of time keeping
dt = toc;
end

%calculates coordinates for pose visualization
function [TransToJoint_j,end_effector_trans] = next_TransformPose(theta)
theta(2) = theta(2) - pi/2;
theta(5) = theta(5) - pi/2;

alpha = [-pi/2, 0, -pi/2, pi/2, pi/2, 0];
a_xdisp = [0, 3, 1, 0 ,0 ,0];
d_zdisp = [1, 0, 0, 3, 0, 1];

DH_m = zeros(4,4,6);
for i=1:6
    DH_m(:,:,i) = [cos(theta(i)), -sin(theta(i))*cos(alpha(i)), sin(theta(i))*sin(alpha(i)), a_xdisp(i)*cos(theta(i));...
                    sin(theta(i)), cos(theta(i))*cos(alpha(i)), -cos(theta(i))*sin(alpha(i)), a_xdisp(i)*sin(theta(i));...
                        0,              sin(alpha(i)),                  cos(alpha(i)),                  d_zdisp(i); ...
                        0,                      0,                          0,                              1];
end

TransToJoint_j= zeros(4,4,6);
TransToJoint_j(:,:,1) = (DH_m(:,:,1));
for j=2:6
    TransToJoint_j(:,:,j) = TransToJoint_j(:,:,j-1)*DH_m(:,:,j);
end

%end effector calc (x,y,z)
end_effector_pts = [0       0         0     0;...
                  -0.5      -0.5    0.5     0.5;...
                     0      -0.5       0     -0.5;...
                     1       1      1       1];

end_effector_trans = zeros(4,4);
for n = 1:4
end_effector_trans(:,n) = TransToJoint_j(:,:,6)*end_effector_pts(:,n);
end


end 

%{
%calculates the next set of theta values
function next_traj_thetas = next_theta(theta_start, theta_finish, dt, t_final)

D = theta_finish - theta_start;
    
    next_traj_thetas = zeros(length(theta_start));
    for i=1:length(theta_start)
        coef_a0 = theta_start(i);
        % recall coef_a1 = 0;
        coef_a2 = (3/(t_final^2))*D(i);
        coef_a3 = -(2/(t_final^3))*D(i);
        next_traj_thetas(i) = coef_a0 + (coef_a2*(dt^2)) + (coef_a3*(dt^3));
    end

end
%}

function T06 = forwardKin(joint_theta)
%1)Determine Homogeneous matrix from DH parameters
%   a)Apply angle values to attain a known homogeneous matrix
%2)Determine angle values from the known homogeneous matrix
syms theta1 theta2 theta3 theta4 theta5 theta6
syms alpha1 alpha2 alpha3 alpha4 alpha5 alpha6
syms d_zdisp1 d_zdisp2 d_zdisp3 d_zdisp4 d_zdisp5 d_zdisp6
syms a_xdisp1 a_xdisp2 a_xdisp3 a_xdisp4 a_xdisp5 a_xdisp6

theta = [joint_theta(1) joint_theta(2)-(pi/2) joint_theta(3) joint_theta(4) joint_theta(5)-(pi/2) joint_theta(6)];
alpha = [-pi/2, 0, -pi/2, pi/2, pi/2, 0];
a_xdisp = [0, 3, 1, 0 ,0 ,0];
d_zdisp = [1, 0, 0, 3, 0, 1];
%
for i = 1:6
DH_m = [cos(theta(i)), -sin(theta(i))*cos(alpha(i)), sin(theta(i))*sin(alpha(i)), a_xdisp(i)*cos(theta(i));...
        sin(theta(i)), cos(theta(i))*cos(alpha(i)), -cos(theta(i))*sin(alpha(i)), a_xdisp(i)*sin(theta(i));...
        0, sin(alpha(i)), cos(alpha(i)), d_zdisp(i); ...
        0, 0, 0, 1];
    if i == 1
        T01 = DH_m;
    end
    if i == 2
        T12 = DH_m;
    end
    if i == 3
        T23 = DH_m;
    end
    if i == 4
        T34 = DH_m;
    end
    if i == 5
        T45 = DH_m;
    end
    if i == 6
        T56 = DH_m;
    end
end

T06 = T01*T12*T23*T34*T45*T56;
fprintf('Forward Kin Matrix: T06 \n')
disp(T06);

end

function [q_angles_rad, q_d] = invCalc(T06_goal)
%1)Determine Homogeneous matrix from DH parameters
%   a)Apply angle values to attain a known homogeneous matrix
%2)Determine angle values from the known homogeneous matrix
syms theta1 theta2 theta3 theta4 theta5 theta6
syms alpha1 alpha2 alpha3 alpha4 alpha5 alpha6
syms d_zdisp1 d_zdisp2 d_zdisp3 d_zdisp4 d_zdisp5 d_zdisp6
syms a_xdisp1 a_xdisp2 a_xdisp3 a_xdisp4 a_xdisp5 a_xdisp6

theta = [theta1 theta2 theta3 theta4 theta5 theta6];
alpha = [-sym(pi/2), 0, -sym(pi/2), sym(pi/2), sym(pi/2), 0];
a_xdisp = [0, 3, 1, 0 ,0 ,0];
d_zdisp = [1, 0, 0, 3, 0, 1];
%

%Take the forward kinematics angles as input for the inverse kinematics
%problem

%Acquire rotation matrix of homogeneous matrix
R06 = T06_goal(1:3,1:3);
%Acquire translation vector from homogeneous matrix
P06 = T06_goal(1:3,4);

%Determine the position from the base frame to the robot's wrist center
%Pc = P06 - d6*[r13; r23; r33]
Pc = P06 - R06(1:3,3).*d_zdisp(6);

%Find first angle (theta 1) in inverse kinematic problem
%!!!! Consider that atan2's range is pi to -pi, NOT 0 to 2pi  !!!!!
q1 = atan2(Pc(2), Pc(1));
q1_d = (180/pi)*q1;



r = sqrt(Pc(1)^2 + Pc(2)^2);
s = sqrt((Pc(3) - d_zdisp(1))^2 + r^2);
d24 = sqrt(a_xdisp(3)^2 + d_zdisp(4)^2);
alpha_q2 = atan2(Pc(3)-d_zdisp(1), r);
beta_q2 = acos((a_xdisp(2)^2 + s^2 - d24^2)/(2*a_xdisp(2)*s));

%Find 2nd angle (theta 2) in inverse kinematic problem (based on theta 3)
q2 = (pi/2) - beta_q2 - alpha_q2;
q2_d = (180/pi)*q2;

%Find 3rd angle (theta 3) in inverse kinematic problem
gamma_q3 = (acos((a_xdisp(2)^2 + d24^2 - s^2)/(2*a_xdisp(2)*d24)));
phi_q3 = atan2(d_zdisp(4),a_xdisp(3));

q3 = pi - gamma_q3 - phi_q3;
q3_d = (180/pi)*q3;



%Use values set in forward kinematics
%a_xdisp = [0, a_xdisp2, a_xdisp3, 0 ,0 ,0]; 

%Use values set in forward kinematics
%d_zdisp = [d_zdisp1, 0, 0, d_zdisp4, 0, d_zdisp6];

%
for i = 1:6
DH_m = [cos(theta(i)), -sin(theta(i))*cos(alpha(i)), sin(theta(i))*sin(alpha(i)), a_xdisp(i)*cos(theta(i));...
        sin(theta(i)), cos(theta(i))*cos(alpha(i)), -cos(theta(i))*sin(alpha(i)), a_xdisp(i)*sin(theta(i));...
        0,                           sin(alpha(i)),                cos(alpha(i)),               d_zdisp(i); ...
        0, 0, 0, 1];
    if i == 1
        T01 = DH_m;
    end
    if i == 2
        T12 = DH_m;
    end
    if i == 3
        T23 = DH_m;
    end
    if i == 4
        T34 = DH_m;
    end
    if i == 5
        T45 = DH_m;
    end
    if i == 6
        T56 = DH_m;
    end
end

T03 = T01*T12*T23;
T03_check = T03;
T06_check = T01*T12*T23*T34*T45*T56;

fprintf('Matrix: T03 with variables\n')
disp(T03);

%invT03_T06 = T03\T06;
invT03_T06 = T34*T45*T56;
fprintf('Matrix: (T03)^-1*T06 with variables \n')
disp(invT03_T06);

%Solve angles for determined values (substitute theta for q)
T03 = subs(T03,{cos(theta1),sin(theta1), sin(theta2), cos(theta2), cos(theta3), sin(theta3)},{cos(q1),sin(q1), sin(q2-(pi/2)), cos(q2-(pi/2)), cos(q3), sin(q3)});
T03 = real(double(T03));
fprintf('Matrix: T03 \n')
disp(T03);

invT03_T06 = T03\T06_goal;
fprintf('Matrix: (T03)^-1*T06 \n')
disp(invT03_T06);

if abs(invT03_T06(3,3)) == 1
    
    q5 = (asin(invT03_T06(3,3)));
    q5_d = (180/pi)*q5;

    if q5 >= 0
        q4 = pi/2;
        q6 = (atan(invT03_T06(2,1)/invT03_T06(1,1))) - q4;
    else
        q6 = pi/2;
        q4 = (atan(invT03_T06(2,1)/invT03_T06(1,1))) - q6;
    end
else
    
    %q5 = atan2(sqrt(1 - invT03_T06(3,3)^2), invT03_T06(3,3));
    q5 = (asin(invT03_T06(3,3))) ;
    q5_d = (180/pi)*q5;
    
    q4 = (atan(invT03_T06(2,3)/invT03_T06(1,3))) ;
    q4_d = (180/pi)*q4;
    
    %q5 = atan2(sqrt(invT03_T06(3,1)^2 + invT03_T06(3,2)^2), invT03_T06(3,3));
    %q5 = acos(-invT03_T06(3,3)); 
    q6 = (atan(-invT03_T06(3,2)/invT03_T06(3,1))) ;
    q6_d = (180/pi)*q6;
end

%Verify Matrix T36 is comparable to input
T36_check = T03_check\T06_check;
fprintf('Matrix: T36_check \n')
disp(T36_check);

%Solve angles for determined values (substitute theta for q)
T36_check = subs(T36_check,{cos(theta4),sin(theta4), sin(theta5), cos(theta5), cos(theta6), sin(theta6)},{cos(q4),sin(q4), sin(q5-(pi/2)), cos(q5-(pi/2)), cos(q6), sin(q6)});
T36_check = real(double(T36_check));
fprintf('Matrix: T36_check \n')
disp(T36_check);


T06_check = T03*T36_check;
fprintf('Matrix: T06_check \n')
disp(T06_check);
q_angles_rad = [q1, q2, q3, q4, q5, q6];
fprintf('Angles: q1, q2 - pi/2 , q3, q4, q5 - pi/2, q6 \n')
q_d = [q1_d, q2_d, q3_d, q4_d, q5_d, q6_d];
disp(q_d)

end