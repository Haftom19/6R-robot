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
TransToJoint_j(:,:,1) = ((DH_m(:,:,1)));
for j=2:6
    TransToJoint_j(:,:,j) = (TransToJoint_j(:,:,j-1)*DH_m(:,:,j));
end

%end effector calc (x,y,z)
end_effector_pts = [0       0         0     0;...
                  -0.5      -0.5    0.5     0.5;...
                     0      -0.5       0     -0.5;...
                     1       1      1       1];

end_effector_trans = zeros(4,4);
for n = 1:4
end_effector_trans(:,n) = (TransToJoint_j(:,:,6)*end_effector_pts(:,n));
end

end

