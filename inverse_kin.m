function q_angles = inverse_kin(x_p,y_p,z_p)
joint_theta(1)= 0;
joint_theta(2)= 90;
joint_theta(3)= 0;
joint_theta(4)= 0;
joint_theta(5)= 90;
joint_theta(6)= 0;

% arm manipulator Fixed arguments:
alpha = [-90, 0, -90, 90, 90, 0];
a = [0, 3, 1, 0 ,0 ,0];
d = [1, 0, 0, 3, 0, 1];

H0_1 = DHmatrix(joint_theta(1),d(1),a(1),alpha(1));
H1_2 = DHmatrix(joint_theta(2),d(2),a(2),alpha(2));
H2_3 = DHmatrix(joint_theta(3),d(3),a(3),alpha(3));
H3_4 = DHmatrix(joint_theta(4),d(4),a(4),alpha(4));
H4_5 = DHmatrix(joint_theta(5),d(5),a(5),alpha(5));
H5_6 = DHmatrix(joint_theta(6),d(6),a(6),alpha(6));

H0_2 = H0_1*H1_2;
H0_3 = H0_2*H2_3;
H0_4 = H0_3*H3_4;
H0_5 = H0_4*H4_5;
H0_6 = H0_5*H5_6;

% y_p =input('insert y position');
% x_p =input('insert x position'); 
% z_p= input('insert z position');
y_p = y_p - (d(6)*H0_6(1,3));
x_p = x_p - (d(6)*H0_6(2,3));
z_p = z_p - (d(6)*H0_6(3,3));

%inverse Kinamatics for the position:
   r    = real(sqrt(x_p^2+y_p^2));
   l3   = real(sqrt(a(3)^2+d(4)^2));
   s    = real(sqrt((z_p-d(1))^2+r^2));
   alfa = (atan2d((z_p-d(1)),r)); %removed abs
   D    = real((a(2)^2+s^2-l3^2)/(2*a(2)*s));
   beta = (atan2d(real(sqrt(1-D^2)),D)); %remove abs
   G    = real((l3^2+a(2)^2-s^2)/(2*a(2)*l3));
   gama = (atan2d(real(sqrt(1-G^2)),G)); %remove abs
   fai  = (atan2d(d(4),a(3)));  %remove abs
   
   theta_1 = atan2d(y_p,x_p);
   %{
    if theta_1 < 0
       theta_1 = atan2d(y_p,x_p) + 180;
   end
   %}
   theta_2 = 90 - (alfa + beta);
   if theta_2 < 0
       theta_2 = 90 + theta_2;
   end
   theta_3 = 180 -(gama+fai);
   
   h0_3_I = transpose (H0_3(1:3,1:3));
   R_H = h0_3_I * H0_6(1:3,1:3);

%{   
theta_5 = atan2d(real(sqrt(1-(R_H(3,3))^2)),R_H(3,3));
theta_5 = (theta_5);
theta_6 = atan2d(-1*(R_H(3,2)),R_H(3,1));
theta_6 = (theta_6);
theta_4 = atan2d(R_H(2,3),R_H(1,3));
theta_4 = (theta_4);
%}
if abs(R_H(3,3)) == 1
    fprintf("R(3,3) == 1") 
    q5 = atan2d(sqrt(1-(R_H(3,3))^2),R_H(3,3));
    theta_5 = (180/pi)*q5;

    if q5 >= 0
        fprintf("q5 > 0") 
        q4 = atan2(R_H(2,3),R_H(1,3));
        q6 = (atan2(R_H(2,1),R_H(1,1))) - q4;
        theta_4 = (180/pi)*q4;
        theta_6 = (180/pi)*q6;
    else
        fprintf("q5 < 0") 
        q6 = atan2(R_H(2,3),R_H(1,3));
        q4 = (atan2(R_H(2,1),R_H(1,1))) - q6;
        theta_4 = (180/pi)*q4;
        theta_6 = (180/pi)*q6;
    end
else
    fprintf("R(3,3) =/= 1")
    %q5 = atan2(sqrt(1 - invT03_T06(3,3)^2), invT03_T06(3,3));
    q5 = atan2d(sqrt(1-(R_H(3,3))^2),R_H(3,3));
    theta_5 = rad2deg(q5);
    
    q4 = (atan2(R_H(2,3),R_H(1,3))) ;
    theta_4 = rad2deg(q4);
    
    %q5 = atan2(sqrt(invT03_T06(3,1)^2 + invT03_T06(3,2)^2), invT03_T06(3,3));
    %q5 = acos(-invT03_T06(3,3)); 
    q6 = (atan2(-R_H(3,2),R_H(3,1))) ;
    theta_6 = rad2deg(q6); 
    
end

q_angles = [theta_1,theta_2,theta_3,theta_4,theta_5,theta_6] ;
end