%Plot Trajectory Example
%Start timer to record elapsed time during trajectory motion
tic
dt = toc;
theta_start = 0;%degrees
theta_finish = 180;%degrees
D = theta_finish - theta_start;
i = 0;
t_final = 3; %3 seconds of pt to pt motion
while dt <= t_final
    i = i+1;
    %Get current time since start of time keeping
    dt = toc;
    time(i) = dt;
    %///////////////////////////////////////////////////////////////
    %next_traj_thetas = next_theta(theta_start, theta_finish, dt, t_final);
    
    coef_a0 = theta_start;
    coef_a1 = 0;
    coef_a2 = (3/(t_final^2))*D;
    coef_a3 = -(2/(t_final^3))*D;
    q(i) = coef_a0 + coef_a1*dt + (coef_a2*(dt^2)) + (coef_a3*(dt^3));
    v(i) = coef_a1 + (2*coef_a2*(dt)) + (3*coef_a3*(dt^2));
    a(i) = (2*coef_a2) + (6*coef_a3*(dt));
 
    
    %//////////////////////////////////////////////////////////////////////
    
end
figure(1)
plot(time,q,'r'); grid on; 
xlabel ('Time in seconds');
ylabel ('Position in degrees');

figure(2)
plot(time,v,'g'); grid on; 
xlabel ('Time in seconds');
ylabel ('Velocity in degrees/sec');

figure(3)
plot(time,a,'b'); grid on; 
xlabel ('Time in seconds');
ylabel ('Accel in degrees/sec^2');