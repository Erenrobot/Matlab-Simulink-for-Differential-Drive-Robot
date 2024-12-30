ts=0.001;
time = 0:0.1:10;
plot(time, desired_vel, time, real_vel,'r--')
plot(time, cm_des, time, cm_real,'r--')
plot(time, error,'r--')
plot(time, pwm,'r--')