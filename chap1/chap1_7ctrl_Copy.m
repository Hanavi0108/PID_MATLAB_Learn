function [u]=chap1_7ctrl_Copy(u1,u2)
persistent pidmat errori error_1
% t=u1;
t = 0.001;
if t==0
   errori=0;
   error_1=0;
end
   
errori=0;
error_1=0;

% kp=2.5;
% ki=0.020;
% kd=0.50;

kp=1.5;
ki=2.0;
kd=0.05;

error=u2;
errord=error-error_1;
% errord=(error-error_1)/t;

errori=errori+error;
% errori=errori+error*t;

u=kp*error+kd*errord+ki*errori;
error_1=error;