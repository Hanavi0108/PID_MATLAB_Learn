%PID Controler with Partial differential
clear all;
close all;

ts=20;
sys=tf([1],[60,1],'inputdelay',80);
dsys=c2d(sys,ts,'zoh');
[num,den]=tfdata(dsys,'v');

u_1=0;u_2=0;u_3=0;u_4=0;u_5=0;
ud_1=0;
y_1=0;y_2=0;y_3=0;
error_1=0;
ei=0;

for k=1:1:100
time(k)=k*ts;

yd(k)=1.0;

%Linear model
y(k)=-den(2)*y_1+num(2)*u_5;

n(k)=0.01*rands(1);
y(k)=y(k)+n(k);

error(k)=yd(k)-y(k);

%PID Controller with partly differential
ei=ei+error(k)*ts;
kc=0.30;
ki=0.0055;
TD=140;

kd=kc*TD/ts;

Tf=180;
Q=tf([1],[Tf,1]);   %Low Freq Signal Filter

M=2;
if M==1      %Using PID with Partial differential
	alfa=Tf/(ts+Tf);
	ud(k)=kd*(1-alfa)*(error(k)-error_1)+alfa*ud_1;
	u(k)=kc*error(k)+ud(k)+ki*ei;
   ud_1=ud(k);
elseif M==2  %Using Simple PID
	u(k)=kc*error(k)+kd*(error(k)-error_1)+ki*ei;
end

%Restricting the output of controller
if u(k)>=10
   u(k)=10;
end
if u(k)<=-10
   u(k)=-10;
end

u_5=u_4;u_4=u_3;u_3=u_2;u_2=u_1;u_1=u(k);
y_3=y_2;y_2=y_1;y_1=y(k);
error_1=error(k);
end
figure(1);
plot(time,yd,'r',time,y,'k:','linewidth',2);
xlabel('time(s)');ylabel('yd,y');
legend('Ideal position signal','Position tracking');
figure(2);
plot(time,u,'r','linewidth',2);
xlabel('time(s)');ylabel('u');
figure(3);
bode(Q,'r');
dcgain(Q);