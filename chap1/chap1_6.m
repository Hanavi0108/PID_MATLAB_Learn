%Discrete PID control for continuous plant
clear
close all;

ts=0.001;  %Sampling time
xk = zeros(2,1);
e  = zeros(1,2000);
de = zeros(1,2000);
u  = zeros(1,2000);
time = zeros(1,2000);
y  = zeros(1,2000);
dy = zeros(1,2000);
yd = zeros(1,2000);


for k=2:1:2000
    time(k) = k*ts;

    yd(k)=0.50*sin(1*2*pi*k*ts);
  
    para=u(k-1);
    tSpan=[0 ts];
    % [tt,xx]=ode45('chap1_6plant_z',tSpan,xk,[],para);
%% 修改ode45部分
    options = odeset('reltol', 1e-8);
% [tt,xx]=ode45('chap1_6plant_z',tSpan,xk,options);
% 用句柄传输参数
    [tt,xx]=ode45(@(t,y) chap1_6plant_z(t,y,para), tSpan, xk);
% 取结果的最后一行
    xk = xx(length(xx),:);

    y(k) = xk(1); 

% 
    e(k)=yd(k)-y(k);

    de(k)=(e(k)-e(k-1))/ts; 

% PD
    u(k)=20.0*e(k)+0.50*de(k);

%Control limit  ??
    if u(k)>10.0
        u(k)=10.0;
    end
    if u(k)<-10.0
        u(k)=-10.0;
    end

% u_1=u(k); % e_1=e(k);
    e(k-1)=e(k);
end
%% 作图
figure(1);
plot(time,yd,'r',time,y,'k:','linewidth',2);
xlabel('time(s)');ylabel('yd,y');
legend('Ideal position signal','Position tracking');
figure(2);
plot(time,yd-y,'r','linewidth',2);
xlabel('time(s)'),ylabel('error');

function dy = chap1_6plant_z(t,y,para)
    u = para;
    J = 0.0067;
    B = 0.1;
    dy = zeros(2,1);
    dy(1) = y(2);
    dy(2) = -(B/J)*y(2) + (1/J)*u;
end