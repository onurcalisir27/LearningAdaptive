clc; close all; clear;

Q=1;         %dimension of y vector
N=2;         %dimension of x vector
M=2;         %dimension of u vector

INPUTS=50;
STEPS=12;
h=0.001;
h1=h/STEPS;
gain=[2000 -1000]';
param=[-1 1 0.01 0.01]';
p=[1000 0 0 0
   0 1000 0 0
   0 0 1000 0
   0 0 0 1000];

for i1=2:INPUTS+1
    param=[param [-1 1 0.01 0.01]'];
    p=[p [1000 0 0 0; 0 1000 0 0; 0 0 1000 0; 0 0 0 1000]];
end

dtheta=[.000 .1 .2 .4 .8 1.5 2.0 1.5 .8 .4]';
for i1=11:INPUTS+1
    dtheta=[dtheta; .8+sin(i1-10)];
end

time=[];

for ti=0:INPUTS
    time=[time ti];
end

theta=[];
omega=[];
u=[];

for i1=1:INPUTS+1
    theta=[theta; 0];
    u=[u; 0];
    omega=[omega; 0];
end

toterr=[];

tn=0;
count=1;
pderr=[0 0]';
error=0;
pendulum_controller = LSTR(0.98);

while 1
    [pderr,u,p,param,theta,omega,tn]= pendulum_controller.solve(count,...
        param,p,dtheta,theta,omega,tn,h1,u,pderr,STEPS,gain);

    if count<INPUTS
        error=error+abs(theta(count+1)-dtheta(count+1))^2;
    end

    count=count+1;

    if count==INPUTS+1
        toterr=[toterr error];

        time = time * h;
        colr = 'g+';
        plot(time,theta-dtheta, colr),hold on
        plot(time,theta-dtheta, colr(1)),pause(1);
        plot(time,theta-dtheta, colr(1)),pause(1);
        disp('Error:'),disp(theta-dtheta);
        return;

    end
end
