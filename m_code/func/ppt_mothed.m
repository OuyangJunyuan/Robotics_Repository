function [x] = ppt_mothed(N)
dt=0.01;%采样间隔;
t1=0:dt:dt*N;%脉冲响应采样点
K=500;T=0.025;alphaT=0.15;a=0.5;N=length(t1)-1;%常数值
et=5;%采样间隔和仿真结束时间
num=K*[T,1];den=[alphaT,1,0];Gk=tf(num,den);  e=feedback(1,Gk);e=series(e,tf(1,[1,0])); %误差传递函数
%%%%%%%%%%%%%%%%%%%%%%
w=impulse(e,t1);
%%%%%%%%%%%%%%%%%%%%%
t2=-et-dt*N:dt:et;%仿真时间段
A=atan(a*t2);
u=a*cos(A).^2; %输入信号
%%%%
for k=1:length(t2)-N
    x(k)=0;
    for n=k-N:k
        x(k)=x(k)+ w(1+k-n)*u(n+N); %从第N个开始卷积，忽略前N个序列，得到的是N+k的卷积值
    end
end
x=dt*x;
end

