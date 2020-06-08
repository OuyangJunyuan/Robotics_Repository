clear all;
clc;
t=[0,10,20,40,60,80,100,140,180,250,300,400,500,600];
y=[0,0,0.2,0.8,2.0,3.6,5.4,8.8,11.8,14.4,16.6,18.4,19.2,19.6];

num1=100;den1=[225,1];sys1=tf(num1,den1,'iodelay',40);
num2=100;den2=[144,1];sys2=tf(num2,den2,'iodelay',56);
[y1,t1]=step(sys1,0:1:600);
[y2,t2]=step(sys2,0:1:600);
y1=y1*0.2;
y2=y2*0.2;
subplot(2,1,1)
plot(t,y,'r',t1,y1,'g',t2,y2,'b');
grid on;
title('波形比较');
legend('原始数据','图形法','计算法');
for kk=1:length(t)
    dy1(kk)=y(kk)-y1(t(kk)+1);
    dy2(kk)=y(kk)-y2(t(kk)+1);
end
subplot(2,1,2)
plot(t,dy1,'g',t,dy2,'b')
grid on;
title('误差比较');
legend('图形法','计算法');
