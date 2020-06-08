clear all;
clc;
t=[  1   3   4   5   8    10  15  20   25   30   40   50   60    70   80];
y=[0.46 1.7 3.7 9.0 19.0 26.4 45 59.9 72.2 80.9 91.3 96.4 99.2  100.3 100.8];
    
num=3000;den=myconv([9.91,1],[9.91,1])
sys=tf(num,den);
[ym,tm]=step(sys,1:82);
subplot(2,1,1);
ym=ym*2/60;
plot(t,y,'r',tm,ym,'b');
grid on;
title('曲线描绘');
legend('原始阶跃曲线','测定阶跃曲线')
subplot(2,1,2);
for kk=1:length(t)
    dy1(kk)=y(kk)-ym(t(kk)+1);
end
plot(t,dy1,'r')
title('误差');
legend('误差曲线');