clear all;
to=0;td=2;dt=0.001;
aglo=5;aglf=-10;aglv=15;
[p1,t1]=CubicPoly_Trajectory(aglo,aglv,0,0,to,to+td,dt);
[p2,t2]=CubicPoly_Trajectory(aglv,aglf,0,0,to+td,to+2*td,dt);

t=[t1,t2];p=[p1,p2];
subplot(3,1,1);
plot(t,p,'b');

subplot(3,1,2);
p=diff(p);
plot(t(1:length(t)-1),p,'b');

subplot(3,1,3);
p=diff(p);
plot(t(1:length(t)-2),p,'b');