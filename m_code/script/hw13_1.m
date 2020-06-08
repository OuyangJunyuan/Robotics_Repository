clear all;

pset=[5,15,40];
tset=[0, 1, 2];
a=80;
dt=0.001;
[p,t]=LFPB_ViaPoint(pset,tset,a,dt);

subplot(3,1,1);
plot(t,p,'r');hold on;
plot(tset,pset,'-*b');

subplot(3,1,2);
p=diff(p)/dt;
plot(t(1:length(t)-1),p,'r')

subplot(3,1,3);
p=diff(p)/dt;
plot(t(1:length(t)-2),p,'r')





