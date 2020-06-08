clear all;
pset=[[0;0],[3;3],[2;1]];
%tset=[0,1+0.2596,2+0.2596];
tset=[0,1,2];
ax=6.3;ay=6;
dt=0.01;
[p1,t1]=LFPB_ViaPoint(pset(1,:),tset,ax,dt);
subplot(2,1,1);
plot(t1,p1);
p1=diff(p1);
subplot(2,1,2);
plot(t1(2:end),p1);

