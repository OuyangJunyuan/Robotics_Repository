clear all;
pset=[[0;0],[3;3],[2;1],[3;3]];
tset=[0,1,2,3];
ax=10;ay=10;
dt=0.01;
[p1,t1]=LFPB_ViaPoint(pset(1,:),tset,ax,dt);
[p2,t2]=LFPB_ViaPoint(pset(2,:),tset,ax,dt);

plot(p1,p2);

