clear;
syms l0 l1 l2 l3 x1 x2 x3 x1d x2d x3d;
%l0=4;l1=3;l2=2;l3=3;x1=0;x2=0;x3=0;x1d=0;x2d=0;x3d=0;
x1=0;x2=0;x3=123123123123;
t23=twist_wqh([0,0,1]',[0,l2,0]',0);
t12=twist_wqh([0,0,1]',[0,l1,0]',0);
t01=twist_wqh([0,0,1]',[0,0,0]',0);
Vs23=t23*x3d
Vs12=t12*x2d
Vs01=t01*x1d
g12=gab_Rp(Rort([0,0,1]',x2),[0,l1,l3]')
g01=gab_Rp(Rort([0,0,1]',x1),[0,0,l0]')
Adg01=Adgab(g01)
Adg12=Adgab(g12)
Vs03=Vs01+Adg01*(Vs12+Adg12*Vs23)
Vs03h=hat(Vs03)
%g03=gab_Rp(eye(3,3),[0,l1+l2,l0+l3]');
%inv(g03)*Vs03h*g03
%Adg03=Adgab(g03)
%hat(inv(Adg03)*Vs03)*[0,0,0,1]'
Vs03h*[0,l1+l2,l0+l3,1]'








