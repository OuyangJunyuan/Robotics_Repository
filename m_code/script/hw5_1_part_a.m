clear;
syms l0 l1 l2 l3 x1 x2 x3 x1d x2d x3d;
l0=4;l1=4;l2=1;l3=1;x1d=0;x2d=0;x3d=1;
tt(6,100)=0;
for k=1:50
    x1=0;x3=0;x2=2*pi*k/100;
    t3=twist_wqh([0,0,1]',[0,l1+l2,0]',0);
    t2=twist_wqh([0,0,1]',[0,l1,0]',0);
    t1=twist_wqh([0,0,1]',[0,0,0]',0);
    g3=twist2gab(t3,x3);
    g2=twist2gab(t2,x2);
    g1=twist2gab(t1,x1);
    g0=	gab_Rp(eye(3,3),[0,l1+l2,l0]');
    gst=g1*g2*g3*g0;
    
    t23=twist_wqh([0,0,1]',[0,l2,0]',0);
    t12=twist_wqh([0,0,1]',[0,l1,0]',0);
    t01=twist_wqh([0,0,1]',[0,0,0]',0);
    Vs23=t23*x3d;
    Vs12=t12*x2d;
    Vs01=t01*x1d;
    g12=gab_Rp(Rort([0,0,1]',x2),[0,l1,l3]');
    g01=gab_Rp(Rort([0,0,1]',x1),[0,0,l0]');
    Adg01=Adgab(g01);
    Adg12=Adgab(g12);
    Vs03=Vs01+Adg01*(Vs12+Adg12*Vs23);
    Vs03h=hat(Vs03);
    qa=gst*[0;1;0;1];
    tt(1:3,k)=qa(1:3)';
    v=Vs03h*qa;
    tt(4:6,k)=v(1:3);
    
end
plot3(tt(1,:),tt(2,:),tt(3,:),'g*',tt(4,:),tt(5,:),tt(6,:),'b');
grid on;
xlabel('x(t)')
ylabel('y(t)')
zlabel('z(t)')





