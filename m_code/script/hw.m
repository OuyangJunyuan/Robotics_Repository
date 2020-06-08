clear;
syms l0 l1 l2 x1 x2 x3;
l0=4;l1=4;l2=1;
tt(3,100)=0;
for k=1:100
    x1=0;x3=0;x2=2*pi*k/100;
    t3=twist([0,0,1]',[0,l1+l2,0]',0)
    t2=twist([0,0,1]',[0,l1,0]',0)
    t1=twist([0,0,1]',[0,0,0]',0)
    g3=twist2gab(t3,x3)
    g2=twist2gab(t2,x2)
    g1=twist2gab(t1,x1)
    g0=gab(eye(3,3),[0,l1+l2,l0]')
    gst=g1*g2*g3*g0;
    tt(1:3,k)=gst(1:3,4)';
end
plot3(tt(1,:),tt(2,:),tt(3,:),'g*',[0,0],[0,0],[0,l0],'b',[0,0],[0,l1],[l0,l0],'r');
grid on;
xlabel('x(t)')
ylabel('y(t)')
zlabel('z(t)')





