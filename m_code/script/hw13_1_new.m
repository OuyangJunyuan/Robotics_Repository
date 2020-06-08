clear all;
[p1,t1]=LFPB_ViaPoint([0,2,2],[0,1,2],10,0.001);
[p2,t2]=LFPB_ViaPoint([0,2,2],[0,1,2],10,0.001);
subplot(3,3,4);
plot(p1,p2,'-b');grid on;title('x-y');
axis([-5,5,-5,5]);
subplot(3,3,2);
plot(t1,p1,'-b');grid on;title('x');
subplot(3,3,3);
plot(t2,p2,'-b');grid on;title('y');

subplot(3,3,5);
p1=diff(p1);p2=diff(p2);
plot(t1(2:end),p1,'-b');grid on;title('vx');
subplot(3,3,6);
plot(t2(2:end),p2,'-b');grid on;title('vy');

subplot(3,3,8);
p1=diff(p1);p2=diff(p2);
plot(t1(3:end),p1,'-b');grid on;title('ax');
subplot(3,3,9);
plot(t2(3:end),p2,'-b');grid on;title('ay')

