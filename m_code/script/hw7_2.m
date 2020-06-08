clear all;
clc;
plot3([0,0],[1,1],[0,1],'g');hold on;grid on;
plot3([0,1],[1,1],[0,0],'g');plot3(0,0,0,'r');
axis([-3,3,-2,3,0,5])
xlabel('x');ylabel('y');zlabel('z');


w1=[0;0;1];w2=[1;0;0];r=[0;1;0];q1=[sqrt(3)/2;1.5;0];q2=[0;2;0];q=[0;0;0];
[a1,a2]=Subproblems4(q,q1,q2,1,1,[0;1;0],w1,w2);
point=[r,q1,q2];
plot3(0,0,0,'r.');plot3(point(1,1),point(2,1),point(3,1),'r*');plot3(point(1,2),point(2,2),point(3,2),'r*');plot3(point(1,3),point(2,3),point(3,3),'r*');
n2=50;n1=10;
for n=1:n2
    q12(:,n)=twist2gab(twist_wqh(w2,r,0),a2/n2*n)*[q;1];
end
for n=1:n1
    q22(:,n)=twist2gab(twist_wqh(w1,r,0),a1/n1*n)*q12(:,n2);
end
plot3(q12(1,:),q12(2,:),q12(3,:),'b.');plot3(q22(1,:),q22(2,:),q22(3,:),'b.');
plot3(q22(1,n1),q22(2,n1),q22(3,n1),'r*');
plot3([q22(1,n1),q1(1)],[q22(2,n1),q1(2)],[q22(3,n1),q1(3)],'r');
plot3([q22(1,n1),q2(1)],[q22(2,n1),q2(2)],[q22(3,n1),q2(3)],'r');


