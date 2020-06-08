clear all;
clc;
w1=[0;0;1]; w2=[1;0;0]; pw1=[0;1;0];pw2=[0;0;0];
q1=[0;2;0];q2=[-1;0;1];
[a1,a2]=Subproblems2_2NonItrstAxis(q1,q2,pw1,pw2,w1,w2);

plot3([0,0],[1,1],[0,1],'g');
grid on; hold on;
axis([-2,2,-2,2,-1,2]);
plot3([0,w2(1)],[0,w2(2)],[0,w2(3)],'g')

n2=50;n1=50;
for n=1:n1
    q11(:,n)=twist2gab(twist_wqh(w1,pw1,0),a1/n1*n)*[q1;1];
end
for n=1:n2
    q22(:,n)=twist2gab(twist_wqh(w2,pw2,0),a2/n2*n)*q11(:,n1);
end
plot3(q11(1,:),q11(2,:),q11(3,:),'b.');plot3(q22(1,:),q22(2,:),q22(3,:),'b.');
plot3(q1(1),q1(2),q1(3),'r*',q2(1),q2(2),q2(3),'r*')

