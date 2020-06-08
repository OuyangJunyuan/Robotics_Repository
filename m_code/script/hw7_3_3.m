clear;
clc;

%stanford manipulator
%%%%%%%%%机械臂建模
l1=1;l2=1;
gst0=gab_Rp(eye(3,3),[0;l2;l1]);
w1=[0;0;1];w2=[-1;0;0];w3=[0;1;0];w4=[-1;0;0];w5=[0;0;1];w6=[0;1;0];
wq12=[0;0;l1];wq3456=[0;l2;l1];
%%%%%%%%%正运动学
for_theta=[-pi/5;pi/6;12;pi/7;pi/5;pi/6]';
fg1=twist2gab(twist_wqh(w1,wq12,0),for_theta(1));
fg2=twist2gab(twist_wqh(w2,wq12,0),for_theta(2));
fg3=twist2gab(twist_wqh(w3,wq3456,inf),for_theta(3));
fg4=twist2gab(twist_wqh(w4,wq3456,0),for_theta(4));
fg5=twist2gab(twist_wqh(w5,wq3456,0),for_theta(5));
fg6=twist2gab(twist_wqh(w6,wq3456,0),for_theta(6));
for_gst=fg1*fg2*fg3*fg4*fg5*fg6*gst0;

%%%%%%%%逆运动学
%%求解theta2
g1=for_gst/gst0;
p=for_gst(1:3,4);
inv_theta(2)=atan2(l1-p(3),sqrt(p(1)^2+p(2)^2));
ig2=twist2gab(twist_wqh(w2,wq12,0),inv_theta(2));
%%求解theta3
if sin(inv_theta(2))>cos(inv_theta(2)) %sin比cos大，角度大于45，用大算，防止除数太小，误差太大，也避免了除0。
    inv_theta(3)=(l1-p(3))/sin(inv_theta(2))-l2;
else
    inv_theta(3)=sqrt(p(1)^2+p(2)^2)/cos(inv_theta(2))-l2;
end
ig3=twist2gab(twist_wqh(w3,wq3456,inf),inv_theta(3));
%%求解theta1 ，这里还有点问题，就是theta2=90°的时候无法使用

if cos(inv_theta(2))>1e-10
    inv_theta(1)=-acos(p(2)/(cos(inv_theta(2))*(l2+inv_theta(3))) );
end
ig1=twist2gab(twist_wqh(w1,wq12,0),inv_theta(1));

%%求解theta4,5 
p=[0;0;l1];%属于l6不属于l4,l5的点
ig4g5p=inv(ig1*ig2*ig3)*g1*[p;1];ig4g5p=ig4g5p(1:3);
[inv_theta(4),inv_theta(5)]=Subproblems2(p,ig4g5p,wq3456,w4,w5);
ig4=twist2gab(twist_wqh(w4,wq3456,0),inv_theta(4));
ig5=twist2gab(twist_wqh(w5,wq3456,0),inv_theta(5));
%%求解theta6
p=[0;l1;0];%不属于l6
g6p=inv(ig1*ig2*ig3*ig4*ig5)*g1*[p;1];g6p=g6p(1:3);
inv_theta(6)=Subproblems1(p,g6p,wq3456,w6);
ig6=twist2gab(twist_wqh(w6,wq3456,0),inv_theta(6));
% % %%结果对比
for_gst
inv_gst=ig1*ig2*ig3*ig4*ig5*ig6*gst0
for_theta
inv_theta
%欧阳俊源 2020/04/16 机器人学导论作业7-3-3 代码