clear;
clc;
%inverse elbow manipulator
%更快的方法当然是利用elbow的工具和空间坐标系的变换来直接使用elbow逆解的结论
%这里试试直接使用逆解
%%%%%%%%%机械臂建模
gst0=gab_Rp(eye(3,3),[0;3;0]);
w1=[0;0;1];w2=[0;1;0];w3=[-1;0;0];w4=[-1;0;0];w5=[-1;0;0];w6=[0;1;0];
wq123=[0;0;0];wq4=[0;1;0];wq56=[0;2;0];
%%%%%%%%%正运动学
for_theta=[pi/4;pi/4;pi/4;pi/4;pi/4;pi/4]'
fg1=twist2gab(twist_wqh(w1,wq123,0),for_theta(1));
fg2=twist2gab(twist_wqh(w2,wq123,0),for_theta(2));
fg3=twist2gab(twist_wqh(w3,wq123,0),for_theta(3));
fg4=twist2gab(twist_wqh(w4,wq4,0),for_theta(4));
fg5=twist2gab(twist_wqh(w5,wq56,0),for_theta(5));
fg6=twist2gab(twist_wqh(w6,wq56,0),for_theta(6));
for_gst=fg1*fg2*fg3*fg4*fg5*fg6*gst0;

% %%%%%%%%逆运动学
%%求解theta4
g1=for_gst/gst0;
p=[0;2;0];pb=[0;0;0];g1p=g1*[p;1];g1p=g1p(1:3);
inv_theta(4)=Subproblems3(p,pb,wq4,w4,norm(g1p-pb));
ig4=twist2gab(twist_wqh(w4,wq4,0),inv_theta(4));
%%求解theta5
p=[0;3;0];%属于l6不属于l5的点
g1p=g1*[p;1];g1p=g1p(1:3);
ig4ipb=inv(ig4)*[pb ;1];ig4ipb=ig4ipb(1:3);
inv_theta(5)=Subproblems3(p,ig4ipb,wq56,w5,norm(g1p-pb));
ig5=twist2gab(twist_wqh(w5,wq56,0),inv_theta(5));
%%求解theta6
p=[1;2;0];%不属于l6的点
g1p=g1*[p;1];g1p=g1p(1:3);
ig4ig5ipb=inv(ig4*ig5)*[pb ;1];ig4ig5ipb=ig4ig5ipb(1:3);
inv_theta(6)=Subproblems3(p,ig4ig5ipb,wq56,w6,norm(g1p-pb));
ig6=twist2gab(twist_wqh(w6,wq56,0),inv_theta(6));
%%求解theta1,2
p=[-1;0;0];%属于l3,不属于l1、l2的点
g1g2g3p=g1*inv(ig4*ig5*ig6)*[p;1];g1g2g3p=g1g2g3p(1:3);
[inv_theta(1),inv_theta(2)]=Subproblems2(p,g1g2g3p,wq123,w1,w2);
ig1=twist2gab(twist_wqh(w1,wq123,0),inv_theta(1));
ig2=twist2gab(twist_wqh(w2,wq123,0),inv_theta(2));
%%求解theta3
p=[0;1;0];%不属于l3
g3p=inv(ig1*ig2)*g1*inv(ig4*ig5*ig6)*[p;1];g3p=g3p(1:3);
inv_theta(3)=Subproblems1(p,g3p,wq123,w3)
ig3=twist2gab(twist_wqh(w3,wq123,0),inv_theta(3));
% %%结果对比
for_gst
inv_gst=ig1*ig2*ig3*ig4*ig5*ig6*gst0
rad2deg(for_theta)
rad2deg(inv_theta)
%欧阳俊源 2020/04/16 机器人学导论作业7-3-2 代码