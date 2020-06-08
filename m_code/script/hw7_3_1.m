clear all;
clc;
%elbow manipulator
%%%%%%%%%机械臂建模
gst0=gab_Rp(eye(3,3),[0;1+1;1]);
w1=[0;0;1];w2=[-1;0;0];w3=[-1;0;0];w4=[0;0;1];w5=[-1;0;0];w6=[0;1;0];
wq12=[0;0;1];wq3=[0;1;1];wq456=[0;2;1];
%%%%%%%%%正运动学
for_theta=[pi/4;pi/5;pi/6;pi/7;pi/8;pi/9]';
fg1=twist2gab(twist_wqh(w1,wq12,0),for_theta(1));
fg2=twist2gab(twist_wqh(w2,wq12,0),for_theta(2));
fg3=twist2gab(twist_wqh(w3,wq3,0),for_theta(3));
fg4=twist2gab(twist_wqh(w4,wq456,0),for_theta(4));
fg5=twist2gab(twist_wqh(w5,wq456,0),for_theta(5));
fg6=twist2gab(twist_wqh(w6,wq456,0),for_theta(6));
for_gst=fg1*fg2*fg3*fg4*fg5*fg6*gst0;

%%%%%%%%逆运动学
%%求解theta3
g1=for_gst/gst0;
qw=[0;2;1];pb=[0;0;1];g1qw=g1*[qw;1];g1qw=g1qw(1:3);
inv_theta(3)=Subproblems3(qw,pb,wq3,w3,norm(g1qw-pb));
ig3=twist2gab(twist_wqh(w3,wq3,0),inv_theta(3));
%%求解theta1,2
qw12=ig3*[qw;1];
[inv_theta(1),inv_theta(2)]=Subproblems2(qw12(1:3),g1qw,pb,w1,w2);
ig1=twist2gab(twist_wqh(w1,wq12,0),inv_theta(1));
ig2=twist2gab(twist_wqh(w2,wq12,0),inv_theta(2));
%%求解theta4,5
g2=inv(ig3)*inv(ig2)*inv(ig1)*g1;
p6=[0;3;1];g2p6=g2*[p6;1];
[inv_theta(4),inv_theta(5)]=Subproblems2(p6,g2p6(1:3),qw,w4,w5);
ig4=twist2gab(twist_wqh(w4,wq456,0),inv_theta(4));
ig5=twist2gab(twist_wqh(w5,wq456,0),inv_theta(5));
%%求解theta6
g3=inv(ig5)*inv(ig4)*g2;
p4=[0;2;2];g3p4=g3*[p4;1];g3p4=g3p4(1:3);
inv_theta(6)=Subproblems1(p4,g3p4,wq456,w6);
ig6=twist2gab(twist_wqh(w6,wq456,0),inv_theta(6));
%%结果对比
for_gst
inv_gst=ig1*ig2*ig3*ig4*ig5*ig6*gst0
rad2deg(for_theta)
rad2deg(inv_theta)
%欧阳俊源 2020/04/14 机器人学导论作业7-3-1 代码
