function [a1,a2] = Subproblems2_2NonItrstAxis(q1,q2,pw1,pw2,w1,w2)
% q1:起始点
% q2:末状态点
% p:一轴上一点
% pb:二轴上一点
% wi:轴
% 顺序：先w1后w2
w1=w1/norm(w1);  w2=w2/norm(w2);
r=norm(q2-pw2);
a1=Subproblems3(q1,pw2,pw1,w1,r);
t1=twist_wqh(w1,pw1,0); g1=twist2gab(t1,a1);
q12=(g1*[q1;1]); q12=q12(1:3);
a2=Subproblems1(q12,q2,pw2,w2);
%欧阳俊源 2020/04/13 机器人学导论作业7-1函数1
end

 