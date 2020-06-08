function [a1,a2] = Subproblems4(p,q1,q2,d1,d2,r,w1,w2)
%欧阳俊源@2020/04/14
%思路：先寻找q,然后用子问题2求解。
%p:初始点
%q1：距离q1点d1距离
%q2：距离q2点d2距离
%r：两轴交点
%w1\w2：先绕w2转，再绕w1转。
q=sphere3itrsect([r';q1';q2'],[norm(p-r),d1,d2]);
[a1,a2]=Subproblems2(p,q,r,w1,w2);
end

