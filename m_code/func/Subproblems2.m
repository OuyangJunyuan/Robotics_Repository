function [a1,a2] = Subproblems2(p,q,r,w1,w2)
%欧阳俊源@2020/04/14
%思路：先找出c点，再化为2个子问题1
%p:原来的位置3x1
%q:目标位置3x1
%r:两轴交点3x1
%w1,w2:两轴3x1
%顺序：先绕w2，再绕w1。
u=p-r;v=q-r;
w1=w1/norm(w1);w2=w2/norm(w2);
t=w1'*w2;
alpha=(t*w2'*u-w1'*v)/(t^2-1);
beta=(t*w1'*v-w2'*u)/(t^2-1);
gamma=sqrt((u'*u-(alpha^2)-(beta^2)-2*alpha*beta*w1'*w2)/(norm(cross(w1,w2))^2));
c=r+alpha*w1+beta*w2+gamma*cross(w1,w2);
a2=Subproblems1(p,c,r,w2);
a1=Subproblems1(c,q,r,w1);
end

