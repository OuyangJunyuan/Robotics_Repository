function [a1,a2] = Subproblems2(p,q,r,w1,w2)
%ŷ����Դ@2020/04/14
%˼·�����ҳ�c�㣬�ٻ�Ϊ2��������1
%p:ԭ����λ��3x1
%q:Ŀ��λ��3x1
%r:���ύ��3x1
%w1,w2:����3x1
%˳������w2������w1��
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

