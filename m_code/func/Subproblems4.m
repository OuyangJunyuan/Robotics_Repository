function [a1,a2] = Subproblems4(p,q1,q2,d1,d2,r,w1,w2)
%ŷ����Դ@2020/04/14
%˼·����Ѱ��q,Ȼ����������2��⡣
%p:��ʼ��
%q1������q1��d1����
%q2������q2��d2����
%r�����ύ��
%w1\w2������w2ת������w1ת��
q=sphere3itrsect([r';q1';q2'],[norm(p-r),d1,d2]);
[a1,a2]=Subproblems2(p,q,r,w1,w2);
end

