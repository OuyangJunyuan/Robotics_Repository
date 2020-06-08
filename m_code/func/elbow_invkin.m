function [theta] = elbow_invkin(gst)
%假设这里l0 l1 l2 都是1
gst0=gab_Rp(eye(3,3),[0;1+1;1]);
g1=gst/gst0;
qw=[0;1+1;1];pb=[0;0;1];g1qw=g1*[qw;1];
theta(3)=Subproblems3(qw,pb,[0;1;1],[-1;0;0],norm(g1qw-pb));

end

