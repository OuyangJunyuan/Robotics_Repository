function a1 = Subproblems1(q,p,r,w)
%q:初始点
%p：末状态点
%r:轴上一点
%w:轴向
w=w/norm(w);
u=q-r;v=p-r;
u1=u-w*w'*u;v1=v-w*w'*v;
a1=atan2(w'*cross(u1,v1),u1'*v1);
if a1>pi
    a1=a1-2*pi;
elseif a1<-pi
    a1=a1+2*pi;
end
end

