function a1 = Subproblems3(p,q,r,w,delta)
% p:��ʼλ��
% q:����p'һ������ĵ�
% r:w����1��
% w:����
% delta������
nw=norm(w);
w=w/nw;%��λ����
u=p-r;v=q-r;
u1=u-w*w'*u;v1=v-w*w'*v;
theta0=atan2(w'*cross(u1,v1),u1'*v1);

delta1=sqrt(delta^2-norm(w'*(p-q))^2);
lu1=norm(u1);lv1=norm(v1);
if lu1+lv1 <= delta1
    error('r1<u1+v1')
end
phi=acos((lu1^2+lv1^2-delta1^2)/(2*lu1*lv1));
if abs(theta0+phi)>=abs(theta0-phi)
    a1=theta0-phi;
else
    a1=theta0+ph1;
end
end

