function [zyz] = exp2zyz(omegatheta)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
wh=[0,-omegatheta(3),omegatheta(2);omegatheta(3),0,-omegatheta(1);-omegatheta(2),omegatheta(1),0];
%R=exp(wh*omegatheta(4))
R=eye(3,3)+ wh*sin(omegatheta(4))+ wh^2 *(1-cos(omegatheta(4)));
y=atan2(sqrt(R(3,1)^2+R(3,2)^2),R(3,3));
if abs(y-0)>1e-4
    z1=asin(R(3,2)/sin(y));
    z2=asin(R(2,3)/sin(y));
else
    sum=acos(R(1,1));
    z1=sum;
    z2=0;
end
zyz=[z1,y,z2];
end

